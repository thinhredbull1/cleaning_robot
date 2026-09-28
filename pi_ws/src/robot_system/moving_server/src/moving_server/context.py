#!/usr/bin/env python3

"""
Shared runtime context handed to every executor.

Owns the resources that must exist exactly once in the node: tf buffer,
odometry input, cmd_vel publishers, the move_base action client and the
service proxies.
"""

import threading

import rospy
import tf2_ros
import actionlib

# Registers PoseStamped & co with the tf2 buffer. Imported for that side
# effect only: without it, tf_buffer.transform() raises
# "Type ... is not loaded or supported".
import tf2_geometry_msgs  # noqa: F401

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction
from std_srvs.srv import Empty, Trigger

from moving_server_msgs.srv import DetectDock

from moving_server.utils import yaw_from_quaternion


class ServerContext:

    def __init__(self, params):

        self.params = params

        # -------------------------------------------------
        # Frames
        # -------------------------------------------------

        # global_frame (map) is only used for the move_base legs.
        #
        # The closed loop controllers run in the odom frame instead: it
        # is continuous and drift free over the short distances they
        # cover, unlike map which jumps whenever the localisation
        # corrects itself.

        self.global_frame = params["global_frame"]
        self.odom_frame = params["odom_frame"]
        self.base_frame = params["base_frame"]

        # -------------------------------------------------
        # Odometry
        #
        # Read from the topic, not from TF: the base driver can reset
        # its odometry (see reset_odometry) and the topic is what
        # carries the new origin, together with the measured velocity.
        # -------------------------------------------------

        self.odom_topic = params["odom_topic"]

        self.odom_timeout = float(params.get("odom_timeout", 0.5))

        self.odom_lock = threading.Lock()

        self.odometry = None
        self.odometry_stamp = rospy.Time(0)

        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=1
        )

        # -------------------------------------------------
        # TF
        # -------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer(
            rospy.Duration(10.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        # -------------------------------------------------
        # Velocity publishers
        #
        # One topic per behaviour so that cmd_vel_mux keeps
        # arbitrating by priority (see cmd_vel_mux/config/mux.yaml).
        # -------------------------------------------------

        self.tracking_cmd_pub = rospy.Publisher(
            params["tracking_cmd_vel_topic"],
            Twist,
            queue_size=1
        )

        self.docking_cmd_pub = rospy.Publisher(
            params["docking_cmd_vel_topic"],
            Twist,
            queue_size=1
        )

        # -------------------------------------------------
        # move_base client
        # -------------------------------------------------

        self.move_base_client = actionlib.SimpleActionClient(
            params["move_base_action"],
            MoveBaseAction
        )

        # -------------------------------------------------
        # Dock detection service
        # -------------------------------------------------

        self.detect_dock_service = params["detect_dock_service"]

        self.detect_dock_proxy = None

        # -------------------------------------------------
        # Odometry reset service
        #
        # Provided by the base driver. Called once the dock has been
        # detected, so the whole approach is expressed relative to a
        # fresh origin instead of an odometry that drifted during the
        # navigation that came before.
        # -------------------------------------------------

        self.reset_odom_service = params.get("reset_odom_service", "")

        # "trigger" -> std_srvs/Trigger, "empty" -> std_srvs/Empty
        self.reset_odom_service_type = params.get(
            "reset_odom_service_type",
            "trigger"
        )

    # =====================================================
    # Startup
    # =====================================================

    def wait_for_move_base(self, timeout):

        rospy.loginfo(
            "[moving_server] Waiting for move_base action server: %s",
            self.params["move_base_action"]
        )

        available = self.move_base_client.wait_for_server(
            rospy.Duration(timeout)
        )

        if not available:

            rospy.logwarn(
                "[moving_server] "
                "move_base is not available yet, "
                "MOVE_TO_POINT goals will be rejected until it is"
            )

        return available

    def get_detect_dock_proxy(self, timeout=5.0):
        """Lazily connect to the dock detection service."""

        try:

            rospy.wait_for_service(
                self.detect_dock_service,
                timeout=timeout
            )

        except rospy.ROSException:

            return None

        if self.detect_dock_proxy is None:

            self.detect_dock_proxy = rospy.ServiceProxy(
                self.detect_dock_service,
                DetectDock
            )

        return self.detect_dock_proxy

    # =====================================================
    # Odometry
    # =====================================================

    def odom_callback(self, message):

        with self.odom_lock:

            self.odometry = message

            self.odometry_stamp = rospy.Time.now()

    def get_odom_pose(self, quiet=False):
        """
        Robot pose (x, y, yaw) in the odom frame, None when the odometry
        is missing or stale. This is the reference of every closed loop
        controller.

        quiet: do not complain about a missing message. Used by
        wait_for_odometry(), which polls right after a reset emptied the
        cache on purpose and where one empty cycle is expected.
        """

        with self.odom_lock:

            odometry = self.odometry

            stamp = self.odometry_stamp

        if odometry is None:

            if not quiet:

                rospy.logwarn_throttle(
                    2.0,
                    "[moving_server] No odometry received yet on %s",
                    self.odom_topic
                )

            return None

        if (rospy.Time.now() - stamp).to_sec() > self.odom_timeout:

            if not quiet:

                rospy.logwarn_throttle(
                    2.0,
                    "[moving_server] Odometry on %s is older than %.2f s",
                    self.odom_topic,
                    self.odom_timeout
                )

            return None

        pose = odometry.pose.pose

        return (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation)
        )

    def get_odom_velocity(self):
        """Measured (vx, vy, w) in the robot frame, None if unavailable."""

        with self.odom_lock:

            odometry = self.odometry

        if odometry is None:
            return None

        twist = odometry.twist.twist

        return (
            twist.linear.x,
            twist.linear.y,
            twist.angular.z
        )

    def reset_odometry(self, timeout=3.0):
        """
        Ask the base driver to reset its odometry, so that the current
        pose becomes the origin. Returns (ok, message).

        Called once the dock has been detected: the whole approach is
        then expressed relative to where the robot stood at detection
        time, instead of an odometry that drifted during the navigation
        that came before.
        """

        if not self.reset_odom_service:

            return False, "No reset_odom_service configured"

        try:

            rospy.wait_for_service(
                self.reset_odom_service,
                timeout=timeout
            )

        except rospy.ROSException:

            return False, (
                "Odometry reset service {} is unavailable".format(
                    self.reset_odom_service
                )
            )

        try:

            if self.reset_odom_service_type == "empty":

                rospy.ServiceProxy(
                    self.reset_odom_service,
                    Empty
                )()

                message = "Odometry reset"

            else:

                response = rospy.ServiceProxy(
                    self.reset_odom_service,
                    Trigger
                )()

                if not response.success:

                    return False, "Odometry reset refused: {}".format(
                        response.message
                    )

                message = response.message or "Odometry reset"

        except rospy.ServiceException as error:

            return False, "Odometry reset failed: {}".format(str(error))

        # Drop the cached message, it still holds the pre-reset pose
        with self.odom_lock:

            self.odometry = None
            self.odometry_stamp = rospy.Time(0)

        rospy.loginfo("[moving_server] %s", message)

        return True, message

    def wait_for_odometry(self, timeout=3.0):
        """Block until a fresh odometry message arrives, e.g. after a reset."""

        deadline = rospy.Time.now() + rospy.Duration(timeout)

        rate = rospy.Rate(50.0)

        while rospy.Time.now() < deadline and not rospy.is_shutdown():

            if self.get_odom_pose(quiet=True) is not None:
                return True

            rate.sleep()

        rospy.logwarn(
            "[moving_server] No odometry on %s within %.1f s",
            self.odom_topic,
            timeout
        )

        return False

    # =====================================================
    # Velocity output
    # =====================================================

    def publish_tracking_command(self, linear_x, linear_y, angular):

        self.tracking_cmd_pub.publish(
            self._make_twist(linear_x, linear_y, angular)
        )

    def publish_docking_command(self, linear_x, linear_y, angular):

        self.docking_cmd_pub.publish(
            self._make_twist(linear_x, linear_y, angular)
        )

    def stop_tracking(self):

        self.tracking_cmd_pub.publish(Twist())

    def stop_docking(self):

        self.docking_cmd_pub.publish(Twist())

    def stop_all(self):

        self.stop_tracking()
        self.stop_docking()

    @staticmethod
    def _make_twist(linear_x, linear_y, angular):
        """Mecanum command: x, y and yaw are all commandable."""

        command = Twist()

        command.linear.x = linear_x
        command.linear.y = linear_y
        command.angular.z = angular

        return command

    # =====================================================
    # TF helpers
    # =====================================================

    def transform_pose(self, pose_stamped, target_frame, timeout=0.2):
        """Transform a PoseStamped, returns None on failure."""

        try:

            return self.tf_buffer.transform(
                pose_stamped,
                target_frame,
                rospy.Duration(timeout)
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException
        ) as error:

            rospy.logwarn_throttle(
                2.0,
                "[moving_server] TF %s -> %s failed: %s",
                pose_stamped.header.frame_id,
                target_frame,
                str(error)
            )

            return None

    def get_robot_pose(self):
        """Robot pose in the global frame, None on failure."""

        pose = PoseStamped()

        pose.header.frame_id = self.base_frame
        pose.header.stamp = rospy.Time(0)
        pose.pose.orientation.w = 1.0

        return self.transform_pose(
            pose,
            self.global_frame,
            timeout=0.5
        )

    def get_robot_yaw(self):

        pose = self.get_robot_pose()

        if pose is None:
            return None

        return yaw_from_quaternion(pose.pose.orientation)
