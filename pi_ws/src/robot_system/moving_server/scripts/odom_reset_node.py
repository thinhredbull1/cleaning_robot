#!/usr/bin/env python3

"""
Resettable odometry relay.

Some odometry sources cannot be reset: Gazebo, or a base driver without
such a service. This node sits between them and moving_server:

    /odom (gazebo)  ->  odom_reset_node  ->  /odom_reset
                              ^
                              |
                        /reset_odom (std_srvs/Trigger)

On reset it stores the current pose as the new origin and republishes
every incoming message relative to it, so the docking approach starts
from (0, 0, 0) exactly as it would with a real reset. The velocities are
passed through untouched: they are already expressed in the robot frame,
which the reset does not move.

The output lives in its own frame (`output_frame`), broadcast as a child
of the input odom frame, so RViz can display the docking paths without
any TF conflict with the odometry of the simulator.

Point moving_server at it:

    odom_topic: /odom_reset
    odom_frame: odom_reset
    reset_odom_service: /reset_odom
"""

import threading

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse

from moving_server.utils import (
    yaw_from_quaternion,
    quaternion_from_yaw,
    to_robot_frame,
)


class OdomResetRelay:

    def __init__(self):

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        self.input_topic = rospy.get_param("~input_topic", "/odom")

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/odom_reset"
        )

        self.reset_service_name = rospy.get_param(
            "~reset_service",
            "/reset_odom"
        )

        self.output_frame = rospy.get_param(
            "~output_frame",
            "odom_reset"
        )

        # Broadcast input_frame -> output_frame, for RViz
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.tf_rate = rospy.get_param("~tf_rate", 20.0)

        # Reset once at start-up, so the frames coincide from the
        # beginning instead of after the first service call
        self.reset_on_start = rospy.get_param("~reset_on_start", True)

        # -------------------------------------------------
        # State
        # -------------------------------------------------

        self.lock = threading.Lock()

        # Pose of the output frame in the input frame
        self.origin = (0.0, 0.0, 0.0)

        self.input_frame = ""

        self.last_pose = None

        self.pending_reset = self.reset_on_start

        # -------------------------------------------------
        # Interfaces
        # -------------------------------------------------

        self.odom_pub = rospy.Publisher(
            self.output_topic,
            Odometry,
            queue_size=10
        )

        self.odom_sub = rospy.Subscriber(
            self.input_topic,
            Odometry,
            self.odom_callback,
            queue_size=10
        )

        self.reset_srv = rospy.Service(
            self.reset_service_name,
            Trigger,
            self.handle_reset
        )

        # Same reset, for drivers whose clients expect std_srvs/Empty
        self.reset_empty_srv = rospy.Service(
            self.reset_service_name + "_empty",
            Empty,
            self.handle_reset_empty
        )

        if self.publish_tf:

            self.tf_broadcaster = tf2_ros.TransformBroadcaster()

            self.tf_timer = rospy.Timer(
                rospy.Duration(1.0 / self.tf_rate),
                self.publish_transform
            )

        rospy.loginfo(
            "[odom_reset] %s -> %s (frame %s), reset service: %s",
            self.input_topic,
            self.output_topic,
            self.output_frame,
            self.reset_service_name
        )

    # =====================================================
    # Relay
    # =====================================================

    def odom_callback(self, message):

        pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw_from_quaternion(message.pose.pose.orientation)
        )

        with self.lock:

            self.input_frame = message.header.frame_id

            self.last_pose = pose

            # A reset asked for between two messages takes effect here,
            # on the first pose that follows it
            if self.pending_reset:

                self.origin = pose

                self.pending_reset = False

                rospy.loginfo(
                    "[odom_reset] New origin: x=%.3f y=%.3f yaw=%.3f",
                    pose[0],
                    pose[1],
                    pose[2]
                )

            origin = self.origin

        relative = to_robot_frame(origin, pose)

        output = Odometry()

        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_frame
        output.child_frame_id = message.child_frame_id

        output.pose.pose.position.x = relative[0]
        output.pose.pose.position.y = relative[1]
        output.pose.pose.position.z = message.pose.pose.position.z

        quaternion = quaternion_from_yaw(relative[2])

        output.pose.pose.orientation.x = quaternion[0]
        output.pose.pose.orientation.y = quaternion[1]
        output.pose.pose.orientation.z = quaternion[2]
        output.pose.pose.orientation.w = quaternion[3]

        output.pose.covariance = message.pose.covariance

        # Velocities are in the robot frame, the reset does not touch them
        output.twist = message.twist

        self.odom_pub.publish(output)

    # =====================================================
    # Reset
    # =====================================================

    def handle_reset(self, request):

        with self.lock:

            if self.last_pose is None:

                return TriggerResponse(
                    False,
                    "No odometry received yet on {}".format(
                        self.input_topic
                    )
                )

            # Applied on the next incoming message, so the pose and the
            # message that carries it always match
            self.pending_reset = True

        return TriggerResponse(True, "Odometry reset")

    def handle_reset_empty(self, request):

        self.handle_reset(request)

        return EmptyResponse()

    # =====================================================
    # TF
    # =====================================================

    def publish_transform(self, event):

        with self.lock:

            origin = self.origin

            input_frame = self.input_frame

        if not input_frame:
            return

        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = input_frame
        transform.child_frame_id = self.output_frame

        transform.transform.translation.x = origin[0]
        transform.transform.translation.y = origin[1]

        quaternion = quaternion_from_yaw(origin[2])

        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(transform)


def main():

    rospy.init_node("odom_reset")

    OdomResetRelay()

    rospy.spin()


if __name__ == "__main__":
    main()
