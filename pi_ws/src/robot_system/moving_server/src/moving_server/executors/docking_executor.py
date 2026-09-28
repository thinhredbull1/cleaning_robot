#!/usr/bin/env python3

"""
MODE_DOCKING / MODE_UNDOCKING.

The robot is expected to be at the pre-dock area already: getting there
is a separate MODE_MOVE_TO_POINT goal, on purpose. A robot that is
already a few dozen centimetres from the charger has no business making
a move_base plan, and the caller is the one who knows whether that leg
is needed at all.

Docking sequence:

    DETECTING_DOCK  call the dock detection service and freeze the dock
                    pose in the robot frame
         |
    RESET_ODOM      ask the base driver to reset its odometry: the whole
                    approach is then expressed relative to where the
                    robot stood at detection time, free of the drift
                    accumulated during the navigation before
         |
    ALIGNING        leg 1: path from the current pose to the alignment
                    point, `align_distance` (40-50 cm) in front of the
                    charger, oriented parallel to it
         |
    TRAJ_TRACKING   leg 2: straight path from that point into the
                    charger
         |
    DONE

Both legs are followed by the same trajectory tracking controller
(controllers/trajectory_tracking_controller.py), in the odom frame,
publishing on the docking cmd_vel topic. Splitting them keeps the final
run perfectly straight: all the lateral correction is absorbed by leg 1,
so the robot enters the charger without any sideways motion left.

Undocking simply drives straight out of the dock for `undock_distance`.
"""

import math

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from moving_server_msgs.msg import MoveCommandFeedback, MoveCommandResult
from moving_server_msgs.srv import DetectDockRequest

from moving_server.utils import (
    yaw_from_quaternion,
    quaternion_from_yaw,
    normalize_angle,
    to_robot_frame,
    from_robot_frame,
    interpolate_poses,
)
from moving_server.executors.base import MotionExecutor
from moving_server.controllers.trajectory_tracking_controller import (
    TrajectoryTrackingConfig,
    TrajectoryTrackingController,
    TrajectoryTrackingResult,
)


class DockingExecutor(MotionExecutor):

    name = "DOCKING"

    # --- Internal sequence states ---
    DETECTING_DOCK = "DETECTING_DOCK"
    RESET_ODOM = "RESET_ODOM"
    ALIGNING = "ALIGNING"
    TRAJ_TRACKING = "TRAJ_TRACKING"
    UNDOCKING = "UNDOCKING"
    DONE = "DONE"

    def __init__(self, context, undocking=False):

        super(DockingExecutor, self).__init__(context)

        self.undocking = undocking

        docking_params = dict(self.params.get("docking", {}))

        # -------------------------------------------------
        # Geometry
        # -------------------------------------------------

        # Distance from the charger of the alignment point (leg 1 goal)
        self.align_distance = float(
            docking_params.get("align_distance", 0.45)
        )

        # Stop that far short of the dock pose itself (leg 2 goal)
        self.final_offset = float(
            docking_params.get("final_offset", 0.0)
        )

        self.path_resolution = float(
            docking_params.get("path_resolution", 0.05)
        )

        # -------------------------------------------------
        # Detection
        # -------------------------------------------------

        self.detection_retries = int(
            docking_params.get("detection_retries", 3)
        )

        self.detection_timeout = float(
            docking_params.get("detection_timeout", 5.0)
        )

        self.min_confidence = float(
            docking_params.get("min_confidence", 0.5)
        )

        # -------------------------------------------------
        # Odometry reset
        # -------------------------------------------------

        self.reset_odom_required = bool(
            docking_params.get("reset_odom_required", True)
        )

        self.reset_odom_timeout = float(
            docking_params.get("reset_odom_timeout", 3.0)
        )

        # -------------------------------------------------
        # Undocking
        # -------------------------------------------------

        self.undock_distance = float(
            docking_params.get("undock_distance", 0.5)
        )

        self.undock_velocity = float(
            docking_params.get("undock_velocity", 0.1)
        )

        # -------------------------------------------------
        # Controllers: same algorithm, two tunings
        # -------------------------------------------------

        tracking_params = dict(docking_params.get("tracking", {}))

        align_params = dict(tracking_params)
        align_params.update(docking_params.get("align_tracking", {}))

        self.config = TrajectoryTrackingConfig(**tracking_params)

        self.align_config = TrajectoryTrackingConfig(**align_params)

        self.controller = TrajectoryTrackingController(self.config)

        self.align_controller = TrajectoryTrackingController(
            self.align_config
        )

        # -------------------------------------------------
        # Path publishers, for RViz
        #
        # Latched: the paths are generated once per goal, so a display
        # opened later still gets them.
        # -------------------------------------------------

        self.align_path_pub = rospy.Publisher(
            "/moving_server/docking/align_path",
            Path,
            queue_size=1,
            latch=True
        )

        self.approach_path_pub = rospy.Publisher(
            "/moving_server/docking/approach_path",
            Path,
            queue_size=1,
            latch=True
        )

        # The detected charger and the two targets, same frame
        self.dock_pose_pub = rospy.Publisher(
            "/moving_server/docking/dock_pose",
            PoseStamped,
            queue_size=1,
            latch=True
        )

        self.align_target_pub = rospy.Publisher(
            "/moving_server/docking/align_target",
            PoseStamped,
            queue_size=1,
            latch=True
        )

        self.final_target_pub = rospy.Publisher(
            "/moving_server/docking/final_target",
            PoseStamped,
            queue_size=1,
            latch=True
        )

        # -------------------------------------------------
        # Runtime
        # -------------------------------------------------

        self.state = self.DONE

        self.dock_id = ""

        # Dock pose frozen in the robot frame at detection time
        self.dock_in_base = None

        # Targets and paths, in the odom frame
        self.dock_in_odom = None
        self.align_target = None
        self.final_target = None
        self.align_path = None
        self.approach_path = None

        self.undock_start_pose = None

    # =====================================================
    # Lifecycle
    # =====================================================

    def start(self, goal):

        self.goal = goal

        self.dock_id = (
            goal.dock_id.strip()
            or self.params.get("default_dock_id", "")
        )

        self.dock_in_base = None
        self.dock_in_odom = None
        self.align_target = None
        self.final_target = None
        self.align_path = None
        self.approach_path = None

        self.controller.reset()
        self.align_controller.reset()

        if self.context.get_odom_pose() is None:

            return False, (
                "No odometry on {}, the docking approach needs it"
                .format(self.context.odom_topic)
            )

        # -------------------------------------------------
        # Undocking: nothing to plan, just drive out
        # -------------------------------------------------

        if self.undocking:

            self.undock_start_pose = self.context.get_odom_pose()

            self.state = self.UNDOCKING

            return True, "Undocking started"

        # -------------------------------------------------
        # Docking: the robot is already at the pre-dock area,
        # start by looking for the charger
        # -------------------------------------------------

        self.state = self.DETECTING_DOCK

        rospy.loginfo(
            "[moving_server] Docking sequence started, dock_id=%s",
            self.dock_id or "<default>"
        )

        return True, "Docking started, detecting the charger"

    def stop(self):

        self.context.stop_docking()

        self.clear_visualization()

        self.state = self.DONE

    # =====================================================
    # Geometry
    # =====================================================

    def docked_yaw(self, dock_yaw):
        """
        Orientation the robot holds during the whole approach.

        Reverse docking: the robot keeps its back to the charger, so its
        yaw equals the outward yaw of the dock. Forward docking: it
        faces the charger, half a turn away.
        """

        if self.config.reverse:
            return normalize_angle(dock_yaw)

        return normalize_angle(dock_yaw + math.pi)

    # =====================================================
    # Dock detection
    # =====================================================

    def detect_dock(self):
        """
        Call the detection service and freeze the dock pose in the robot
        frame. It has to be captured *before* the odometry reset, and in
        the robot frame, because that is the only frame both sides of
        the reset agree on.

        Returns (ok, message).
        """

        proxy = self.context.get_detect_dock_proxy(
            timeout=self.detection_timeout
        )

        if proxy is None:

            return False, (
                "Dock detection service {} is unavailable".format(
                    self.context.detect_dock_service
                )
            )

        request = DetectDockRequest()
        request.dock_id = self.dock_id

        for attempt in range(1, self.detection_retries + 1):

            try:

                response = proxy(request)

            except rospy.ServiceException as error:

                rospy.logwarn(
                    "[moving_server] Dock detection call failed "
                    "(%d/%d): %s",
                    attempt,
                    self.detection_retries,
                    str(error)
                )

                continue

            if not response.success:

                rospy.logwarn(
                    "[moving_server] Dock not detected (%d/%d): %s",
                    attempt,
                    self.detection_retries,
                    response.message
                )

                rospy.sleep(0.5)

                continue

            if response.confidence < self.min_confidence:

                rospy.logwarn(
                    "[moving_server] Detection confidence too low "
                    "(%.2f < %.2f)",
                    response.confidence,
                    self.min_confidence
                )

                rospy.sleep(0.5)

                continue

            dock_in_base = self.pose_in_base_frame(response.dock_pose)

            if dock_in_base is None:

                return False, (
                    "Cannot transform the detected dock pose into {}"
                    .format(self.context.base_frame)
                )

            self.dock_in_base = dock_in_base

            return True, "Dock detected (confidence {:.2f})".format(
                response.confidence
            )

        return False, "Dock detection failed after {} attempts".format(
            self.detection_retries
        )

    def pose_in_base_frame(self, pose_stamped):
        """PoseStamped -> (x, y, yaw) in the base frame, None on failure."""

        pose = PoseStamped()

        pose.header.frame_id = (
            pose_stamped.header.frame_id or self.context.base_frame
        )
        pose.header.stamp = rospy.Time(0)
        pose.pose = pose_stamped.pose

        if pose.header.frame_id != self.context.base_frame:

            transformed = self.context.transform_pose(
                pose,
                self.context.base_frame,
                timeout=1.0
            )

            if transformed is None:
                return None

            pose = transformed

        return (
            pose.pose.position.x,
            pose.pose.position.y,
            normalize_angle(
                yaw_from_quaternion(pose.pose.orientation)
            )
        )

    # =====================================================
    # Target and path generation (odom frame)
    # =====================================================

    def build_paths(self):
        """
        Build the two legs of the approach, in the odom frame, from the
        dock pose frozen in the robot frame.

        Returns (ok, message).
        """

        robot_pose = self.context.get_odom_pose()

        if robot_pose is None:

            return False, "No odometry after the reset"

        # Dock pose in the (freshly reset) odom frame
        self.dock_in_odom = from_robot_frame(
            robot_pose,
            self.dock_in_base
        )

        dock_x, dock_y, dock_yaw = self.dock_in_odom

        # Outward axis of the charger
        axis_x = math.cos(dock_yaw)
        axis_y = math.sin(dock_yaw)

        docked_yaw = self.docked_yaw(dock_yaw)

        # -------------------------------------------------
        # Target 1: aligned, 40-50 cm in front of the charger
        # -------------------------------------------------

        self.align_target = (
            dock_x + self.align_distance * axis_x,
            dock_y + self.align_distance * axis_y,
            docked_yaw
        )

        # -------------------------------------------------
        # Target 2: the charger itself
        # -------------------------------------------------

        self.final_target = (
            dock_x + self.final_offset * axis_x,
            dock_y + self.final_offset * axis_y,
            docked_yaw
        )

        # -------------------------------------------------
        # Leg 1: current pose -> alignment point
        # Leg 2: alignment point -> charger, perfectly straight
        # -------------------------------------------------

        self.align_path = interpolate_poses(
            robot_pose,
            self.align_target,
            self.path_resolution
        )

        self.approach_path = interpolate_poses(
            self.align_target,
            self.final_target,
            self.path_resolution
        )

        self.publish_visualization()

        return True, (
            "Approach planned: align at {:.2f} m, entry over {:.2f} m"
            .format(
                self.align_distance,
                max(self.align_distance - self.final_offset, 0.0)
            )
        )

    # =====================================================
    # Visualization
    # =====================================================

    def publish_visualization(self):
        """Publish the plan for RViz: both legs and the three poses."""

        self.align_path_pub.publish(
            self.make_path(self.align_path)
        )

        self.approach_path_pub.publish(
            self.make_path(self.approach_path)
        )

        self.dock_pose_pub.publish(
            self.make_pose_stamped(self.dock_in_odom)
        )

        self.align_target_pub.publish(
            self.make_pose_stamped(self.align_target)
        )

        self.final_target_pub.publish(
            self.make_pose_stamped(self.final_target)
        )

    def clear_visualization(self):
        """Empty everything, so RViz does not keep a stale plan."""

        empty_path = Path()
        empty_path.header.frame_id = self.context.odom_frame
        empty_path.header.stamp = rospy.Time.now()

        self.align_path_pub.publish(empty_path)
        self.approach_path_pub.publish(empty_path)

    def make_path(self, poses):

        path = Path()

        path.header.frame_id = self.context.odom_frame
        path.header.stamp = rospy.Time.now()

        path.poses = [
            self.make_pose_stamped(pose)
            for pose in poses
        ]

        return path

    def make_pose_stamped(self, pose):
        """(x, y, yaw) in the odom frame -> PoseStamped."""

        x, y, yaw = pose

        stamped = PoseStamped()

        stamped.header.frame_id = self.context.odom_frame
        stamped.header.stamp = rospy.Time.now()

        stamped.pose.position.x = x
        stamped.pose.position.y = y

        quaternion = quaternion_from_yaw(yaw)

        stamped.pose.orientation.x = quaternion[0]
        stamped.pose.orientation.y = quaternion[1]
        stamped.pose.orientation.z = quaternion[2]
        stamped.pose.orientation.w = quaternion[3]

        return stamped

    def path_in_robot_frame(self, path):
        """Odom frame path -> robot frame, None if odometry is missing."""

        robot_pose = self.context.get_odom_pose()

        if robot_pose is None:
            return None

        return [
            to_robot_frame(robot_pose, pose)
            for pose in path
        ]

    # =====================================================
    # Control cycle
    # =====================================================

    def update(self, dt):

        if self.state == self.DETECTING_DOCK:
            return self.update_detecting()

        if self.state == self.RESET_ODOM:
            return self.update_reset_odom()

        if self.state == self.ALIGNING:
            return self.update_leg(
                self.align_controller,
                self.align_path,
                MoveCommandFeedback.STATE_ALIGNING,
                "ALIGNING",
                "Moving to the alignment point",
                0.3,
                0.7,
                dt
            )

        if self.state == self.TRAJ_TRACKING:
            return self.update_leg(
                self.controller,
                self.approach_path,
                MoveCommandFeedback.STATE_TRAJ_TRACKING,
                "TRAJ_TRACKING",
                "Entering the charger",
                0.7,
                1.0,
                dt
            )

        if self.state == self.UNDOCKING:
            return self.update_undocking(dt)

        return self.failed(
            MoveCommandResult.RESULT_ABORTED,
            "Docking executor is in an unexpected state"
        )

    # -----------------------------------------------------
    # Step 1: detection
    # -----------------------------------------------------

    def update_detecting(self):

        detected, message = self.detect_dock()

        if not detected:

            return self.failed(
                MoveCommandResult.RESULT_DETECTION_FAIL,
                message,
                state=MoveCommandFeedback.STATE_DETECTING_DOCK,
                state_name="DETECTING_DOCK",
                progress=0.0
            )

        rospy.loginfo(
            "[moving_server] %s, at x=%.3f y=%.3f yaw=%.3f "
            "in the robot frame",
            message,
            self.dock_in_base[0],
            self.dock_in_base[1],
            self.dock_in_base[2]
        )

        self.state = self.RESET_ODOM

        return self.running(
            state=MoveCommandFeedback.STATE_RESET_ODOM,
            state_name="RESET_ODOM",
            message=message,
            progress=0.15
        )

    # -----------------------------------------------------
    # Step 3: odometry reset, then plan both legs
    # -----------------------------------------------------

    def update_reset_odom(self):

        reset, message = self.context.reset_odometry(
            timeout=self.reset_odom_timeout
        )

        if not reset:

            if self.reset_odom_required:

                return self.failed(
                    MoveCommandResult.RESULT_ODOM_FAIL,
                    message,
                    state=MoveCommandFeedback.STATE_RESET_ODOM,
                    state_name="RESET_ODOM",
                    progress=0.15
                )

            rospy.logwarn(
                "[moving_server] %s, continuing on the current "
                "odometry",
                message
            )

        elif not self.context.wait_for_odometry(
            timeout=self.reset_odom_timeout
        ):

            return self.failed(
                MoveCommandResult.RESULT_ODOM_FAIL,
                "No odometry received after the reset",
                state=MoveCommandFeedback.STATE_RESET_ODOM,
                state_name="RESET_ODOM",
                progress=0.15
            )

        planned, plan_message = self.build_paths()

        if not planned:

            return self.failed(
                MoveCommandResult.RESULT_ODOM_FAIL,
                plan_message,
                state=MoveCommandFeedback.STATE_RESET_ODOM,
                state_name="RESET_ODOM",
                progress=0.15
            )

        rospy.loginfo("[moving_server] %s", plan_message)

        self.align_controller.reset()

        self.state = self.ALIGNING

        return self.running(
            state=MoveCommandFeedback.STATE_ALIGNING,
            state_name="ALIGNING",
            message=plan_message,
            progress=0.5
        )

    # -----------------------------------------------------
    # Steps 4 and 5: the two tracking legs
    # -----------------------------------------------------

    def update_leg(
        self,
        controller,
        path,
        state,
        state_name,
        message,
        progress_start,
        progress_end,
        dt
    ):

        robot_path = self.path_in_robot_frame(path)

        if robot_path is None:

            self.context.stop_docking()

            return self.running(
                state=state,
                state_name=state_name,
                message="Waiting for the odometry",
                progress=progress_start
            )

        linear_x, linear_y, angular, status = controller.compute(
            robot_path,
            dt,
            self.context.get_odom_velocity()
        )

        if status == TrajectoryTrackingResult.REACHED:

            self.context.stop_docking()

            return self.on_leg_reached(state_name)

        if status != TrajectoryTrackingResult.RUNNING:

            self.context.stop_docking()

            return self.failed(
                MoveCommandResult.RESULT_TRACKING_FAIL,
                "{} aborted: {} (cross track error {:.3f} m)".format(
                    state_name,
                    status,
                    controller.cross_track_error
                ),
                state=state,
                state_name=state_name
            )

        self.context.publish_docking_command(
            linear_x,
            linear_y,
            angular
        )

        travelled = self.travelled_ratio(controller, path)

        return self.running(
            state=state,
            state_name=state_name,
            message=message,
            distance_remaining=controller.distance_remaining,
            heading_error=controller.heading_error,
            progress=(
                progress_start
                + (progress_end - progress_start) * travelled
            )
        )

    def on_leg_reached(self, state_name):

        if self.state == self.ALIGNING:

            rospy.loginfo(
                "[moving_server] Alignment point reached, "
                "entering the charger"
            )

            self.controller.reset()

            self.state = self.TRAJ_TRACKING

            return self.running(
                state=MoveCommandFeedback.STATE_TRAJ_TRACKING,
                state_name="TRAJ_TRACKING",
                message="Aligned with the charger",
                progress=0.8
            )

        self.state = self.DONE

        return self.succeeded(
            message="Docked",
            state=MoveCommandFeedback.STATE_FINISHING,
            state_name="FINISHING",
            progress=1.0
        )

    @staticmethod
    def travelled_ratio(controller, path):

        total = len(path) - 1

        if total <= 0:
            return 0.0

        return min(
            1.0,
            float(controller.nearest_index) / float(total)
        )

    # -----------------------------------------------------
    # Undocking
    # -----------------------------------------------------

    def update_undocking(self, dt):

        robot_pose = self.context.get_odom_pose()

        if robot_pose is None:

            self.context.stop_docking()

            return self.running(
                state=MoveCommandFeedback.STATE_UNDOCKING,
                state_name="UNDOCKING",
                message="Waiting for the odometry"
            )

        travelled = math.hypot(
            robot_pose[0] - self.undock_start_pose[0],
            robot_pose[1] - self.undock_start_pose[1]
        )

        remaining = self.undock_distance - travelled

        if remaining <= 0.0:

            self.context.stop_docking()

            self.state = self.DONE

            return self.succeeded(
                message="Undocked ({:.2f} m)".format(travelled),
                state=MoveCommandFeedback.STATE_FINISHING,
                state_name="FINISHING",
                progress=1.0
            )

        # Docking is done in reverse, so undocking drives forward.
        # Pure translation along x, the heading is left untouched: on a
        # mecanum base there is no need to turn around to leave the dock.
        velocity = self.undock_velocity

        if not self.config.reverse:
            velocity = -velocity

        self.context.publish_docking_command(velocity, 0.0, 0.0)

        return self.running(
            state=MoveCommandFeedback.STATE_UNDOCKING,
            state_name="UNDOCKING",
            message="Leaving the dock",
            distance_remaining=remaining,
            progress=min(
                1.0,
                travelled / max(self.undock_distance, 1e-3)
            )
        )
