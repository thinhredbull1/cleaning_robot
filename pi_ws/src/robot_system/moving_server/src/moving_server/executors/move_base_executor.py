#!/usr/bin/env python3

"""
MODE_MOVE_TO_POINT.

Pure forwarding to move_base: the moving_server stays the single entry
point for motion, move_base keeps doing the planning. Velocity commands
are published by move_base itself on the navigation topic of the mux, so
this executor never publishes a Twist.
"""

import rospy

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal

from moving_server_msgs.msg import MoveCommandFeedback, MoveCommandResult

from moving_server.utils import distance_2d
from moving_server.executors.base import MotionExecutor


class MoveBaseTask:
    """
    Thin non blocking wrapper around the move_base action client, reused
    by the docking sequence to reach the pre-dock pose.
    """

    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"

    def __init__(self, context):

        self.context = context

        self.client = context.move_base_client

        self.target_pose = None

        self.active = False

    def start(self, target_pose):
        """target_pose: geometry_msgs/PoseStamped. Returns (ok, message)."""

        if not self.client.wait_for_server(rospy.Duration(2.0)):

            return False, "move_base action server is not available"

        pose = target_pose

        if not pose.header.frame_id:

            pose.header.frame_id = self.context.global_frame

        if pose.header.frame_id != self.context.global_frame:

            transformed = self.context.transform_pose(
                pose,
                self.context.global_frame,
                timeout=1.0
            )

            if transformed is None:

                return False, "Cannot transform target_pose to {}".format(
                    self.context.global_frame
                )

            pose = transformed

        pose.header.stamp = rospy.Time.now()

        self.target_pose = pose

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.client.send_goal(goal)

        self.active = True

        rospy.loginfo(
            "[moving_server] move_base goal sent: "
            "frame=%s x=%.3f y=%.3f",
            pose.header.frame_id,
            pose.pose.position.x,
            pose.pose.position.y
        )

        return True, "move_base goal sent"

    def update(self):
        """Returns (status, message)."""

        if not self.active:
            return self.FAILED, "move_base task is not active"

        state = self.client.get_state()

        if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):

            return self.RUNNING, "Navigating"

        self.active = False

        if state == GoalStatus.SUCCEEDED:

            return self.SUCCEEDED, "move_base reached the goal"

        status_text = self.client.get_goal_status_text()

        return self.FAILED, (
            "move_base terminated with state {} ({})".format(
                state,
                status_text
            )
        )

    def distance_remaining(self):

        if self.target_pose is None:
            return 0.0

        robot_pose = self.context.get_robot_pose()

        if robot_pose is None:
            return 0.0

        return distance_2d(
            robot_pose.pose.position,
            self.target_pose.pose.position
        )

    def cancel(self):

        if not self.active:
            return

        self.client.cancel_goal()

        self.client.wait_for_result(rospy.Duration(2.0))

        self.active = False


class MoveBaseExecutor(MotionExecutor):

    name = "MOVE_TO_POINT"

    def __init__(self, context):

        super(MoveBaseExecutor, self).__init__(context)

        self.task = MoveBaseTask(context)

        self.initial_distance = 0.0

    # -----------------------------------------------------

    def start(self, goal):

        self.goal = goal

        position = goal.target_pose.pose.position
        orientation = goal.target_pose.pose.orientation

        # An all zero quaternion is an unset pose, not a valid one
        if (
            orientation.x == 0.0
            and orientation.y == 0.0
            and orientation.z == 0.0
            and orientation.w == 0.0
        ):

            return False, "target_pose has an invalid orientation"

        accepted, message = self.task.start(goal.target_pose)

        if not accepted:
            return False, message

        robot_pose = self.context.get_robot_pose()

        if robot_pose is not None:

            self.initial_distance = distance_2d(
                robot_pose.pose.position,
                position
            )

        return True, message

    # -----------------------------------------------------

    def update(self, dt):

        status, message = self.task.update()

        remaining = self.task.distance_remaining()

        if self.initial_distance > 1e-3:

            progress = 1.0 - min(
                1.0,
                remaining / self.initial_distance
            )

        else:

            progress = 0.0

        if status == MoveBaseTask.RUNNING:

            return self.running(
                state=MoveCommandFeedback.STATE_MOVE_BASE,
                state_name="MOVE_BASE",
                message=message,
                distance_remaining=remaining,
                progress=progress
            )

        if status == MoveBaseTask.SUCCEEDED:

            return self.succeeded(
                message=message,
                state=MoveCommandFeedback.STATE_FINISHING,
                state_name="FINISHING",
                progress=1.0
            )

        return self.failed(
            MoveCommandResult.RESULT_MOVE_BASE_FAIL,
            message,
            state=MoveCommandFeedback.STATE_MOVE_BASE,
            state_name="MOVE_BASE",
            distance_remaining=remaining,
            progress=progress
        )

    # -----------------------------------------------------

    def stop(self):

        self.task.cancel()
