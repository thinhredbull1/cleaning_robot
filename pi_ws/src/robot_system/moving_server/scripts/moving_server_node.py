#!/usr/bin/env python3

"""
moving_server: single motion gateway of the robot.

Exposes one action, /moving_server/move_command, whose mode selects the
executor that will run:

    MODE_MOVE_TO_POINT -> forwarded to move_base
    MODE_TRACK_PERSON  -> point-to-point tracking of a person topic
    MODE_DOCKING       -> pre-dock (move_base) + detection + trajectory
                          tracking
    MODE_UNDOCKING     -> straight move out of the dock

Only one goal runs at a time: a new goal preempts the running one.
"""

import rospy
import actionlib

from std_srvs.srv import Trigger, TriggerResponse

from moving_server_msgs.msg import (
    MoveCommandAction,
    MoveCommandGoal,
    MoveCommandResult,
    MoveCommandFeedback,
    MovingServerState,
)

from moving_server.context import ServerContext
from moving_server.executors.base import ExecutorStatus
from moving_server.executors.move_base_executor import MoveBaseExecutor
from moving_server.executors.track_person_executor import (
    TrackPersonExecutor,
)
from moving_server.executors.docking_executor import DockingExecutor


MODE_NAMES = {
    MoveCommandGoal.MODE_MOVE_TO_POINT: "MOVE_TO_POINT",
    MoveCommandGoal.MODE_TRACK_PERSON: "TRACK_PERSON",
    MoveCommandGoal.MODE_DOCKING: "DOCKING",
    MoveCommandGoal.MODE_UNDOCKING: "UNDOCKING",
}


class MovingServer:

    def __init__(self):

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        self.params = {

            "global_frame": rospy.get_param(
                "~global_frame",
                "map"
            ),

            "odom_frame": rospy.get_param(
                "~odom_frame",
                "odom"
            ),

            "base_frame": rospy.get_param(
                "~base_frame",
                "base_footprint"
            ),

            "odom_topic": rospy.get_param(
                "~odom_topic",
                "/odom"
            ),

            "odom_timeout": rospy.get_param(
                "~odom_timeout",
                0.5
            ),

            "reset_odom_service": rospy.get_param(
                "~reset_odom_service",
                "/reset_odom"
            ),

            # "trigger" -> std_srvs/Trigger, "empty" -> std_srvs/Empty
            "reset_odom_service_type": rospy.get_param(
                "~reset_odom_service_type",
                "trigger"
            ),

            "control_rate": rospy.get_param(
                "~control_rate",
                20.0
            ),

            "default_timeout": rospy.get_param(
                "~default_timeout",
                300.0
            ),

            "move_base_action": rospy.get_param(
                "~move_base_action",
                "/move_base"
            ),

            "person_pose_topic": rospy.get_param(
                "~person_pose_topic",
                "/person_tracker/pose"
            ),

            "detect_dock_service": rospy.get_param(
                "~detect_dock_service",
                "/dock_detector/detect_dock"
            ),

            "tracking_cmd_vel_topic": rospy.get_param(
                "~tracking_cmd_vel_topic",
                "/cmd_vel/tracking"
            ),

            "docking_cmd_vel_topic": rospy.get_param(
                "~docking_cmd_vel_topic",
                "/cmd_vel/docking"
            ),

            "default_dock_id": rospy.get_param(
                "~default_dock_id",
                ""
            ),

            "person_tracking": rospy.get_param(
                "~person_tracking",
                {}
            ),

            "docking": rospy.get_param(
                "~docking",
                {}
            ),

            "docks": rospy.get_param(
                "~docks",
                {}
            ),
        }

        self.control_rate = float(self.params["control_rate"])

        self.default_timeout = float(self.params["default_timeout"])

        # -------------------------------------------------
        # Context and executors
        # -------------------------------------------------

        self.context = ServerContext(self.params)

        self.context.wait_for_move_base(
            rospy.get_param("~move_base_wait_timeout", 10.0)
        )

        self.executors = {

            MoveCommandGoal.MODE_MOVE_TO_POINT:
                MoveBaseExecutor(self.context),

            MoveCommandGoal.MODE_TRACK_PERSON:
                TrackPersonExecutor(self.context),

            MoveCommandGoal.MODE_DOCKING:
                DockingExecutor(self.context, undocking=False),

            MoveCommandGoal.MODE_UNDOCKING:
                DockingExecutor(self.context, undocking=True),
        }

        # -------------------------------------------------
        # Runtime state
        # -------------------------------------------------

        self.active_mode = None
        self.active_state = MoveCommandFeedback.STATE_IDLE
        self.active_state_name = "IDLE"
        self.active_message = ""

        self.emergency_stop = False

        self.stop_requested = False

        # -------------------------------------------------
        # State publisher
        # -------------------------------------------------

        self.state_pub = rospy.Publisher(
            "/moving_server/state",
            MovingServerState,
            queue_size=1,
            latch=True
        )

        self.state_timer = rospy.Timer(
            rospy.Duration(0.5),
            self.publish_state
        )

        # -------------------------------------------------
        # Emergency / reset services
        # -------------------------------------------------

        self.cancel_srv = rospy.Service(
            "/moving_server/stop",
            Trigger,
            self.handle_stop
        )

        self.stop_srv = rospy.Service(
            "/moving_server/emergency_stop",
            Trigger,
            self.handle_emergency_stop
        )

        self.resume_srv = rospy.Service(
            "/moving_server/clear_emergency_stop",
            Trigger,
            self.handle_clear_emergency_stop
        )

        # -------------------------------------------------
        # Action server
        # -------------------------------------------------

        self.action_server = actionlib.SimpleActionServer(
            "/moving_server/move_command",
            MoveCommandAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )

        self.action_server.start()

        rospy.loginfo(
            "[moving_server] Started, action: "
            "/moving_server/move_command"
        )

    # =====================================================
    # State reporting
    # =====================================================

    def publish_state(self, event=None):

        message = MovingServerState()

        message.header.stamp = rospy.Time.now()

        message.active = self.active_mode is not None

        message.mode = (
            self.active_mode
            if self.active_mode is not None
            else 0
        )

        message.mode_name = (
            MODE_NAMES.get(self.active_mode, "UNKNOWN")
            if self.active_mode is not None
            else "IDLE"
        )

        message.state = self.active_state
        message.state_name = self.active_state_name

        message.message = (
            "EMERGENCY_STOP"
            if self.emergency_stop
            else self.active_message
        )

        self.state_pub.publish(message)

    # =====================================================
    # Emergency stop
    # =====================================================

    def handle_stop(self, request):
        """
        Stop the running goal, whatever its mode.

        For move_base goals the cancel has to reach move_base itself,
        otherwise it keeps publishing its own velocity: zeroing our
        topics would not stop the robot.
        """

        if not self.action_server.is_active():

            self.context.stop_all()

            return TriggerResponse(True, "No goal is running")

        self.stop_requested = True

        self.context.stop_all()

        self.context.move_base_client.cancel_all_goals()

        rospy.loginfo("[moving_server] Stop requested")

        return TriggerResponse(True, "Stopping the running goal")

    def handle_emergency_stop(self, request):

        self.emergency_stop = True

        self.context.stop_all()

        # Same reason as above: move_base owns its own cmd_vel stream
        self.context.move_base_client.cancel_all_goals()

        # The control loop watches this flag and aborts the running goal

        rospy.logwarn(
            "[moving_server] EMERGENCY STOP requested"
        )

        return TriggerResponse(
            True,
            "Emergency stop engaged, motion goals are rejected"
        )

    def handle_clear_emergency_stop(self, request):

        self.emergency_stop = False

        rospy.loginfo(
            "[moving_server] Emergency stop cleared"
        )

        return TriggerResponse(
            True,
            "Emergency stop cleared"
        )

    # =====================================================
    # Result helpers
    # =====================================================

    def make_result(self, success, result_code, message):

        result = MoveCommandResult()

        result.success = success
        result.result_code = result_code
        result.message = message

        robot_pose = self.context.get_robot_pose()

        if robot_pose is not None:
            result.final_pose = robot_pose

        return result

    # =====================================================
    # Goal execution
    # =====================================================

    def execute_callback(self, goal):

        mode_name = MODE_NAMES.get(goal.mode)

        # -------------------------------------------------
        # Validation
        # -------------------------------------------------

        if mode_name is None:

            self.action_server.set_aborted(
                self.make_result(
                    False,
                    MoveCommandResult.RESULT_INVALID_GOAL,
                    "Unknown mode {}".format(goal.mode)
                )
            )

            return

        if self.emergency_stop:

            self.action_server.set_aborted(
                self.make_result(
                    False,
                    MoveCommandResult.RESULT_ABORTED,
                    "Emergency stop is engaged"
                )
            )

            return

        executor = self.executors[goal.mode]

        rospy.loginfo(
            "[moving_server] New goal: %s",
            mode_name
        )

        accepted, message = executor.start(goal)

        if not accepted:

            rospy.logwarn(
                "[moving_server] Goal rejected: %s",
                message
            )

            executor.stop()

            self.action_server.set_aborted(
                self.make_result(
                    False,
                    MoveCommandResult.RESULT_INVALID_GOAL,
                    message
                )
            )

            return

        self.active_mode = goal.mode

        self.stop_requested = False

        timeout = (
            goal.timeout
            if goal.timeout > 0.0
            else self.default_timeout
        )

        self.run_control_loop(
            executor,
            timeout
        )

        executor.stop()

        self.stop_requested = False

        self.active_mode = None
        self.active_state = MoveCommandFeedback.STATE_IDLE
        self.active_state_name = "IDLE"

        self.publish_state()

    # -----------------------------------------------------
    # Control loop of one goal
    # -----------------------------------------------------

    def run_control_loop(self, executor, timeout):

        rate = rospy.Rate(self.control_rate)

        start_time = rospy.Time.now()

        last_time = start_time

        while not rospy.is_shutdown():

            # ---------------------------------------------
            # Preemption, from the action or from ~/stop
            # ---------------------------------------------

            if (
                self.action_server.is_preempt_requested()
                or
                self.stop_requested
            ):

                self.context.stop_all()

                rospy.loginfo(
                    "[moving_server] Goal preempted"
                )

                self.action_server.set_preempted(
                    self.make_result(
                        False,
                        MoveCommandResult.RESULT_PREEMPTED,
                        "Goal preempted"
                    )
                )

                return

            # ---------------------------------------------
            # Emergency stop
            # ---------------------------------------------

            if self.emergency_stop:

                self.context.stop_all()

                self.action_server.set_aborted(
                    self.make_result(
                        False,
                        MoveCommandResult.RESULT_ABORTED,
                        "Emergency stop engaged during execution"
                    )
                )

                return

            # ---------------------------------------------
            # Timeout
            # ---------------------------------------------

            now = rospy.Time.now()

            elapsed = (now - start_time).to_sec()

            if timeout > 0.0 and elapsed > timeout:

                self.context.stop_all()

                rospy.logwarn(
                    "[moving_server] Goal timed out after %.1f s",
                    elapsed
                )

                self.action_server.set_aborted(
                    self.make_result(
                        False,
                        MoveCommandResult.RESULT_TIMEOUT,
                        "Timeout after {:.1f} s".format(elapsed)
                    )
                )

                return

            # ---------------------------------------------
            # One control cycle
            # ---------------------------------------------

            dt = (now - last_time).to_sec()

            if dt <= 0.0:
                dt = 1.0 / self.control_rate

            last_time = now

            outcome = executor.update(dt)

            self.active_state = outcome.state
            self.active_state_name = outcome.state_name
            self.active_message = outcome.message

            if outcome.status == ExecutorStatus.RUNNING:

                self.publish_feedback(outcome)

                rate.sleep()

                continue

            self.context.stop_all()

            if outcome.status == ExecutorStatus.SUCCEEDED:

                rospy.loginfo(
                    "[moving_server] Goal succeeded: %s",
                    outcome.message
                )

                self.action_server.set_succeeded(
                    self.make_result(
                        True,
                        MoveCommandResult.RESULT_SUCCESS,
                        outcome.message
                    )
                )

                return

            rospy.logwarn(
                "[moving_server] Goal aborted: %s",
                outcome.message
            )

            self.action_server.set_aborted(
                self.make_result(
                    False,
                    outcome.result_code,
                    outcome.message
                )
            )

            return

        # Node shutting down
        self.context.stop_all()

        if self.action_server.is_active():

            self.action_server.set_aborted(
                self.make_result(
                    False,
                    MoveCommandResult.RESULT_ABORTED,
                    "Node shutdown"
                )
            )

    # -----------------------------------------------------

    def publish_feedback(self, outcome):

        feedback = MoveCommandFeedback()

        feedback.state = outcome.state
        feedback.state_name = outcome.state_name
        feedback.distance_remaining = outcome.distance_remaining
        feedback.heading_error = outcome.heading_error
        feedback.progress = outcome.progress
        feedback.message = outcome.message

        self.action_server.publish_feedback(feedback)

    # =====================================================
    # Shutdown
    # =====================================================

    def shutdown(self):

        rospy.loginfo(
            "[moving_server] Shutting down"
        )

        self.context.stop_all()


def main():

    rospy.init_node("moving_server")

    server = MovingServer()

    rospy.on_shutdown(server.shutdown)

    rospy.spin()


if __name__ == "__main__":
    main()
