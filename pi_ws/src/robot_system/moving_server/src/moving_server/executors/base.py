#!/usr/bin/env python3

"""
Executor interface.

Every mode of the action is implemented as a non blocking state machine:
the node calls `start()` once, then `update()` at the control rate until
something else than RUNNING comes back, and finally `stop()`.

Executors never touch the action server: they only report status, the
node maps that onto feedback / result.
"""

from moving_server_msgs.msg import MoveCommandFeedback, MoveCommandResult


class ExecutorStatus:

    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


class ExecutorOutcome:
    """Return value of `update()`."""

    def __init__(
        self,
        status,
        state=MoveCommandFeedback.STATE_IDLE,
        state_name="IDLE",
        message="",
        result_code=MoveCommandResult.RESULT_SUCCESS,
        distance_remaining=0.0,
        heading_error=0.0,
        progress=0.0
    ):

        self.status = status
        self.state = state
        self.state_name = state_name
        self.message = message
        self.result_code = result_code
        self.distance_remaining = distance_remaining
        self.heading_error = heading_error
        self.progress = progress

    @property
    def is_running(self):

        return self.status == ExecutorStatus.RUNNING


class MotionExecutor:

    #: Human readable mode name, overridden by subclasses
    name = "BASE"

    def __init__(self, context):

        self.context = context

        self.params = context.params

        self.goal = None

    # -----------------------------------------------------
    # To be implemented
    # -----------------------------------------------------

    def start(self, goal):
        """
        Validate and start the goal.

        Returns (accepted, message).
        """

        raise NotImplementedError

    def update(self, dt):
        """Advance one control cycle, returns an ExecutorOutcome."""

        raise NotImplementedError

    def stop(self):
        """Release every resource, always called once."""

        raise NotImplementedError

    # -----------------------------------------------------
    # Shortcuts
    # -----------------------------------------------------

    def running(self, **kwargs):

        return ExecutorOutcome(
            ExecutorStatus.RUNNING,
            **kwargs
        )

    def succeeded(self, message="", **kwargs):

        return ExecutorOutcome(
            ExecutorStatus.SUCCEEDED,
            message=message,
            result_code=MoveCommandResult.RESULT_SUCCESS,
            **kwargs
        )

    def failed(self, result_code, message, **kwargs):

        return ExecutorOutcome(
            ExecutorStatus.FAILED,
            message=message,
            result_code=result_code,
            **kwargs
        )
