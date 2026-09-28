#!/usr/bin/env python3

"""
MODE_TRACK_PERSON.

Sequence:
  1. subscribe to the person pose topic
  2. transform the pose into the base frame
  3. run the point-to-point tracking algorithm
  4. publish on the tracking cmd_vel topic of the mux

The algorithm itself lives in
controllers/person_follow_controller.py and knows nothing about ROS.
"""

import threading

import rospy

from geometry_msgs.msg import PoseStamped

from moving_server_msgs.msg import MoveCommandFeedback, MoveCommandResult

from moving_server.executors.base import MotionExecutor
from moving_server.controllers.person_follow_controller import (
    PersonFollowConfig,
    PersonFollowController,
)


class TrackPersonExecutor(MotionExecutor):

    name = "TRACK_PERSON"

    def __init__(self, context):

        super(TrackPersonExecutor, self).__init__(context)

        config_params = dict(
            self.params.get("person_tracking", {})
        )

        self.config = PersonFollowConfig(**config_params)

        self.controller = PersonFollowController(self.config)

        self.target_timeout = float(
            self.params
            .get("person_tracking", {})
            .get("target_timeout", 1.5)
        )

        self.subscriber = None

        self.lock = threading.Lock()

        self.last_target = None
        self.last_target_stamp = rospy.Time(0)

        self.hold_time = 0.0
        self.in_goal_since = None

    # =====================================================
    # Lifecycle
    # =====================================================

    def start(self, goal):

        self.goal = goal

        topic = goal.target_topic.strip()

        if not topic:

            topic = self.params["person_pose_topic"]

        # Per goal override of the desired distance
        if goal.follow_distance > 0.0:

            self.config.follow_distance = goal.follow_distance

        else:

            self.config.follow_distance = float(
                self.params
                .get("person_tracking", {})
                .get("follow_distance", 1.0)
            )

        if self.config.follow_distance < self.config.min_safe_distance:

            return False, (
                "follow_distance ({:.2f}) is below min_safe_distance "
                "({:.2f})".format(
                    self.config.follow_distance,
                    self.config.min_safe_distance
                )
            )

        self.hold_time = max(0.0, goal.hold_time)

        self.in_goal_since = None

        with self.lock:

            self.last_target = None
            self.last_target_stamp = rospy.Time(0)

        self.controller.reset()

        self.subscriber = rospy.Subscriber(
            topic,
            PoseStamped,
            self.target_callback,
            queue_size=1
        )

        rospy.loginfo(
            "[moving_server] Tracking person on %s, "
            "follow_distance=%.2f m",
            topic,
            self.config.follow_distance
        )

        return True, "Tracking started on {}".format(topic)

    def stop(self):

        if self.subscriber is not None:

            self.subscriber.unregister()

            self.subscriber = None

        self.context.stop_tracking()

    # =====================================================
    # Target input
    # =====================================================

    def target_callback(self, message):

        with self.lock:

            self.last_target = message

            self.last_target_stamp = rospy.Time.now()

    def get_target_in_base_frame(self):
        """Returns (x, y) in the base frame, None if unusable."""

        with self.lock:

            target = self.last_target

        if target is None:
            return None

        pose = PoseStamped()

        pose.header.frame_id = target.header.frame_id
        # Time(0) -> latest available transform, tolerant to detector lag
        pose.header.stamp = rospy.Time(0)
        pose.pose = target.pose

        if not pose.header.frame_id:

            # Detector already publishes in the robot frame
            return (
                target.pose.position.x,
                target.pose.position.y
            )

        if pose.header.frame_id == self.context.base_frame:

            return (
                target.pose.position.x,
                target.pose.position.y
            )

        transformed = self.context.transform_pose(
            pose,
            self.context.base_frame
        )

        if transformed is None:
            return None

        return (
            transformed.pose.position.x,
            transformed.pose.position.y
        )

    # =====================================================
    # Control cycle
    # =====================================================

    def update(self, dt):

        now = rospy.Time.now()

        with self.lock:

            last_stamp = self.last_target_stamp

        # -------------------------------------------------
        # Target lost
        # -------------------------------------------------

        if last_stamp == rospy.Time(0):

            age = self.target_timeout + 1.0

        else:

            age = (now - last_stamp).to_sec()

        if age > self.target_timeout:

            self.context.stop_tracking()

            self.controller.reset()

            return self.failed(
                MoveCommandResult.RESULT_TARGET_LOST,
                "Person lost, no pose for {:.2f} s".format(age),
                state=MoveCommandFeedback.STATE_TRACKING_PERSON,
                state_name="TRACKING_PERSON"
            )

        target = self.get_target_in_base_frame()

        if target is None:

            # Transform not ready yet, hold still for this cycle
            self.context.stop_tracking()

            return self.running(
                state=MoveCommandFeedback.STATE_TRACKING_PERSON,
                state_name="TRACKING_PERSON",
                message="Waiting for a usable target pose"
            )

        # -------------------------------------------------
        # Tracking algorithm
        # -------------------------------------------------

        linear_x, linear_y, angular, at_goal = self.controller.compute(
            target[0],
            target[1],
            dt
        )

        self.context.publish_tracking_command(
            linear_x,
            linear_y,
            angular
        )

        distance_remaining = abs(self.controller.distance_error)

        # -------------------------------------------------
        # Termination
        # -------------------------------------------------

        if self.hold_time > 0.0:

            if at_goal:

                if self.in_goal_since is None:
                    self.in_goal_since = now

                held = (now - self.in_goal_since).to_sec()

                if held >= self.hold_time:

                    self.context.stop_tracking()

                    return self.succeeded(
                        message="Person held at {:.2f} m for {:.1f} s"
                        .format(
                            self.config.follow_distance,
                            held
                        ),
                        state=MoveCommandFeedback.STATE_FINISHING,
                        state_name="FINISHING",
                        progress=1.0
                    )

            else:

                self.in_goal_since = None

        # hold_time == 0 -> track forever, until preempt or timeout
        return self.running(
            state=MoveCommandFeedback.STATE_TRACKING_PERSON,
            state_name="TRACKING_PERSON",
            message="Following person",
            distance_remaining=distance_remaining,
            heading_error=self.controller.heading_error,
            progress=0.0
        )
