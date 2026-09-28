#!/usr/bin/env python3

"""
Trajectory tracking algorithm (docking approach).

Unlike the person follower, this controller is given a *path* and has to
stay on it, finishing on an exact pose.

The base is a mecanum drive, which changes the problem completely
compared to a differential robot: position and orientation are
decoupled. The controller therefore

  * drives the translation straight towards a look-ahead point on the
    path, correcting the cross track error on the y axis,
  * regulates the heading independently on the path orientation,

so the robot can stay aligned with the dock during the whole approach
instead of steering into it. Docking is still performed backwards
(`reverse`), which for a mecanum base only means the reference
orientation is mirrored: the motion itself is a plain translation.

Set `holonomic = False` for the differential fallback (pure pursuit).

No ROS dependency: poses are plain (x, y, yaw) tuples in the robot frame.
"""

import math

from moving_server.utils import (
    clamp,
    clamp_vector,
    normalize_angle,
    apply_slew_rate,
)


class TrajectoryTrackingConfig:

    def __init__(self, **kwargs):

        # --- Pure pursuit / look-ahead ---
        self.lookahead_distance = 0.35      # [m]
        self.min_lookahead_distance = 0.15  # [m]
        # Lookahead grows with speed: L = L0 + k * v
        self.lookahead_gain = 0.4

        # --- Gains ---
        self.kp_linear = 1.0
        self.kp_lateral = 1.2
        self.kp_angular = 1.6
        self.kp_cross_track = 1.0
        self.kp_final_yaw = 1.2

        # --- Limits ---
        self.max_linear_velocity = 0.15
        self.max_lateral_velocity = 0.10
        self.max_translation_speed = 0.18
        self.min_linear_velocity = 0.03
        self.max_angular_velocity = 0.6
        self.linear_acceleration = 0.3
        self.lateral_acceleration = 0.3
        self.angular_acceleration = 1.5

        # --- Tolerances ---
        self.goal_tolerance = 0.03          # [m]
        self.yaw_tolerance = 0.05           # [rad]

        # Abort if the robot drifts further than this from the path
        self.max_cross_track_error = 0.30   # [m]

        # --- Behaviour ---
        self.holonomic = True
        self.reverse = True
        # Distance from the goal where the robot starts slowing down
        self.slowdown_distance = 0.30       # [m]

        for key, value in kwargs.items():

            if hasattr(self, key):
                setattr(self, key, value)


class TrajectoryTrackingResult:

    RUNNING = "RUNNING"
    REACHED = "REACHED"
    OFF_TRACK = "OFF_TRACK"
    INVALID = "INVALID"


class TrajectoryTrackingController:
    """Look-ahead tracker with decoupled heading control."""

    def __init__(self, config=None):

        self.config = config or TrajectoryTrackingConfig()

        self.reset()

    # -----------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------

    def reset(self):

        self.last_linear = 0.0
        self.last_lateral = 0.0
        self.last_angular = 0.0

        # Measured velocity, refreshed by compute()
        self.current_velocity = None

        # Index of the closest path point, never goes backwards
        self.nearest_index = 0

        self.distance_remaining = 0.0
        self.heading_error = 0.0
        self.cross_track_error = 0.0

    # -----------------------------------------------------
    # Helpers
    # -----------------------------------------------------

    def _update_nearest_index(self, path):
        """Closest path point, searching forward only."""

        best_index = self.nearest_index
        best_distance = float("inf")

        for index in range(self.nearest_index, len(path)):

            x, y, _ = path[index]

            distance = math.hypot(x, y)

            if distance < best_distance:

                best_distance = distance
                best_index = index

        self.nearest_index = best_index

        return best_index, best_distance

    def _find_lookahead_point(self, path, start_index, lookahead):
        """First point at least `lookahead` away, else the last one."""

        for index in range(start_index, len(path)):

            x, y, _ = path[index]

            if math.hypot(x, y) >= lookahead:
                return path[index]

        return path[-1]

    def _signed_cross_track_error(self, path, index):
        """Lateral offset of the robot w.r.t. the path segment [m]."""

        if index + 1 >= len(path):
            index = max(0, len(path) - 2)

        if index + 1 >= len(path):
            return 0.0

        x0, y0, _ = path[index]
        x1, y1, _ = path[index + 1]

        segment_x = x1 - x0
        segment_y = y1 - y0

        segment_length = math.hypot(segment_x, segment_y)

        if segment_length < 1e-6:
            return 0.0

        # Robot is at the origin of its own frame
        return (
            segment_x * (0.0 - y0)
            -
            segment_y * (0.0 - x0)
        ) / segment_length

    def _reference_yaw(self, path, index):
        """
        Orientation the robot should hold, in its own frame.

        The path poses already carry the docking orientation, so the
        error is simply the yaw of the reference pose seen from the
        robot. On a mecanum base this holds for both directions of
        travel: `reverse` never mirrors the heading, only the motion.
        """

        _, _, path_yaw = path[index]

        return normalize_angle(path_yaw)

    # -----------------------------------------------------
    # Control
    # -----------------------------------------------------

    def compute(self, path, dt, current_velocity=None):
        """
        path: list of (x, y, yaw) poses in the robot frame, ordered from
              the entry point to the final docked pose.

        current_velocity: measured (vx, vy, w) from the odometry. When
        given, the slew rate limiter starts from what the base really
        does instead of from the last command, so a saturated or
        slipping base cannot make the commands run away.

        Returns (linear_x, linear_y, angular, status).
        """

        config = self.config

        self.current_velocity = current_velocity

        if not path:

            return 0.0, 0.0, 0.0, TrajectoryTrackingResult.INVALID

        goal_x, goal_y, goal_yaw = path[-1]

        distance_to_goal = math.hypot(goal_x, goal_y)

        self.distance_remaining = distance_to_goal

        nearest_index, _ = self._update_nearest_index(path)

        cross_track_error = self._signed_cross_track_error(
            path,
            nearest_index
        )

        self.cross_track_error = cross_track_error

        if abs(cross_track_error) > config.max_cross_track_error:

            return 0.0, 0.0, 0.0, TrajectoryTrackingResult.OFF_TRACK

        if config.holonomic:

            return self._compute_holonomic(
                path,
                nearest_index,
                cross_track_error,
                goal_x,
                goal_y,
                goal_yaw,
                distance_to_goal,
                dt
            )

        return self._compute_differential(
            path,
            nearest_index,
            cross_track_error,
            goal_yaw,
            distance_to_goal,
            dt
        )

    # -----------------------------------------------------
    # Mecanum
    # -----------------------------------------------------

    def _compute_holonomic(
        self,
        path,
        nearest_index,
        cross_track_error,
        goal_x,
        goal_y,
        goal_yaw,
        distance_to_goal,
        dt
    ):

        config = self.config

        # -------------------------------------------------
        # Heading: hold the dock orientation all the way,
        # converge on the final one near the goal
        # -------------------------------------------------

        if distance_to_goal <= config.slowdown_distance:

            yaw_error = normalize_angle(goal_yaw)
            yaw_gain = config.kp_final_yaw

        else:

            yaw_error = self._reference_yaw(path, nearest_index)
            yaw_gain = config.kp_angular

        self.heading_error = yaw_error

        # -------------------------------------------------
        # Terminal condition: position *and* orientation
        # -------------------------------------------------

        if (
            distance_to_goal <= config.goal_tolerance
            and
            abs(yaw_error) <= config.yaw_tolerance
        ):

            self.last_linear = 0.0
            self.last_lateral = 0.0
            self.last_angular = 0.0

            return 0.0, 0.0, 0.0, TrajectoryTrackingResult.REACHED

        if abs(yaw_error) <= config.yaw_tolerance:

            angular = 0.0

        else:

            angular = clamp(
                yaw_gain * yaw_error,
                -config.max_angular_velocity,
                config.max_angular_velocity
            )

        # -------------------------------------------------
        # Translation
        # -------------------------------------------------

        if distance_to_goal <= config.goal_tolerance:

            # On the goal, only the yaw is left
            velocity_x = 0.0
            velocity_y = 0.0

        elif distance_to_goal <= config.slowdown_distance:

            # Final approach: proportional control on the goal itself
            velocity_x = config.kp_linear * goal_x
            velocity_y = config.kp_lateral * goal_y

        else:

            lookahead = max(
                config.min_lookahead_distance,
                config.lookahead_distance
                + config.lookahead_gain
                * math.hypot(self.last_linear, self.last_lateral)
            )

            lookahead = min(lookahead, distance_to_goal)

            target_x, target_y, _ = self._find_lookahead_point(
                path,
                nearest_index,
                lookahead
            )

            norm = math.hypot(target_x, target_y)

            if norm < 1e-6:

                velocity_x = 0.0
                velocity_y = 0.0

            else:

                speed = config.max_translation_speed

                velocity_x = speed * target_x / norm
                velocity_y = speed * target_y / norm

                # Cross track correction, purely lateral on a mecanum
                velocity_y += config.kp_cross_track * cross_track_error

        velocity_x, velocity_y = clamp_vector(
            velocity_x,
            velocity_y,
            config.max_linear_velocity,
            config.max_lateral_velocity,
            config.max_translation_speed
        )

        # Keep the command above the dead zone of the drive
        speed = math.hypot(velocity_x, velocity_y)

        if 0.0 < speed < config.min_linear_velocity:

            boost = config.min_linear_velocity / speed

            velocity_x *= boost
            velocity_y *= boost

        return self._smooth(velocity_x, velocity_y, angular, dt)

    # -----------------------------------------------------
    # Differential fallback (pure pursuit)
    # -----------------------------------------------------

    def _compute_differential(
        self,
        path,
        nearest_index,
        cross_track_error,
        goal_yaw,
        distance_to_goal,
        dt
    ):

        config = self.config

        # -------------------------------------------------
        # Terminal phase: align in place on the docked yaw
        # -------------------------------------------------

        if distance_to_goal <= config.goal_tolerance:

            yaw_error = normalize_angle(goal_yaw)

            self.heading_error = yaw_error

            if abs(yaw_error) <= config.yaw_tolerance:

                self.last_linear = 0.0
                self.last_lateral = 0.0
                self.last_angular = 0.0

                return 0.0, 0.0, 0.0, TrajectoryTrackingResult.REACHED

            angular = clamp(
                config.kp_final_yaw * yaw_error,
                -config.max_angular_velocity,
                config.max_angular_velocity
            )

            return self._smooth(0.0, 0.0, angular, dt)

        # -------------------------------------------------
        # Pure pursuit
        # -------------------------------------------------

        lookahead = max(
            config.min_lookahead_distance,
            config.lookahead_distance
            + config.lookahead_gain * abs(self.last_linear)
        )

        lookahead = min(lookahead, max(distance_to_goal, 1e-3))

        target_x, target_y, _ = self._find_lookahead_point(
            path,
            nearest_index,
            lookahead
        )

        # In reverse the robot "looks" along its -x axis
        if config.reverse:

            heading_error = normalize_angle(
                math.atan2(-target_y, -target_x)
            )

        else:

            heading_error = normalize_angle(
                math.atan2(target_y, target_x)
            )

        self.heading_error = heading_error

        angular = (
            config.kp_angular * heading_error
            +
            config.kp_cross_track * cross_track_error
        )

        # Steering is mirrored when driving backwards
        if config.reverse:
            angular = -angular

        angular = clamp(
            angular,
            -config.max_angular_velocity,
            config.max_angular_velocity
        )

        speed = config.max_linear_velocity

        if distance_to_goal < config.slowdown_distance:

            speed *= max(
                0.0,
                distance_to_goal / config.slowdown_distance
            )

        speed *= max(0.2, math.cos(heading_error))

        speed = max(speed, config.min_linear_velocity)

        linear = -speed if config.reverse else speed

        return self._smooth(linear, 0.0, angular, dt)

    # -----------------------------------------------------

    def _smooth(self, velocity_x, velocity_y, angular, dt):

        config = self.config

        if self.current_velocity is None:

            previous = (
                self.last_linear,
                self.last_lateral,
                self.last_angular
            )

        else:

            previous = self.current_velocity

        velocity_x = apply_slew_rate(
            velocity_x,
            previous[0],
            config.linear_acceleration,
            dt
        )

        velocity_y = apply_slew_rate(
            velocity_y,
            previous[1],
            config.lateral_acceleration,
            dt
        )

        angular = apply_slew_rate(
            angular,
            previous[2],
            config.angular_acceleration,
            dt
        )

        self.last_linear = velocity_x
        self.last_lateral = velocity_y
        self.last_angular = angular

        return (
            velocity_x,
            velocity_y,
            angular,
            TrajectoryTrackingResult.RUNNING
        )
