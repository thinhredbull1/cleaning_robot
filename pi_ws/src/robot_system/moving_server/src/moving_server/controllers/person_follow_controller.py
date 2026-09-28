#!/usr/bin/env python3

"""
Point-to-point tracking algorithm (person following).

The controller only knows about the *current* target position expressed
in the robot frame. There is no path, no history: the robot regulates its
distance to `follow_distance` while keeping the target in front of it.

The base is a mecanum drive, so the command is holonomic: the robot
translates directly towards the standoff point (vx, vy) and rotates to
face the target at the same time. Set `holonomic = False` to fall back to
the differential behaviour (turn first, then drive).

Deliberately free of any ROS dependency so it can be unit tested and
reused. The executor is responsible for tf, timeouts and publishing.
"""

import math

from moving_server.utils import (
    clamp,
    clamp_vector,
    normalize_angle,
    apply_slew_rate,
)


class PersonFollowConfig:

    def __init__(self, **kwargs):

        # --- Objective ---
        self.follow_distance = 1.0      # [m] distance to keep
        self.distance_tolerance = 0.15  # [m] dead band on distance
        self.angle_tolerance = 0.10     # [rad] dead band on bearing

        # --- Gains ---
        self.kp_linear = 0.8            # forward / radial
        self.kp_lateral = 0.9           # sideways, mecanum only
        self.kp_angular = 1.5

        # --- Limits ---
        self.max_linear_velocity = 0.45     # [m/s] on x
        self.max_lateral_velocity = 0.35    # [m/s] on y
        self.max_translation_speed = 0.5    # [m/s] on |(vx, vy)|
        self.min_linear_velocity = 0.05
        self.max_angular_velocity = 1.2
        self.linear_acceleration = 0.6
        self.lateral_acceleration = 0.6
        self.angular_acceleration = 2.0

        # --- Behaviour ---
        self.holonomic = True
        # Differential mode only: above this bearing the robot only rotates
        self.turn_in_place_angle = 0.7   # [rad]
        # Never drive backwards to keep the distance
        self.allow_backward = True
        # Below this distance the robot always backs off / stops
        self.min_safe_distance = 0.35    # [m]

        for key, value in kwargs.items():

            if hasattr(self, key):
                setattr(self, key, value)


class PersonFollowController:
    """Regulates (distance, bearing) of a moving point target."""

    def __init__(self, config=None):

        self.config = config or PersonFollowConfig()

        self.reset()

    # -----------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------

    def reset(self):

        self.last_linear = 0.0
        self.last_lateral = 0.0
        self.last_angular = 0.0

        # Last computed errors, used by the executor for feedback
        self.distance_error = 0.0
        self.heading_error = 0.0

    # -----------------------------------------------------
    # Control
    # -----------------------------------------------------

    def compute(self, target_x, target_y, dt):
        """
        target_x / target_y: target position in the robot frame [m].

        Returns (linear_x, linear_y, angular, at_goal).
        """

        config = self.config

        distance = math.hypot(target_x, target_y)

        bearing = normalize_angle(
            math.atan2(target_y, target_x)
        )

        # Positive -> too far, negative -> too close
        distance_error = distance - config.follow_distance

        self.distance_error = distance_error
        self.heading_error = bearing

        at_goal = (
            abs(distance_error) <= config.distance_tolerance
            and
            abs(bearing) <= config.angle_tolerance
        )

        # -------------------------------------------------
        # Angular: always face the target
        # -------------------------------------------------

        if abs(bearing) <= config.angle_tolerance:

            angular = 0.0

        else:

            angular = clamp(
                config.kp_angular * bearing,
                -config.max_angular_velocity,
                config.max_angular_velocity
            )

        # -------------------------------------------------
        # Translation
        # -------------------------------------------------

        if config.holonomic:

            linear, lateral = self._compute_holonomic(
                target_x,
                target_y,
                distance,
                distance_error
            )

        else:

            linear = self._compute_differential(
                distance,
                distance_error,
                bearing
            )

            lateral = 0.0

        # -------------------------------------------------
        # Smoothing
        # -------------------------------------------------

        linear = apply_slew_rate(
            linear,
            self.last_linear,
            config.linear_acceleration,
            dt
        )

        lateral = apply_slew_rate(
            lateral,
            self.last_lateral,
            config.lateral_acceleration,
            dt
        )

        angular = apply_slew_rate(
            angular,
            self.last_angular,
            config.angular_acceleration,
            dt
        )

        self.last_linear = linear
        self.last_lateral = lateral
        self.last_angular = angular

        return linear, lateral, angular, at_goal

    # -----------------------------------------------------
    # Mecanum: drive straight to the standoff point
    # -----------------------------------------------------

    def _compute_holonomic(
        self,
        target_x,
        target_y,
        distance,
        distance_error
    ):

        config = self.config

        if distance < 1e-3:

            # Degenerate: target on top of the robot, back off on x
            return -config.min_linear_velocity, 0.0

        # -------------------------------------------------
        # Safety bubble: push away radially
        # -------------------------------------------------

        if distance < config.min_safe_distance:

            if not config.allow_backward:
                return 0.0, 0.0

            intrusion = config.min_safe_distance - distance

            scale = -(
                config.kp_linear * intrusion
                + config.min_linear_velocity
            ) / distance

            return clamp_vector(
                scale * target_x,
                scale * target_y,
                config.max_linear_velocity,
                config.max_lateral_velocity,
                config.max_translation_speed
            )

        # -------------------------------------------------
        # Dead band
        # -------------------------------------------------

        if abs(distance_error) <= config.distance_tolerance:
            return 0.0, 0.0

        # -------------------------------------------------
        # Standoff point: on the target line, follow_distance away
        # -------------------------------------------------

        ratio = distance_error / distance

        error_x = target_x * ratio
        error_y = target_y * ratio

        velocity_x = config.kp_linear * error_x
        velocity_y = config.kp_lateral * error_y

        if not config.allow_backward:
            velocity_x = max(0.0, velocity_x)

        velocity_x, velocity_y = clamp_vector(
            velocity_x,
            velocity_y,
            config.max_linear_velocity,
            config.max_lateral_velocity,
            config.max_translation_speed
        )

        # Avoid commands too small to move the robot
        speed = math.hypot(velocity_x, velocity_y)

        if 0.0 < speed < config.min_linear_velocity:

            boost = config.min_linear_velocity / speed

            velocity_x *= boost
            velocity_y *= boost

        return velocity_x, velocity_y

    # -----------------------------------------------------
    # Differential fallback
    # -----------------------------------------------------

    def _compute_differential(self, distance, distance_error, bearing):

        config = self.config

        if distance < config.min_safe_distance:

            if not config.allow_backward:
                return 0.0

            return -min(
                config.max_linear_velocity,
                config.kp_linear
                * (config.min_safe_distance - distance)
                + config.min_linear_velocity
            )

        if abs(bearing) > config.turn_in_place_angle:

            # Badly misaligned: rotate first
            return 0.0

        if abs(distance_error) <= config.distance_tolerance:
            return 0.0

        linear = config.kp_linear * distance_error

        # Scale down while turning
        linear *= max(0.0, math.cos(bearing))

        if not config.allow_backward:
            linear = max(0.0, linear)

        linear = clamp(
            linear,
            -config.max_linear_velocity,
            config.max_linear_velocity
        )

        if 0.0 < abs(linear) < config.min_linear_velocity:

            linear = math.copysign(
                config.min_linear_velocity,
                linear
            )

        return linear
