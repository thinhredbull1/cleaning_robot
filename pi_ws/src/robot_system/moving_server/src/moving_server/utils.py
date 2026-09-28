#!/usr/bin/env python3

"""Small math / geometry helpers shared by controllers and executors."""

import math


def clamp(value, minimum, maximum):

    return max(minimum, min(maximum, value))


def clamp_vector(
    velocity_x,
    velocity_y,
    max_x,
    max_y,
    max_norm=None
):
    """
    Clamp a mecanum translation command.

    Each axis is limited first, then the resulting vector is scaled down
    uniformly if its norm exceeds max_norm, so the *direction* of the
    motion is preserved instead of being distorted by the per axis
    saturation.
    """

    velocity_x = clamp(velocity_x, -max_x, max_x)
    velocity_y = clamp(velocity_y, -max_y, max_y)

    if max_norm is None or max_norm <= 0.0:
        return velocity_x, velocity_y

    norm = math.hypot(velocity_x, velocity_y)

    if norm > max_norm:

        scale = max_norm / norm

        velocity_x *= scale
        velocity_y *= scale

    return velocity_x, velocity_y


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def yaw_from_quaternion(quaternion):
    """Yaw [rad] of a geometry_msgs/Quaternion."""

    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        +
        quaternion.x * quaternion.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        +
        quaternion.z * quaternion.z
    )

    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """Return (x, y, z, w) of a rotation about z."""

    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0)
    )


def distance_2d(point_a, point_b):

    return math.hypot(
        point_a.x - point_b.x,
        point_a.y - point_b.y
    )


def to_robot_frame(reference, point):
    """
    Express `point` (x, y, yaw), given in a fixed frame, in the frame of
    the robot whose pose in that same frame is `reference` (x, y, yaw).
    """

    reference_x, reference_y, reference_yaw = reference
    point_x, point_y, point_yaw = point

    delta_x = point_x - reference_x
    delta_y = point_y - reference_y

    cos_yaw = math.cos(reference_yaw)
    sin_yaw = math.sin(reference_yaw)

    return (
        cos_yaw * delta_x + sin_yaw * delta_y,
        -sin_yaw * delta_x + cos_yaw * delta_y,
        normalize_angle(point_yaw - reference_yaw)
    )


def from_robot_frame(reference, point):
    """Inverse of `to_robot_frame`."""

    reference_x, reference_y, reference_yaw = reference
    point_x, point_y, point_yaw = point

    cos_yaw = math.cos(reference_yaw)
    sin_yaw = math.sin(reference_yaw)

    return (
        reference_x + cos_yaw * point_x - sin_yaw * point_y,
        reference_y + sin_yaw * point_x + cos_yaw * point_y,
        normalize_angle(reference_yaw + point_yaw)
    )


def interpolate_poses(start, end, resolution):
    """
    Straight line of (x, y, yaw) poses from `start` to `end`, sampled
    every `resolution` metres. Orientation is taken from `end`, which is
    what a straight docking approach needs: the robot holds the docked
    orientation the whole way in.
    """

    start_x, start_y, _ = start
    end_x, end_y, end_yaw = end

    length = math.hypot(end_x - start_x, end_y - start_y)

    steps = max(1, int(length / max(resolution, 1e-3)))

    path = []

    for index in range(steps + 1):

        ratio = float(index) / float(steps)

        path.append((
            start_x + ratio * (end_x - start_x),
            start_y + ratio * (end_y - start_y),
            end_yaw
        ))

    return path


def apply_slew_rate(target, current, rate_limit, dt):
    """Limit how fast a command may change, |d/dt| <= rate_limit."""

    if rate_limit <= 0.0 or dt <= 0.0:
        return target

    max_step = rate_limit * dt

    delta = clamp(
        target - current,
        -max_step,
        max_step
    )

    return current + delta
