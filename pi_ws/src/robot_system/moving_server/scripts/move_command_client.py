#!/usr/bin/env python3

"""
Command line client for /moving_server/move_command.

    rosrun moving_server move_command_client.py point 1.0 2.0 0.0
    rosrun moving_server move_command_client.py track --distance 1.2
    rosrun moving_server move_command_client.py dock dock_01
    rosrun moving_server move_command_client.py undock
"""

import sys
import argparse

import rospy
import actionlib

from moving_server_msgs.msg import MoveCommandAction, MoveCommandGoal

from moving_server.utils import quaternion_from_yaw


def build_goal(arguments):

    goal = MoveCommandGoal()

    goal.timeout = arguments.timeout

    if arguments.command == "point":

        goal.mode = MoveCommandGoal.MODE_MOVE_TO_POINT

        goal.target_pose.header.frame_id = arguments.frame

        goal.target_pose.pose.position.x = arguments.x
        goal.target_pose.pose.position.y = arguments.y

        quaternion = quaternion_from_yaw(arguments.yaw)

        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

    elif arguments.command == "track":

        goal.mode = MoveCommandGoal.MODE_TRACK_PERSON

        goal.target_topic = arguments.topic
        goal.follow_distance = arguments.distance
        goal.hold_time = arguments.hold_time

    elif arguments.command == "dock":

        goal.mode = MoveCommandGoal.MODE_DOCKING

        goal.dock_id = arguments.dock_id

    else:

        goal.mode = MoveCommandGoal.MODE_UNDOCKING

    return goal


def feedback_callback(feedback):

    rospy.loginfo(
        "[client] %s | remaining %.2f m | heading %.2f rad | "
        "%.0f%% | %s",
        feedback.state_name,
        feedback.distance_remaining,
        feedback.heading_error,
        100.0 * feedback.progress,
        feedback.message
    )


def parse_arguments():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="0 -> use the default of the server"
    )

    subparsers = parser.add_subparsers(dest="command")
    subparsers.required = True

    point_parser = subparsers.add_parser("point")
    point_parser.add_argument("x", type=float)
    point_parser.add_argument("y", type=float)
    point_parser.add_argument(
        "yaw",
        type=float,
        nargs="?",
        default=0.0
    )
    point_parser.add_argument("--frame", default="map")

    track_parser = subparsers.add_parser("track")
    track_parser.add_argument("--topic", default="")
    track_parser.add_argument("--distance", type=float, default=0.0)
    track_parser.add_argument("--hold-time", type=float, default=0.0)

    dock_parser = subparsers.add_parser("dock")
    dock_parser.add_argument("dock_id", nargs="?", default="")

    subparsers.add_parser("undock")

    return parser.parse_args(
        rospy.myargv(argv=sys.argv)[1:]
    )


def main():

    arguments = parse_arguments()

    rospy.init_node("move_command_client", anonymous=True)

    client = actionlib.SimpleActionClient(
        "/moving_server/move_command",
        MoveCommandAction
    )

    rospy.loginfo("[client] Waiting for moving_server...")

    client.wait_for_server()

    client.send_goal(
        build_goal(arguments),
        feedback_cb=feedback_callback
    )

    rospy.on_shutdown(client.cancel_goal)

    client.wait_for_result()

    result = client.get_result()

    if result is None:

        rospy.logwarn("[client] No result received")

        return

    rospy.loginfo(
        "[client] success=%s code=%d message=%s",
        result.success,
        result.result_code,
        result.message
    )


if __name__ == "__main__":
    main()
