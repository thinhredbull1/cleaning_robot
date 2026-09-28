#!/usr/bin/env python3

"""
Mock implementation of the dock detection service.

Lets the docking sequence be exercised without the real charger
detector: the dock pose comes from the `docks` parameter and is returned
in its configured frame, exactly as a real detector would return it in
its sensor frame.

The approach path is left empty on purpose: moving_server rebuilds both
legs itself (alignment point, then the straight run in) from the dock
pose, in the odom frame, after the odometry reset.

For bring-up only, this node can also serve a fake odometry reset
service (`provide_mock_reset_odom`), so the whole sequence can be run
without the base driver.

Replace it with the real detector; only the DetectDock interface matters
to moving_server.
"""

import rospy

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse

from moving_server_msgs.srv import DetectDock, DetectDockResponse

from moving_server.utils import quaternion_from_yaw


class MockDockDetector:

    def __init__(self):

        self.docks = rospy.get_param("~docks", {})

        self.default_dock_id = rospy.get_param(
            "~default_dock_id",
            ""
        )

        service_name = rospy.get_param(
            "~detect_dock_service",
            "/dock_detector/detect_dock"
        )

        self.service = rospy.Service(
            service_name,
            DetectDock,
            self.handle_detect_dock
        )

        # -------------------------------------------------
        # Optional fake odometry reset
        # -------------------------------------------------

        self.reset_service = None

        if rospy.get_param("~provide_mock_reset_odom", False):

            reset_name = rospy.get_param(
                "~reset_odom_service",
                "/reset_odom"
            )

            self.reset_service = rospy.Service(
                reset_name,
                Trigger,
                self.handle_reset_odom
            )

            rospy.logwarn(
                "[mock_dock_detector] Serving a FAKE odometry reset "
                "on %s, bring-up only",
                reset_name
            )

        rospy.loginfo(
            "[mock_dock_detector] Ready on %s, known docks: %s",
            service_name,
            list(self.docks.keys())
        )

    # -----------------------------------------------------

    def handle_detect_dock(self, request):

        response = DetectDockResponse()

        dock_id = request.dock_id.strip() or self.default_dock_id

        dock_config = self.docks.get(dock_id)

        if not dock_config or "dock_pose" not in dock_config:

            response.success = False
            response.message = "Unknown dock_id: {}".format(dock_id)

            return response

        dock_pose = dock_config["dock_pose"]

        response.dock_pose = self.make_pose(
            dock_config.get("frame_id", "map"),
            float(dock_pose[0]),
            float(dock_pose[1]),
            float(dock_pose[2])
        )

        # moving_server generates the approach itself
        response.approach_path.header.frame_id = (
            dock_config.get("frame_id", "map")
        )

        response.success = True
        response.confidence = 1.0
        response.message = "Mock detection of {}".format(dock_id)

        return response

    # -----------------------------------------------------

    def handle_reset_odom(self, request):

        rospy.loginfo(
            "[mock_dock_detector] Fake odometry reset called"
        )

        return TriggerResponse(True, "Mock odometry reset")

    # -----------------------------------------------------

    @staticmethod
    def make_pose(frame_id, x, y, yaw):

        pose = PoseStamped()

        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = x
        pose.pose.position.y = y

        quaternion = quaternion_from_yaw(yaw)

        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose


def main():

    rospy.init_node("mock_dock_detector")

    MockDockDetector()

    rospy.spin()


if __name__ == "__main__":
    main()
