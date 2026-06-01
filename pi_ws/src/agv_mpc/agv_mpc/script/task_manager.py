#!/usr/bin/env python3

import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped


class AGVController:

    def __init__(self):

        rospy.init_node("agv_state_machine")

        # =====================================
        # ROBOT STATE
        # =====================================
        self.rx = 0.0
        self.ry = 0.0
        self.yaw = 0.0

        self.last_yaw = 0.0
        self.current_node = None

        # =====================================
        # SENSOR STATE
        # =====================================
        self.last_sensor = None
        self.sensor_changed = False

        # =====================================
        # OBSTACLE STATE
        # =====================================
        self.obstacle_detected = False
        self.obstacle_done = False

        # waiting robot stop after obstacle path
        self.waiting_recovery = False

        self.last_move_x = 0.0
        self.last_move_y = 0.0

        self.stop_time = None

        # =====================================
        # LAST NORMAL PATH
        # =====================================
        self.last_path = None

        # =====================================
        # GRAPH
        # =====================================
        self.node_positions = {
            1: (0.0, 0.0),
            2: (0.0, 1.0),
            3: (0.0, 2.0)
            # 3: (1.0, 2.0)
        }

        self.node_threshold = 0.4
        self.odom_count = 0
        # =====================================
        # PUB / SUB
        # =====================================

        # publish avoidance path
        self.pub_path = rospy.Publisher(
            "/desired_path",
            Path,
            queue_size=1
        )
        self.odom_path_pub = rospy.Publisher(
            '/recorded_path',
            Path,
            queue_size=10
        )
        # publish normal trajectory name
        self.pub_traj = rospy.Publisher(
            "/trajectory_type",
            String,
            queue_size=1
        )

        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/sensor", Bool, self.sensor_cb)
        rospy.Subscriber("/obstacle", Bool, self.obstacle_cb)

        rospy.loginfo("AGV Controller Started")

    # =====================================
    # ODOM CALLBACK
    # =====================================
    def odom_cb(self, msg):

        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        quat = [
            q.x,
            q.y,
            q.z,
            q.w
        ]

        _, _, self.yaw = euler_from_quaternion(quat)

        self.current_node = self.get_closest_node(
            self.rx,
            self.ry
        )

        self.decision_maker()
        # self.odom_count += 1
        # if self.odom_count % 5 == 0:

        #     self.odom_path.header = msg.header
        #     self.odom_path.header.frame_id = "odom"

        #     pose = PoseStamped()
        #     pose.header = msg.header
        #     pose.pose = msg.pose.pose
        #     pose.header.frame_id = "odom"

        #     self.odom_path.poses.append(pose)

        #     self.odom_path_pub.publish(self.odom_path)

    # =====================================
    # SENSOR CALLBACK
    # =====================================
    def sensor_cb(self, msg):

        if self.last_sensor is None:
            self.last_sensor = msg.data
            return

        if msg.data != self.last_sensor:
            self.sensor_changed = True

        self.last_sensor = msg.data

    # =====================================
    # OBSTACLE CALLBACK
    # =====================================
    def obstacle_cb(self, msg):

        self.obstacle_detected = msg.data

    # =====================================
    # FIND CLOSEST NODE
    # =====================================
    def get_closest_node(self, x, y):

        min_dist = 1e9
        closest = None

        for nid, (nx, ny) in self.node_positions.items():

            d = math.sqrt(
                (x - nx) ** 2 +
                (y - ny) ** 2
            )

            if d < min_dist and d < self.node_threshold:
                min_dist = d
                closest = nid

        return closest

    # =====================================
    # MAIN STATE MACHINE
    # =====================================
    def decision_maker(self):

        # =====================================
        # OBSTACLE MODE
        # =====================================
        if self.obstacle_detected:

            # publish obstacle path once
            if not self.obstacle_done:

                rospy.loginfo(
                    "Obstacle detected -> publish avoidance arc"
                )

                arc = self.generate_obstacle_path(
                    self.rx,
                    self.ry,
                    self.yaw
                )

                self.publish_path(arc)

                self.obstacle_done = True

                # start checking robot stop
                self.waiting_recovery = True

                self.last_move_x = self.rx
                self.last_move_y = self.ry
                self.last_yaw = self.yaw
                self.stop_time = rospy.Time.now()

                return

            # =====================================
            # CHECK IF ROBOT STOPPED
            # =====================================
            if self.waiting_recovery:

                dist = math.sqrt(
                    (self.rx - self.last_move_x) ** 2 +
                    (self.ry - self.last_move_y) ** 2
                )
                yaw_diff = abs(self.yaw - self.last_yaw)
                yaw_diff = abs(yaw_diff)
                # robot still moving
                print(f"dist: {dist:.3f}, yaw_diff: {yaw_diff:.3f}")
                if dist > 0.02 or yaw_diff > 0.05:

                    self.last_move_x = self.rx
                    self.last_move_y = self.ry
                    self.last_yaw = self.yaw
                    self.stop_time = rospy.Time.now()

                # robot stopped
                else:

                    dt = (
                        rospy.Time.now() -
                        self.stop_time
                    ).to_sec()

                    # stopped enough
                    if dt > 0.6:

                        rospy.loginfo(
                            "Avoidance complete -> resume path"
                        )

                        if self.last_path is not None:

                            self.pub_traj.publish(
                                self.last_path
                            )

                            rospy.loginfo(
                                f"Republish path: {self.last_path}"
                            )

                        self.waiting_recovery = False
                        self.obstacle_done = False
                        self.obstacle_detected = False

                return

        # =====================================
        # NORMAL MODE
        # =====================================
        if self.current_node is None and self.last_path is None:
            return

        if not self.sensor_changed:
            return

        next_path = self.get_next_path(
            self.current_node
        )

        if next_path is None:
            return

        self.last_path = next_path

        self.pub_traj.publish(next_path)

        rospy.loginfo(
            f"[NODE {self.current_node}] publish: {next_path}"
        )

        self.sensor_changed = False

    # =====================================
    # GRAPH LOGIC
    # =====================================
    def get_next_path(self, node):

        graph = {
            0: "0-1",
            1: "1-2",
            2: "2-3",
            3: "3-1"
        }

        return graph.get(node, None)

    # =====================================
    # GENERATE OBSTACLE ARC
    # =====================================
    def generate_obstacle_path(self, x0, y0, yaw):

        pts = []

        R = 0.4
        n = 60

        # tâm phía trước robot (local frame)
        cx = R
        cy = 0

        # bắt đầu từ điểm (0,0) => theta = pi
        thetas = np.linspace(np.pi, 0, n)

        for th in thetas:

            # local point
            lx = cx + R * math.cos(th)
            ly = cy + R * math.sin(th)

            # transform sang global
            gx = x0 + (
                lx * math.cos(yaw) -
                ly * math.sin(yaw)
            )

            gy = y0 + (
                lx * math.sin(yaw) +
                ly * math.cos(yaw)
            )

            pts.append((gx, gy))

        return pts

    # =====================================
    # PUBLISH PATH
    # =====================================
    def publish_path(self, points):

        path = Path()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"

        for (x, y) in points:

            pose = PoseStamped()

            pose.header = path.header

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.pub_path.publish(path)

        rospy.loginfo(
            f"Published avoidance path ({len(points)} pts)"
        )


# =====================================
# MAIN
# =====================================
if __name__ == "__main__":

    AGVController()

    rospy.spin()