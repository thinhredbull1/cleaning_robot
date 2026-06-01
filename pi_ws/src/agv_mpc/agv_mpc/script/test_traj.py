#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np


class PathNode:

    def __init__(self):

        rospy.init_node('path_node')
        rospy.loginfo("Trajectory path node started")

        self.frame_id = "odom"
        self.trajectory_type = "8_shape"

        self.pub = rospy.Publisher('/desired_path', Path, queue_size=10)
        rospy.Subscriber('/trajectory_type', String, self.cb)

        pts = 30

        # angles
        th_br_out = np.linspace(-np.pi/2, 0, pts)
        th_br_in  = np.linspace(0, np.pi/2, pts)

        th_tr_in  = np.linspace(-np.pi/2, 0, pts)
        th_tr_out = np.linspace(0, np.pi/2, pts)

        th_tl_out = np.linspace(np.pi/2, np.pi, pts)
        th_tl_in  = np.linspace(np.pi, 3*np.pi/2, pts)

        th_bl_in  = np.linspace(np.pi/2, np.pi, pts)
        th_bl_out = np.linspace(np.pi, 3*np.pi/2, pts)

        # =====================
        # STRAIGHT SEGMENT 0->1
        # =====================
        self.p01_x = np.linspace(-1, 1, pts*2)
        self.p01_y = np.zeros_like(self.p01_x)

        # =====================
        # 1 -> 2
        # =====================
        x1, y1 = self.arc(2, 0.5, 0.5, th_br_out)
        x2, y2 = self.arc(2, 0.5, 0.5, th_br_in)

        self.p12_x = np.concatenate([
            np.linspace(1, 2, pts),
            x1, x2,
            np.linspace(2, 1, pts)
        ])
        self.p12_y = np.concatenate([
            np.zeros(pts),
            y1, y2,
            np.ones(pts)
        ])

        # =====================
        # 2 -> 3
        # =====================
        x1, y1 = self.arc(0, 1.5, 0.5, th_tl_out)
        x2, y2 = self.arc(0, 1.5, 0.5, th_tl_in)

        self.p23_x = np.concatenate([
            np.linspace(1, 0, pts),
            x1, x2,
            np.linspace(0, 1, pts)
        ])
        self.p23_y = np.concatenate([
            2*np.ones(pts),
            y1, y2,
            np.ones(pts)
        ])
        # print("p12_x:", p12_x)
        # print("p12_y:", p12_y)
        self.p23_x = self.p23_x[::-1]
        self.p23_y = self.p23_y[::-1]

        # =====================
        # 3 -> 2
        # =====================
        x1, y1 = self.arc(2, 1.5, 0.5, th_tr_in)
        x2, y2 = self.arc(2, 1.5, 0.5, th_tr_out)

        self.p32_x = np.concatenate([
            np.linspace(1, 2, pts),
            x1, x2,
            np.linspace(2, 1, pts)
        ])
        self.p32_y = np.concatenate([
            np.ones(pts),
            y1, y2,
            2*np.ones(pts)
        ])
        self.p32_x = self.p32_x[::-1]  
        self.p32_y = self.p32_y[::-1]

        # =====================
        # 2 -> 1
        # =====================
        x1, y1 = self.arc(0, 0.5, 0.5, th_bl_in)
        x2, y2 = self.arc(0, 0.5, 0.5, th_bl_out)

        self.p21_x = np.concatenate([
            np.linspace(1, 0, pts),
            x1, x2,
            np.linspace(0, 1, pts)
        ])
        self.p21_y = np.concatenate([
            np.ones(pts),
            y1, y2,
            np.zeros(pts)
        ])

    # =========================
    # ARC FUNCTION
    # =========================
    def arc(self, cx, cy, r, th):
        x = cx + r * np.cos(th)
        y = cy + r * np.sin(th)
        return x, y

    # =========================
    # BUILD FULL PATH
    # =========================
    def generate_and_publish(self):




        if self.trajectory_type == "8_shape":
            # x = np.concatenate([p12_x, p23_x,p32_x])
            # y = np.concatenate([p12_y, p23_y, p32_y])
            x = np.concatenate([self.p12_x, self.p23_x, self.p32_x, self.p21_x])
            y = np.concatenate([self.p12_y, self.p23_y, self.p32_y, self.p21_y])
        elif self.trajectory_type == "0-1":
            x = self.p01_x
            y = self.p01_y
        elif self.trajectory_type == "1-2":
            x = self.p12_x
            y = self.p12_y
        elif self.trajectory_type == "2-3":
            x = self.p23_x
            y = self.p23_y
        elif self.trajectory_type == "3-2":
            x = self.p32_x
            y = self.p32_y
        elif self.trajectory_type == "2-1":
            x = self.p21_x
            y = self.p21_y
        elif self.trajectory_type == "3-1":
            x = np.concatenate([self.p32_x, self.p21_x])
            y = np.concatenate([self.p32_y, self.p21_y])
        else:
            rospy.logwarn("Unknown trajectory type: %s. Defaulting to 8_shape.", self.trajectory_type)
            x = np.concatenate([self.p12_x, self.p23_x, self.p32_x, self.p21_x])
            y = np.concatenate([self.p12_y, self.p23_y, self.p32_y, self.p21_y])
        # =====================
        # BUILD ROS PATH
        # =====================
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.frame_id

        for i in range(len(x)):

            pose = PoseStamped()
            pose.header = path_msg.header

            pose.pose.position.x = x[i]
            pose.pose.position.y = y[i]
            pose.pose.position.z = 0.0

            if i < len(x) - 1:
                dx = x[i+1] - x[i]
                dy = y[i+1] - y[i]
                yaw = np.arctan2(dy, dx)
            else:
                yaw = 0.0

            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path_msg.poses.append(pose)

        self.pub.publish(path_msg)
        rospy.loginfo("Published desired_path with %d points", len(path_msg.poses))

    # =========================
    # CALLBACK
    # =========================
    def cb(self, msg):
        self.trajectory_type = msg.data
        self.generate_and_publish()


# =========================
if __name__ == '__main__':
    node = PathNode()
    rospy.spin()