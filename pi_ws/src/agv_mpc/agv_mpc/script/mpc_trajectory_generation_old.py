#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, pi, sin,sqrt
from tf.transformations import quaternion_from_euler
import numpy as np
class PathNode:

    def __init__(self):

        rospy.init_node('path_node')
        rospy.loginfo("path_node is started!!")

        # =========================
        # Variables
        # =========================
        self.odom_path = Path()
        self.error_path = Path()
        self.desired_path = Path()

        self.robot_odom = Odometry()

        self.start_x = 0.0
        self.start_y = 0.0

        self.odom_count = 0

        self.sum_error = 0.0
        self.error_x = 0.0

        self.get_new_path = False

        self.frame_id = "odom"

        self.trajectory_type = rospy.get_param(
            '~trajectory_type',
            'circle'
        )

        self.current_path_x = 0.0
        self.current_path_y = 0.0
        self.current_path_theta = 0.0

        # =========================
        # Publishers
        # =========================
        self.desired_path_pub = rospy.Publisher(
            '/desired_path',
            Path,
            queue_size=500
        )

        self.odom_path_pub = rospy.Publisher(
            '/recorded_path',
            Path,
            queue_size=10
        )

        self.error_path_pub = rospy.Publisher(
            '/error_path',
            Path,
            queue_size=10
        )

        # =========================
        # Subscribers
        # =========================
        rospy.Subscriber(
            '/odom',
            Odometry,
            self.odom_cb
        )

        rospy.Subscriber(
            '/trajectory_type',
            String,
            self.trajectory_cb
        )

    # =========================================================
    # ODOM CALLBACK
    # =========================================================
    def odom_cb(self, data):

        self.robot_odom = data
        self.odom_count += 1

        # ===== ADD THIS: update start pose =====
        pos = data.pose.pose.position
        ori = data.pose.pose.orientation

        import tf.transformations as tf

        (_, _, yaw) = tf.euler_from_quaternion([
            ori.x,
            ori.y,
            ori.z,
            ori.w
        ])

        self.start_x = pos.x
        self.start_y = pos.y
        self.start_yaw = yaw
        # =======================================

        if self.odom_count % 5 == 0:

            self.odom_path.header = data.header
            self.odom_path.header.frame_id = self.frame_id

            pose = PoseStamped()
            pose.header = data.header
            pose.pose = data.pose.pose
            pose.header.frame_id = self.frame_id

            self.odom_path.poses.append(pose)

            self.odom_path_pub.publish(self.odom_path)

    # =========================================================
    # TRAJECTORY CALLBACK
    # =========================================================
    def trajectory_cb(self, msg):

        self.trajectory_type = msg.data.strip()

        rospy.loginfo(
            f"Received trajectory type: {self.trajectory_type}"
        )

        self.start_x = self.robot_odom.pose.pose.position.x
        self.start_y = self.robot_odom.pose.pose.position.y

        self.generate_desired_path()

    # =========================================================
    # ERROR PATH
    # =========================================================
    def generate_error_path(self):

        self.error_path.header.frame_id = self.frame_id
        self.error_path.header.seq = 1

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.get_rostime()

        pose.pose.position.x = self.current_path_x
        pose.pose.position.y = self.current_path_y
        pose.pose.orientation.w = 1.0

        self.error_path.poses.append(pose)

        pose = PoseStamped()

        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.get_rostime()

        pose.pose.position.x = (
            self.robot_odom.pose.pose.position.x
        )

        pose.pose.position.y = self.current_path_y
        pose.pose.orientation.w = 1.0

        self.error_path.poses.append(pose)

        self.error_path_pub.publish(self.error_path)

    # =========================================================
    # GENERATE DESIRED PATH
    # =========================================================
    def append_trajectory(self, traj_x, traj_y):
    

        for t in range(len(traj_x)):

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.seq = t
            pose.header.stamp = rospy.get_rostime()

            # =========================
     
            x = traj_x[t]
            y = traj_y[t]

            pose.pose.position.x = x
            pose.pose.position.y = y

            # heading
            if t < len(traj_x) - 1:
                dx = traj_x[t + 1] - traj_x[t]
                dy = traj_y[t + 1] - traj_y[t]
            else:
                dx = traj_x[t] - traj_x[t - 1]
                dy = traj_y[t] - traj_y[t - 1]

            yaw = atan2(dy, dx)

            q = quaternion_from_euler(0, 0, yaw)

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            self.desired_path.poses.append(pose)

    def tf(self, x, y):
        x0 = self.start_x
        y0 = self.start_y
        yaw = self.start_yaw

        x_w = x0 + cos(yaw)*x - sin(yaw)*y
        y_w = y0 + sin(yaw)*x + cos(yaw)*y
        return x_w, y_w
    def generate_desired_path(self):

        rospy.loginfo("generate_desired_path")

        self.desired_path.poses = []

        iterations = 200

        # =====================================================
        # CIRCLE
        # =====================================================
        if self.trajectory_type == "circle":

            radius = 0.5
            period = iterations

            for t in range(iterations):

                pose = PoseStamped()

                pose.header.seq = t
                pose.header.frame_id = self.frame_id
                pose.header.stamp = rospy.get_rostime()

                theta = 2 * pi * t / period

                x_local = radius * sin(theta)
                y_local = -radius * cos(theta) + radius

                x, y = self.tf(x_local, y_local)

                pose.pose.position.x = x
                pose.pose.position.y = y

                grad = theta + self.start_yaw
                q = quaternion_from_euler(0, 0, grad)

                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.desired_path.poses.append(pose)
        elif self.trajectory_type == "0_1":

            pts = 100

            # straight line
            traj_x = np.linspace(0, 2, pts)
            traj_y = np.zeros(pts)

            self.append_trajectory(traj_x, traj_y)


        # =========================================================


        elif self.trajectory_type == "1_2":

            pts = 100

            # line: (0,0) -> (1,0)
            line1_x = np.linspace(0, 1, pts, endpoint=False)
            line1_y = np.zeros(pts)

            # right U-turn arc
            theta = np.linspace(-np.pi/2, np.pi/2, pts)

            arc_x = 1 + 0.5 * np.cos(theta)
            arc_y = 0.5 + 0.5 * np.sin(theta)

            # line: (1,1) -> (0,1)
            line2_x = np.linspace(1, 0, pts)
            line2_y = np.ones(pts)

            traj_x = np.concatenate([
                line1_x,
                arc_x,
                line2_x
            ])

            traj_y = np.concatenate([
                line1_y,
                arc_y,
                line2_y
            ])

            self.append_trajectory(traj_x, traj_y)


        # =========================================================


        elif self.trajectory_type == "2_3":

            pts = 100

            # line: (0,0) -> (1,0)
            line1_x = np.linspace(0, 1, pts, endpoint=False)
            line1_y = np.zeros(pts)

            # right U-turn arc
            theta = np.linspace(-np.pi/2, np.pi/2, pts)

            arc_x = 1 + 0.5 * np.cos(theta)
            arc_y = 0.5 + 0.5 * np.sin(theta)

            # line: (1,1) -> (0,1)
            line2_x = np.linspace(1, 0, pts)
            line2_y = np.ones(pts)

            traj_x = np.concatenate([
                line1_x,
                arc_x,
                line2_x
            ])

            traj_y = np.concatenate([
                line1_y,
                arc_y,
                line2_y
            ])

            self.append_trajectory(traj_x, traj_y)


        # =========================================================


        elif self.trajectory_type == "3_2":

            pts = 100

            # line: (0,0) -> (-1,0)
            line1_x = np.linspace(0, -1, pts, endpoint=False)
            line1_y = np.zeros(pts)

            # left U-turn arc
            theta = np.linspace(np.pi/2, 3*np.pi/2, pts)

            arc_x = -1 + 0.5 * np.cos(theta)
            arc_y = -0.5 + 0.5 * np.sin(theta)

            # line: (-1,-1) -> (0,-1)
            line2_x = np.linspace(-1, 0, pts)
            line2_y = -1 * np.ones(pts)

            traj_x = np.concatenate([
                line1_x,
                arc_x,
                line2_x
            ])

            traj_y = np.concatenate([
                line1_y,
                arc_y,
                line2_y
            ])

            self.append_trajectory(traj_x, traj_y)


        # =========================================================


        elif self.trajectory_type == "2_1":

            pts = 100

            # line: (0,0) -> (-1,0)
            line1_x = np.linspace(0, -1, pts, endpoint=False)
            line1_y = np.zeros(pts)

            # left U-turn arc
            theta = np.linspace(np.pi/2, 3*np.pi/2, pts)

            arc_x = -1 + 0.5 * np.cos(theta)
            arc_y = -0.5 + 0.5 * np.sin(theta)

            # line: (-1,-1) -> (0,-1)
            line2_x = np.linspace(-1, 0, pts)
            line2_y = -1 * np.ones(pts)

            traj_x = np.concatenate([
                line1_x,
                arc_x,
                line2_x
            ])

            traj_y = np.concatenate([
                line1_y,
                arc_y,
                line2_y - 1.0
            ])

            self.append_trajectory(traj_x, traj_y)
        # =====================================================
        # SINE
        # =====================================================
        elif self.trajectory_type == "sine":

            amplitude = 5.0
            wavelength = 20.0
            total_length = 20.0

            for t in range(iterations):

                pose = PoseStamped()

                pose.header.seq = t
                pose.header.frame_id = self.frame_id
                pose.header.stamp = rospy.get_rostime()

                u = t / float(iterations - 1)

                x_local = u * total_length
                y_local = amplitude * sin(2 * pi * x_local / wavelength)

                x, y = self.tf(x_local, y_local)

                pose.pose.position.x = x
                pose.pose.position.y = y

                eps = 1e-3
                u2 = min(u + eps, 1.0)

                x2_local = u2 * total_length
                y2_local = amplitude * sin(2 * pi * x2_local / wavelength)

                grad_local = atan2(y2_local - y_local, x2_local - x_local)
                grad = grad_local + self.start_yaw

                q = quaternion_from_euler(0, 0, grad)

                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.desired_path.poses.append(pose)

        # =====================================================
        # L_ARC
        # =====================================================
        elif self.trajectory_type == "L_arc":

            r = 1.0

            straight_len = 1.0
            step = 0.05

            N1 = int(straight_len / step)
            N2 = iterations - N1

            # ---------------------
            # 1. Straight +X
            # ---------------------
            for i in range(N1):

                pose = PoseStamped()
                pose.header.seq = i
                pose.header.frame_id = self.frame_id
                pose.header.stamp = rospy.get_rostime()

                x_local = i * step
                y_local = 0

                x, y = self.tf(x_local, y_local)

                pose.pose.position.x = x
                pose.pose.position.y = y

                yaw = self.start_yaw

                q = quaternion_from_euler(0, 0, yaw)

                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.desired_path.poses.append(pose)

            # endpoint straight
            end_x = (N1 - 1) * step
            end_y = 0

            # ---------------------
            # 2. Quarter circle (TURN UP)
            # ---------------------
            center_x = end_x
            center_y = end_y - r   # đổi tâm xuống để quay phải đẹp hơn

            for i in range(N2):

                pose = PoseStamped()
                pose.header.seq = N1 + i
                pose.header.frame_id = self.frame_id
                pose.header.stamp = rospy.get_rostime()

                u = i / float(N2 - 1)

                # -90 degree turn
                theta = pi/2 - u * (pi/2)

                x_local = center_x + r * cos(theta)
                y_local = center_y + r * sin(theta)

                x, y = self.tf(x_local, y_local)

                pose.pose.position.x = x
                pose.pose.position.y = y

                grad = theta + self.start_yaw
                q = quaternion_from_euler(0, 0, grad)

                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.desired_path.poses.append(pose)

        
        elif self.trajectory_type == "L_back":

            r = 1.0

            straight_len = 5.0
            step = 0.05

            N1 = int(straight_len / step)
            N2 = iterations - N1

            # =========================
            # 1. Straight BACKWARD (-X local)
            # =========================
            for i in range(N1):

                pose = PoseStamped()
                pose.header.seq = i
                pose.header.frame_id = self.frame_id
                pose.header.stamp = rospy.get_rostime()

                x_local = -i * step
                y_local = 0

                x, y = self.tf(x_local, y_local)

                pose.pose.position.x = x
                pose.pose.position.y = y

                yaw = self.start_yaw + pi   # quay mặt về sau

                q = quaternion_from_euler(0, 0, yaw)

                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.desired_path.poses.append(pose)

            # endpoint
            end_x = -(N1 - 1) * step
            end_y = 0

            # =========================
            # 2. Quarter circle (-90° turn, continue backward-side turn)
            # =========================
            center_x = end_x - r
            center_y = end_y

            for i in range(N2):

                pose = PoseStamped()
                pose.header.seq = N1 + i
                pose.header.frame_id = self.frame_id
                pose.header.stamp = rospy.get_rostime()

                u = i / float(N2 - 1)

                # clockwise turn
                theta = pi/2 - u * (pi/2)

                x_local = center_x + r * cos(theta)
                y_local = center_y + r * sin(theta)

                x, y = self.tf(x_local, y_local)

                pose.pose.position.x = x
                pose.pose.position.y = y

                grad = theta + self.start_yaw + pi

                q = quaternion_from_euler(0, 0, grad)

                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.desired_path.poses.append(pose)
        # =====================================================
        # SPLINE
        # =====================================================
        elif self.trajectory_type == "spline":

            self.x_move = 10.0
            self.y_move = 10.0

            P0 = (0, 0)
            P3 = (self.x_move, self.y_move)
            P1 = (0, self.y_move)
            P2 = (self.x_move, 0)

            def bezier(t, P0, P1, P2, P3):
                x = (1-t)**3 * P0[0] + 3*(1-t)**2*t*P1[0] + 3*(1-t)*t**2*P2[0] + t**3*P3[0]
                y = (1-t)**3 * P0[1] + 3*(1-t)**2*t*P1[1] + 3*(1-t)*t**2*P2[1] + t**3*P3[1]
                return x, y

            for t in range(iterations):

                pose = PoseStamped()

                pose.header.seq = t
                pose.header.frame_id = self.frame_id
                pose.header.stamp = rospy.get_rostime()

                u = t / float(iterations - 1)

                x_local, y_local = bezier(u, P0, P1, P2, P3)

                x, y = self.tf(x_local, y_local)

                pose.pose.position.x = x
                pose.pose.position.y = y

                eps = 1e-3
                u2 = min(u + eps, 1.0)

                x2_local, y2_local = bezier(u2, P0, P1, P2, P3)

                grad_local = atan2(y2_local - y_local, x2_local - x_local)
                grad = grad_local + self.start_yaw

                q = quaternion_from_euler(0, 0, grad)

                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.desired_path.poses.append(pose)

        # =====================================================
        # SQUARE (FIX luôn theo local frame)
        # =====================================================
        elif self.trajectory_type == "square":

            length = 10
            seg = iterations // 4

            x = 0
            y = 0

            for t in range(iterations):

                pose = PoseStamped()

                pose.header.seq = t
                pose.header.frame_id = self.frame_id
                pose.header.stamp = rospy.get_rostime()

                if t < seg:
                    y += length / seg
                    yaw = pi/2

                elif t < 2*seg:
                    x -= length / seg
                    yaw = pi

                elif t < 3*seg:
                    y -= length / seg
                    yaw = -pi/2

                else:
                    x += length / seg
                    yaw = 0

                x_w, y_w = self.tf(x, y)

                pose.pose.position.x = x_w
                pose.pose.position.y = y_w

                q = quaternion_from_euler(0, 0, yaw + self.start_yaw)

                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.desired_path.poses.append(pose)

        # publish
        self.desired_path.header.frame_id = self.frame_id
        self.desired_path.header.stamp = rospy.get_rostime()

        self.desired_path_pub.publish(self.desired_path)

        self.get_new_path = True

    # =========================================================
    # CALCULATE ERROR
    # =========================================================
    def calculate_error(
        self,
        path_x,
        path_y,
        path_theta,
        robot_x,
        robot_y,
        robot_theta
    ):

        self.error_x = abs(path_x - robot_x)

        self.sum_error += self.error_x

        print("path_x:", path_x)
        print("path_y:", path_y)
        print("path_theta:", path_theta)

        print("robot_x:", robot_x)
        print("robot_y:", robot_y)
        print("robot_theta:", robot_theta)




if __name__ == '__main__':

    node = PathNode()

    rospy.spin()