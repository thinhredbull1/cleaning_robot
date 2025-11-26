#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf.transformations as tf_trans
import math
import time

class OdomCmdImuMonitor:
    def __init__(self):
        rospy.init_node("odom_cmd_imu_monitor", anonymous=True)

        # CMD_VEL data
        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0

        # ODOM data
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_linear_vel = 0.0
        self.odom_angular_vel = 0.0

        # IMU data
        self.imu_yaw = 0.0
        self.imu_gyro_x = 0.0
        self.imu_gyro_y = 0.0
        self.imu_gyro_z = 0.0

        # Subscribers
        rospy.Subscriber("/cmd_vel",        Twist, self.cmd_vel_callback)
        rospy.Subscriber("/odom",           Odometry, self.odom_callback)
        rospy.Subscriber("/imu",            Imu, self.imu_callback)

        self.last_print = time.time()

        self.run()

    # CMD_VEL
    def cmd_vel_callback(self, msg):
        self.cmd_vel_linear = msg.linear.x
        self.cmd_vel_angular = msg.angular.z

    # ODOM
    def odom_callback(self, msg):
        # Position
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        # Quaternion → yaw
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = tf_trans.euler_from_quaternion(quat)
        self.odom_yaw = yaw

        # Velocities
        self.odom_linear_vel = msg.twist.twist.linear.x
        self.odom_angular_vel = msg.twist.twist.angular.z

    # IMU
    def imu_callback(self, msg):
        # Orientation (yaw)
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = tf_trans.euler_from_quaternion(quat)
        self.imu_yaw = yaw

        # Angular velocity (gyroscope)
        self.imu_gyro_x = msg.angular_velocity.x
        self.imu_gyro_y = msg.angular_velocity.y
        self.imu_gyro_z = msg.angular_velocity.z

    def run(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            if time.time() - self.last_print >= 0.5:
                self.last_print = time.time()

                print("======== ROBOT STATUS ========")
                print(f"CMD_VEL: linear.x = {self.cmd_vel_linear:.3f}, angular.z = {self.cmd_vel_angular:.3f}")
                print(f"ODOM VEL: linear.x = {self.odom_linear_vel:.3f}, angular.z = {self.odom_angular_vel:.3f}")
                print(f"ODOM POSE: x = {self.odom_x:.3f}, y = {self.odom_y:.3f}, yaw = {math.degrees(self.odom_yaw):.2f} deg")
                print(f"IMU YAW : {math.degrees(self.imu_yaw):.2f} deg")
                print(f"IMU GYRO: x = {self.imu_gyro_x:.3f}, y = {self.imu_gyro_y:.3f}, z = {self.imu_gyro_z:.3f}")
                print("================================\n")

            rate.sleep()


if __name__ == "__main__":
    try:
        OdomCmdImuMonitor()
    except rospy.ROSInterruptException:
        pass
