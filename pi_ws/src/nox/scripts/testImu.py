#!/usr/bin/env python3
import rospy
import board
import busio
import adafruit_bno055
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import time

class BNO055Node:
    def __init__(self):
        # --- ROS Node ---
        rospy.init_node("bno055_imu_node", anonymous=True)
        self.pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
        self.rate = rospy.Rate(100)  # Hz, đọc nhanh nhất

        # --- I2C + BNO055 ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
        time.sleep(0.1)
        self.bno.mode = adafruit_bno055.IMUPLUS_MODE  # gyro + accel, no mag

        # --- Covariance matrices ---
        self.orientation_cov = [0.002]*9
        self.angular_velocity_cov = [0.0002]*9
        self.linear_acceleration_cov = [0.0005]*9

        rospy.loginfo("BNO055 IMU node started, publishing to /imu/data")

    def read_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # --- Linear Acceleration ---
        accel = self.bno.acceleration
        if accel is not None:
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]
        else:
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0

        # --- Angular Velocity (deg/s -> rad/s) ---
        gyro = self.bno.gyro
        if gyro is not None:
            imu_msg.angular_velocity.x = math.radians(gyro[0])
            imu_msg.angular_velocity.y = math.radians(gyro[1])
            imu_msg.angular_velocity.z = math.radians(gyro[2])
        else:
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0

        # --- Orientation quaternion ---
        quat = self.bno.quaternion  # [w, x, y, z]
        if quat is not None:
            q = Quaternion()
            q.w = quat[0]
            q.x = quat[1]
            q.y = quat[2]
            q.z = quat[3]
            imu_msg.orientation = q
        else:
            imu_msg.orientation.w = 1.0
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0

        # --- Covariances ---
        imu_msg.orientation_covariance = self.orientation_cov
        imu_msg.angular_velocity_covariance = self.angular_velocity_cov
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_cov

        return imu_msg

    def run(self):
        while not rospy.is_shutdown():
            imu_msg = self.read_imu()
            self.pub.publish(imu_msg)
            self.rate.sleep()

# === Main ===
if __name__ == "__main__":
    try:
        node = BNO055Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
