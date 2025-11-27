#!/usr/bin/env python3
import rospy
import board
import busio
import adafruit_bno055
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time

class BNO055Node:
    def __init__(self):
        # --- ROS Node ---
        rospy.init_node("bno055_imu_node", anonymous=True)
        self.pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
        self.rate = rospy.Rate(100)  # Hz, đọc nhanh nhất
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # --- I2C + BNO055 ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
        time.sleep(0.1)
        # self.bno.mode = adafruit_bno055.NDOF_MODE  # gyro + accel, no mag
        self.last_imu_msg=Imu()
        # --- Covariance matrices ---
        self.orientation_cov = [0.19320297061877081, 0, 0, 0, 0.01985739557701275, 0, 0, 0, 6.70298336970668e-06]
        self.angular_velocity_cov = [1.4056574841526518e-03, 0, 0, 0, 2.504884899521743e-03, 0, 0, 0, 2.407325369826665e-07]
        self.linear_acceleration_cov = [0.05417740681601826, 0, 0, 0, 0.000143956145868615, 0, 0, 0, 215.17343152207158]
       
        if not self.wait_for_calibration(timeout=10.0):
            rospy.logerr("BNO055 calibration failed or timed out after 10s. Shutting down node.")
            rospy.signal_shutdown("BNO055 not calibrated")
            return
           
        rospy.loginfo("BNO055 IMU node started, publishing to /imu/data")
    def normalize_quat(self, q):
        w, x, y, z = q
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm < 1e-6:
            return [1.0, 0.0, 0.0, 0.0]
        inv = 1.0 / norm
        return [w*inv, x*inv, y*inv, z*inv]
    def wait_for_calibration(self, timeout=10.0):
   
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                sys, gyro, accel, mag = self.bno.calibration_status
                rospy.loginfo(f"Calibration → System:{sys}  Gyro:{gyro}  Accel:{accel}  Mag:{mag}")

               
                if sys > 0 and gyro == 3 and accel >= 0 and mag >= 0:
                    rospy.loginfo("BNO055 calibration complete!")
                    return True
            except Exception as e:
                rospy.logwarn(f"Error reading calibration status: {e}")

            time.sleep(0.5)

        return False
    def read_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        orient_var=self.orientation_cov
        accel_var=self.linear_acceleration_cov
        gyro_var=self.angular_velocity_cov
        valid_data = True
        # --- Linear Acceleration ---
        accel = self.bno.acceleration
        if accel and all(v is not None for v in accel):
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]
        else:
            accel_var=[1e6,0,0, 0,1e6,0, 0,0,1e6]
            # rospy.logwarn("BNO055 acceleration data is None")
            # imu_msg.linear_acceleration.x = 0.0
            # imu_msg.linear_acceleration.y = 0.0
            # imu_msg.linear_acceleration.z = 0.0

        # --- Angular Velocity (deg/s -> rad/s) ---
        gyro = self.bno.gyro
        if gyro is not None:
            imu_msg.angular_velocity.x = math.radians(gyro[0])
            imu_msg.angular_velocity.y = math.radians(gyro[1])
            imu_msg.angular_velocity.z = math.radians(gyro[2])
        else:
            
            # rospy.logwarn("Gyro data is None, setting angular velocity to zero")
            # return None
            # imu_msg.angular_velocity.x = 0.0
            # imu_msg.angular_velocity.y = 0.0
            # imu_msg.angular_velocity.z = 0.0
            imu_msg.angular_velocity=self.last_imu_msg.angular_velocity
            # gyro_var=[1e6,0,0, 0,1e6,0, 0,0,1e6]
            valid_data = False
        

        # --- Orientation quaternion ---
        quat = self.bno.quaternion  # [w, x, y, z]
        if quat and all(v is not None for v in quat):
            w, x, y, z = quat
            norm = math.sqrt(w*w + x*x + y*y + z*z)
            if 0.9 < norm < 1.1:  # cho phép sai số nhỏ
                
                inv_norm = 1.0 / norm
                normalized = [w*inv_norm, x*inv_norm, y*inv_norm, z*inv_norm]
                w, x, y, z = normalized
                # quat_ok = True
                q = Quaternion()
                q.w = w
                q.x = x
                q.y = y
                q.z = z
                imu_msg.orientation = q
            else:
                # rospy.logwarn("Quaternion data is not good")
                # print(norm_after)
                imu_msg.orientation=self.last_imu_msg.orientation
        else:
            # rospy.logwarn("Quaternion data is None, setting orientation to zero")
            # return None
            imu_msg.orientation=self.last_imu_msg.orientation
            # imu_msg.orientation.w = 1.0
            # imu_msg.orientation.x = 0.0
            # imu_msg.orientation.y = 0.0
            # imu_msg.orientation.z = 0.0
            # orient_var=[1e6,0,0,0,1e6,0,0,0,1e6]  # High uncertainty
            valid_data = False
       
        # --- Covariances ---
        imu_msg.orientation_covariance = orient_var
        imu_msg.angular_velocity_covariance = gyro_var
        imu_msg.linear_acceleration_covariance = accel_var
        self.last_imu_msg=imu_msg
 
        return self.last_imu_msg

    def run(self):
        while not rospy.is_shutdown():
            imu_msg = self.read_imu()
            if imu_msg is not None:

                t = TransformStamped()
                t.header.stamp = imu_msg.header.stamp
                t.header.frame_id = "base_link"      # parent
                t.child_frame_id  = "imu_link"       # child
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0

                    # Dữ liệu tốt → publish TF thật
                t.transform.rotation = imu_msg.orientation
                self.tf_broadcaster.sendTransform(t)
                self.pub.publish(imu_msg)
               

        # Dữ liệu lỗi → vẫn publish TF identity (không làm EKF bị drift)
      

               
            self.rate.sleep()

# === Main ===
if __name__ == "__main__":
    try:
        node = BNO055Node()
        node.run()
    except rospy.ROSInterruptException:
        pass