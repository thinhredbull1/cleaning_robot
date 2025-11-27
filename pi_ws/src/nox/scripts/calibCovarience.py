#!/usr/bin/env python3
import board
import busio
import adafruit_bno055
import time
import math
import numpy as np

# === 1. Khởi tạo BNO055 IMU ===
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
time.sleep(0.1)
bno.mode = adafruit_bno055.IMUPLUS_MODE  # gyro + accel, no mag
time.sleep(0.1)

# === 2. Thông số đo ===
num_samples = 500           # số mẫu đo
delay_s = 0.01              # 10ms → ~100Hz
print("Bắt đầu đo IMU, giữ IMU cố định...")

# Lưu dữ liệu
accel_data = []
gyro_data = []
orientation_data = []

for _ in range(num_samples):
    # --- Accel (m/s^2) ---
    accel = bno.acceleration
    if accel is not None:
        accel_data.append(accel)
    
    # --- Gyro (deg/s -> rad/s) ---
    gyro = bno.gyro
    if gyro is not None:
        gx, gy, gz = [math.radians(g) for g in gyro]
        gyro_data.append([gx, gy, gz])
    
    # --- Orientation quaternion (w, x, y, z) -> Euler roll/pitch/yaw ---
    quat = bno.quaternion
    if quat is not None:
        w, x, y, z = quat
        # convert to roll, pitch, yaw
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        orientation_data.append([roll, pitch, yaw])
    
    time.sleep(delay_s)

# === 3. Chuyển sang numpy để tính variance ===
accel_data = np.array(accel_data)        # x, y, z
gyro_data = np.array(gyro_data)          # x, y, z
orientation_data = np.array(orientation_data)  # roll, pitch, yaw

# Tính variance
accel_var = np.var(accel_data, axis=0)
gyro_var = np.var(gyro_data, axis=0)
orient_var = np.var(orientation_data, axis=0)

# === 4. Tạo ma trận covariance 3x3 (row-major) ===
def var_to_cov_matrix(var):
    return [var[0],0,0, 0,var[1],0, 0,0,var[2]]

orientation_cov = var_to_cov_matrix(orient_var)          # roll, pitch, yaw
angular_velocity_cov = var_to_cov_matrix(gyro_var)       # x, y, z
linear_acceleration_cov = var_to_cov_matrix(accel_var)   # x, y, z

# === 5. In ra kết quả để paste vào ROS EKF ===
print("\n# Orientation covariance (roll, pitch, yaw)")
print(orientation_cov)

print("\n# Angular velocity covariance (x, y, z)")
print(angular_velocity_cov)

print("\n# Linear acceleration covariance (x, y, z)")
print(linear_acceleration_cov)
