#!/usr/bin/env python3
import board
import busio
import adafruit_bno055
import time
import math
import numpy as np

# === 1. Khởi tạo BNO055 IMU ===
print("Initializing I2C connection...")
i2c = busio.I2C(board.SCL, board.SDA)
print("Creating BNO055 instance...")
bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
time.sleep(0.5)

# Check sensor connection
# print(f"BNO055 System Status: {bno.system_status()}")
# print(f"BNO055 System Error: {bno.system_error()}")

# Set to NDOF mode (9-DOF, all sensors enabled) for better quaternion
bno.mode = adafruit_bno055.NDOF_MODE  # Use NDOF instead of IMUPLUS for quaternion
time.sleep(0.5)
print("BNO055 initialized in NDOF mode")
while True:
    sys, gyro, accel, mag = bno.calibration_status
    print(f"Calib - Sys:{sys} Gyro:{gyro} Accel:{accel} Mag:{mag}")
    if sys > 0 and gyro == 3 and accel >=0 and mag >= 0:
        print("Sensor fully calibrated!")
        break
    time.sleep(1)
# === 2. Thông số đo ===
num_samples = 500           # số mẫu đo
delay_s = 0.05              # 10ms → ~100Hz
print("Bắt đầu đo IMU, giữ IMU cố định...")

# Lưu dữ liệu
accel_data = []
gyro_data = []
orientation_data = []

for i in range(num_samples):
    # --- Accel (m/s^2) ---
    accel = bno.acceleration
    if accel and all(v is not None for v in accel):
        accel_data.append(accel)
    else:
        print(f"Skip accel {i}: {accel}")
    
    # --- Gyro (deg/s -> rad/s) ---
    gyro = bno.gyro
    if gyro and all(v is not None for v in gyro):
        gyro_data.append([math.radians(v) for v in gyro])
    else:
        print(f"Skip gyro {i}: {gyro}")
    
    # --- Orientation quaternion (w, x, y, z) -> Euler roll/pitch/yaw ---
    quat = bno.quaternion
    if quat and all(v is not None for v in quat):
        w, x, y, z = quat
        # Validate quaternion values
        if w is not None and x is not None and y is not None and z is not None:
            # convert to roll, pitch, yaw
            t0 = 2.0 * (w * x + y * z)
            t1 = 1.0 - 2.0 * (x * x + y * y)
            roll = math.atan2(t0, t1)

            t2 = 2.0 * (w * y - z * x)
            t2 = 1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch = math.asin(t2)

            t3 = 2.0 * (w * z + x * y)
            t4 = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(t3, t4)

            orientation_data.append([roll, pitch, yaw])
        else:
            print(f"Warning: Sample {i} - Quaternion component is None")
    else:
        print(f"Skip quat {i}: {quat}")
    
    time.sleep(delay_s)

# === 3. Chuyển sang numpy để tính variance ===
# Filter out empty arrays
if len(accel_data) == 0:
    print("ERROR: No acceleration data collected!")
    exit(1)
if len(gyro_data) == 0:
    print("ERROR: No gyro data collected!")
    exit(1)
if len(orientation_data) == 0:  
    print("ERROR: No orientation data collected!")
    exit(1)

print(f"Collected {len(accel_data)} accel samples, {len(gyro_data)} gyro samples, {len(orientation_data)} orientation samples")

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