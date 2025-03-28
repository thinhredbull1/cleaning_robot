#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import smbus  # import SMBus module of I2C
from math import atan2, asin, sqrt

class MPU6050:
	def __init__(self, device_address=0x68):
		self.bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
		self.Device_Address = 0x68  # MPU6050 device address
		rospy.init_node('mpu_publish', anonymous=True)
		# MPU6050 Registers and their Address
		self.PWR_MGMT_1 = 0x6B
		self.SMPLRT_DIV = 0x19
		self.CONFIG = 0x1A
		self.GYRO_CONFIG = 0x1B
		self.INT_ENABLE = 0x38
		self.ACCEL_XOUT_H = 0x3B
		self.ACCEL_YOUT_H = 0x3D
		self.ACCEL_ZOUT_H = 0x3F
		self.GYRO_XOUT_H = 0x43
		self.GYRO_YOUT_H = 0x45
		self.GYRO_ZOUT_H = 0x47
		self.MPU6050_RA_CONFIG = 0x1A
		self.MPU6050_RA_ACCEL_CONFIG = 0x1C
		self.MPU6050_AXOFFSET = 443.957
		self.MPU6050_AYOFFSET = -4194.2655
		self.MPU6050_AZOFFSET = 32.2855
		self.MPU6050_GXOFFSET = -50.051
		self.MPU6050_GYOFFSET = -23.1275
		self.MPU6050_GZOFFSET = -19.732
		self.twoKpDef = 2.0 * 0.5
		self.twoKiDef = 2.0 * 0.0
		self.MPU6050_AXGAIN = 4096.0
		self.MPU6050_AYGAIN = 4096.0
		self.MPU6050_AZGAIN = 4096.0
		self.MPU6050_GXGAIN = 16.384
		self.MPU6050_GYGAIN = 16.384
		self.MPU6050_GZGAIN = 16.384
		self.twoKp = self.twoKpDef
		self.twoKi = self.twoKiDef
		self.q0 = 1.0
		self.q1 = 0.0
		self.q2 = 0.0
		self.q3 = 0.0
		self.integralFBx = 0.0
		self.integralFBy = 0.0
		self.integralFBz = 0.0
		self.acc_x = 0.0
		self.acc_y = 0.0
		self.acc_z = 0.0
		self.gyro_x = 0.0
		self.gyro_y = 0.0
		self.gyro_z = 0.0
		self.mpu_pub = rospy.Publisher("/mpu",Float64,queue_size=5)
	def MPU_Init(self):	
        # write to sample rate register
        # sample rate = 1000/(1+rate) rate =7
		self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 0x00)

		# Write to power management register
		self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)

		# Write to Configuration register
		self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)

		# Write to Gyro configuration register
		self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 0x18)
		self.bus.write_byte_data(self.Device_Address, self.MPU6050_RA_ACCEL_CONFIG, 0x10)
		self.bus.write_byte_data(self.Device_Address, self.MPU6050_RA_CONFIG, 0x00)

	def read_raw_data(self, addr):
	# Accelero and Gyro value are 16-bit
		high = self.bus.read_byte_data(self.Device_Address, addr)
		low = self.bus.read_byte_data(self.Device_Address, addr + 1)

		# concatenate higher and lower value
		value = ((high << 8) | low)

		# to get signed value from mpu6050
		if value > 32768:
			value = value - 65536
		return value

	def read_mpu_data(self):
		self.acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
		self.acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
		self.acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

		# Read Gyroscope raw value
		self.gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
		self.gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
		self.gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

	def update_quaternion(self):
		axg = (float)(self.acc_x - self.MPU6050_AXOFFSET) / self.MPU6050_AXGAIN
		ayg = (float)(self.acc_y - self.MPU6050_AYOFFSET) / self.MPU6050_AYGAIN
		azg = (float)(self.acc_z - self.MPU6050_AZOFFSET) / self.MPU6050_AZGAIN
		gxrs = (float)(self.gyro_x - self.MPU6050_GXOFFSET) / self.MPU6050_GXGAIN
		gyrs = (float)(self.gyro_y - self.MPU6050_GYOFFSET) / self.MPU6050_GYGAIN 
		gzrs = (float)(self.gyro_z - self.MPU6050_GZOFFSET) / self.MPU6050_GZGAIN
		return gxrs, gyrs, gzrs, axg, ayg, azg

	def MahonyAHRSupdateIMU(self, gx, gy, gz, ax, ay, az, sampleFreq):
		norm = 0.0
		halfvx = 0.0
		halfvy = 0.0
		halfvz = 0.0
		halfex = 0.0
		halfey = 0.0
		halfez = 0.0
		qa = 0.0
		qb = 0.0
		qc = 0.0
		if not((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
			norm = sqrt(ax * ax + ay * ay + az * az)
			ax /= norm
			ay /= norm
			az /= norm
			halfvx = self.q1 * self.q3 - self.q0 * self.q2
			halfvy = self.q0 * self.q1 + self.q3*self.q2
			halfvz = self.q0 * self.q0 - 0.5 + self.q3 * self.q3

			halfex = (ay * halfvz - az * halfvy)
			halfey = (az * halfvx - ax * halfvz)
			halfez = (ax * halfvy - ay * halfvx)
			if self.twoKi > 0.0:
				self.integralFBx += self.twoKi * halfex * (1.0 / sampleFreq)
				self.integralFBy += self.twoKi * halfey * (1.0 / sampleFreq)
				self.integralFBz += self.twoKi * halfez * (1.0 / sampleFreq)
				gx += self.integralFBx
				gy += self.integralFBy
				gz += self.integralFBz
			else:
				self.integralFBx = 0.0
				self.integralFBy = 0.0
				self.integralFBz = 0.0
				gx += self.twoKp * halfex
				gy += self.twoKp * halfey
				gz += self.twoKp * halfez
				gx *= (0.5 * (1.0 / sampleFreq))
				gy *= (0.5 * (1.0 / sampleFreq))
				gz *= (0.5 * (1.0 / sampleFreq))
				qa = self.q0
				qb = self.q1
				qc = self.q2
				self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
				self.q1 += (qa * gx + qc * gz - self.q3 * gy)
				self.q2 += (qa * gy - qb * gz + self.q3 * gx)
				self.q3 += (qa * gz + qb * gy - qc * gx)
				norm = sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
				self.q0 /= norm
				self.q1 /= norm
				self.q2 /= norm
				self.q3 /= norm
	def get_roll_pitch_yaw(self):
		yaw = -atan2(2.0 * (self.q1 * self.q2 + self.q0 * self.q3), self.q0 * self.q0 + self.q1 * self.q1 - self.q2 * self.q2 - self.q3 * self.q3) * 57.29577951
		pitch = asin(2.0 * (self.q0 * self.q2-self.q1 * self.q3)) * 57.29577951
		roll = atan2((self.q0 * self.q1 + self.q2 * self.q3), 0.5-(self.q1*self.q1+self.q2*self.q2)) * 57.29577951
		return roll, pitch, yaw
	def calib_mpu(self):
		cal_x = 0.0
		cal_y = 0.0
		cal_z = 0.0
		cal_gyro_x = 0.0
		cal_gyro_y = 0.0
		cal_gyro_z = 0.0
		count_print = 0
		for i in range(0, 2000):
			count_print += 1
			self.read_mpu_data()
			cal_x += self.acc_x
			cal_y += self.acc_y
			cal_z += self.acc_z
			cal_gyro_x += self.gyro_x
			cal_gyro_y += self.gyro_y
			cal_gyro_z += self.gyro_z
			if count_print >= 20:
				count_print = 0
				print(self.acc_x)
				print(self.acc_y)
				print(self.acc_z)
				print(self.gyro_x)
				print(self.gyro_y)
				print(self.gyro_z)
			rospy.sleep(0.01)
		print("done")
		self.MPU6050_AXOFFSET = cal_x / 2000.0
		self.MPU6050_AYOFFSET = cal_y / 2000.0
		self.MPU6050_AZOFFSET = cal_z / 2000.0
		self.MPU6050_GXOFFSET = cal_gyro_x/2000.0
		self.MPU6050_GYOFFSET = cal_gyro_y/2000.0
		self.MPU6050_GZOFFSET = cal_gyro_z/2000.0
		print(self.MPU6050_AXOFFSET)
		print(self.MPU6050_AYOFFSET)
		print(self.MPU6050_AZOFFSET)
		print(self.MPU6050_GXOFFSET)
		print(self.MPU6050_GYOFFSET)
		print(self.MPU6050_GZOFFSET)
	def run(self):
		self.calib_mpu()
		start_time=rospy.Time.now()
		rate=rospy.Rate(100)
		roll=0
		pitch=0
		pitch_now=0
		count_=0
		last_time_2=rospy.Time.now()
		d_gyro=0
		angle_pub=0
		while not rospy.is_shutdown():
			self.read_mpu_data()
			gx,gy,gz,ax,ay,az=self.update_quaternion()
	#Read Accelerometer raw value
			current_time=rospy.Time.now()
			elapstime=(current_time-start_time).to_sec()
			start_time=current_time
			roll += gx * elapstime
			pitch += gy * elapstime
			# self.MahonyAHRSupdateIMU(gx,gy,gz,ax,ay,az,(1.0/elapstime))
			# r,p,y=self.get_roll_pitch_yaw()
			time_publish=(current_time-last_time_2).to_sec()
			if(time_publish>=0.05):
				#print("r:"+str(r))
				count_=0
				theta_now=Float64()
			
				theta_now.data=pitch
				# print("pitch2:"+str(theta_now.data))
				# print(time_publish)
				last_time_2=current_time
				self.mpu_pub.publish(theta_now)
			# rate.sleep()
		
if __name__ == '__main__':
	try:
		mpu_class=MPU6050()
		mpu_class.run()
		#main()
	except rospy.ROSInterruptException:
		pass
