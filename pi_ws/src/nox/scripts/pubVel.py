#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tf
from enum import Enum
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import numpy as np
import math
import sys
import os
import time
import rospkg
from std_msgs.msg import Bool,Int32,Float32
from geometry_msgs.msg import Twist
import serial

class MecanumRobot:
    def __init__(self):
        rospy.init_node('mecanum_robot')
        print("INIT")
        self.NMOTORS = 2  
        self.MotorError=False
        self.total_length = 14.82
          # Adjust this value based on your robot design
        #length=41.5/2=20.75
        #width =34.3/2=17.15
        self.onOffMor=False
        self.M_LEFT=0
        self.M_RIGHT=1
        self.M_RIGHT_DOWN=0
        self.acc_time=10
        self.dec_time=10
        self.M_RIGHT_UP=1
        self.M_LEFT_DOWN=2
        self.M_LEFT_UP=3
        self.moving=False
        # self.x_offset = 1.0  # Scaling factor for x
        # self.y_offset = 1.0  # Scaling factor for y
        # self.theta_offset = 1.0  # Scaling factor for theta
        self.use_imu=rospy.get_param('~use_imu',False) #cm
        print(self.use_imu)
        self.pose_sub = rospy.Subscriber('/pose2D', Pose2D, self.pose_callback)
        rospy.Subscriber('/toggle_topic',Int32, self.CylinderCB)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.last_pose = None
        self.last_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=5)
        # print("not use imu")
        self.WHEEL_DIAMETER=rospy.get_param('~wheel_diameter',15.2) #cm
        self.ENCODER_TOTAL=rospy.get_param('~encoder_total',1000)
        self.encoder_total=np.zeros(4)
        self.test_mode=rospy.get_param('~test_mode',0) # 0 - speed 1 - position 2 - normal
        self.start_velocity_test=rospy.get_param('~start_run', 1.0)
        self.x_offset= rospy.get_param('~x_offset', 1.0)
        self.y_offset = rospy.get_param('~y_offset', 1.0)
        self.theta_offset = rospy.get_param('~theta_offset', 1.0)
        self.distance_move= rospy.get_param('~distance_move', 1.0)
        self.axis_move = rospy.get_param('~axis_move', 1.0)
        self.velocity_move = rospy.get_param('~velocity_move', 1.0)
        self.robot_pose = np.zeros(3)  # [x, y, theta]
        self.speed_desired = np.zeros(self.NMOTORS)  # Desired speeds for each motor cm/s
        # "/final_cmd_vel_mux/output"
        self.speed_filter_x=0.0
        self.speed_filter_y=0.0
        self.speed_filter_z=0.0
        self.last_speed_x=0.0
        self.last_speed_y=0.0
        self.last_speed_z=0.0
        self.a_coeff=0.52188555
        self.b_coeff=0.23905722
        self.xylanh=1
        self.cmd_msg = Twist()  # Initialize a Twist message
        # self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.MotorErrorPub=rospy.Publisher('MOTOR_ERROR',Bool,queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.delta_encod_total1=np.zeros(4)
        self.serial_port_name = rospy.get_param('port', '/dev/ttyACM0')
        self.baud = rospy.get_param('baud', 57600)
        # self.pub_encoder=rospy.publish
        self.serial_port = serial.Serial(self.serial_port_name,  self.baud)
        # modbus_connection = ZLAC8015D.ModbusConnection(port="/dev/ttyUSB0")
        # rospy.sleep(1.5)
        self.desired = rospy.Publisher('/speed_desired', Float32, queue_size=1)
        self.feedback = rospy.Publisher('/speed_feedback', Float32, queue_size=1)
        self.last_time_encod=rospy.Time.now()
        self.inverseDir=[]
        self.teleop=False
        self.lastEncoderTick=[0,0,0,0]
        self.delta_cvt = 60.0/(math.pi * self.WHEEL_DIAMETER)  # cm_s --> rpm
        self.vx_now=0
        self.vy_now=0
        self.theta_now=0
        self.last_msg=1
       
        self.vx_run=0
        self.vy_run=0
        self.w_run=0
     
        # self.motors=[self.motorUp,self.motorDown]
        # self.last_time=time.time()
        self.speed_wheel_cm_s=[0,0,0,0]
        self.getCmdVel=False
        self.WHEEL_DIAMETER=6.5
        self.ENCODER_TOTAL=2464 #3696 datasheet 4271 get
        self.cmPerCount=(math.pi * self.WHEEL_DIAMETER) / self.ENCODER_TOTAL # 7.05
        # rospy.sleep(1.5)
        # self.motorUp=ZLAC8015D.Controller(modbus_connection,id=1) # 0 forn
        # self.motorDown=ZLAC8015D.Controller(modbus_connection,id=2) # 1
        self.stop_all=False
        self.ms_pub_encoder=20.0
        self.rate_hz=(2000.0/self.ms_pub_encoder)
        # self.dt=1.0/self.rate_hz
        self.ms_pid=10
        print(f"total:{self.total_length}")
        self.rpm_to_cm_s=np.zeros(2)
        self.rate = rospy.Rate(self.rate_hz)  # 10 Hz
        rospy.Subscriber("/cmd_vel", Twist, self.cmdVelCb)  # Change to Twist message
    def cmdVelCb(self,msg):
        self.getCmdVel=True
        self.vx_run=msg.linear.x*100
        self.w_run=msg.angular.z
        self.calSpeed(self.vx_run,self.w_run)
        self.runRobot()
    def CylinderCB(self,msg):
        self.xylanh=msg.data
    def updatePos(self):
        encoderTick=[0,0]
        delta_tick=[0,0,0,0]
        delta=[0,0,0,0]
   
        self.rpm_to_cm_s[self.M_LEFT]=self.encoder_total[0]
        self.rpm_to_cm_s[self.M_RIGHT]=self.encoder_total[1]
        for i in range(self.NMOTORS):
            delta[i] =  self.rpm_to_cm_s[i] *self.cmPerCount  # 0.14260
        dxy = (delta[self.M_LEFT]+delta[self.M_RIGHT])
        dtheta = -((delta[self.M_LEFT]-delta[self.M_RIGHT]))/self.total_length
        dx = math.cos(dtheta) * dxy
        dy = math.sin(dtheta) * dxy
        self.vx_now=0
        self.vy_now=0
        self.theta_now=0
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time_encod).to_sec()
        dx=dx/100.0
        dy=dy/100.0
        # print(dt)
        #encoder chay lau troi --> 
        #gps sai so tinh 1-2m 
        ## fusion localization--> EKF extended kalmen filter
        if dt>0:
            # dt=dt
            # dtheta=dtheta
            self.vx_now=dx/dt
            self.vy_now=dy/dt
            self.theta_now=dtheta/dt

        self.robot_pose[0] +=dx * self.x_offset*math.cos(self.robot_pose[2]) -dy*math.sin(self.robot_pose[2])
        self.robot_pose[1] += dy * self.y_offset*math.cos(self.robot_pose[2]) + dx*math.sin(self.robot_pose[2])
        self.robot_pose[2] += dtheta * self.theta_offset
        if(self.robot_pose[2])>=math.pi:
            self.robot_pose[2]-=2.0*math.pi
        elif(self.robot_pose[2]<=math.pi):
            self.robot_pose[2]+=2.0*math.pi
        # print(f"x:{self.theta_now} ; y:{self.robot_pose[2]}")
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # print(f"x:{self.robot_pose[0]} y:{self.robot_pose[1]}")
        odom_x=self.robot_pose[0]
        odom_y=self.robot_pose[1]
        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from yaw angle
        q = tf.transformations.quaternion_from_euler(0, 0, self.robot_pose[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance = [
        0.1, 0,    0,    0,    0,    0,
        0,    0.1, 0,    0,    0,    0,
        0,    0,    1e6, 0,    0,    0,
        0,    0,    0,    1e6, 0,    0,
        0,    0,    0,    0,    1e6, 0,
        0,    0,    0,    0,    0,    0.1
        ]
        odom.twist.covariance = [
        0.05, 0,    0,    0,    0,    0,
        0,    0.05, 0,    0,    0,    0,
        0,    0,    1e6, 0,    0,    0,
        0,    0,    0,    1e6, 0,    0,
        0,    0,    0,    0,    1e6, 0,
        0,    0,    0,    0,    0,    0.1
        ]
        # odom.pose.covariance = ODOM_POSE_COVARIANCE
        # odom.twist.covariance = ODOM_TWIST_COVARIANCE
        # print(self.vx_now)
        speed_measure_x=self.vx_now
        speed_measure_y=self.vy_now
        speed_measure_z=self.theta_now
        self.speed_filter_x=self.a_coeff * self.speed_filter_x + self.b_coeff * speed_measure_x + self.last_speed_x * self.b_coeff
        self.speed_filter_y=self.a_coeff * self.speed_filter_y + self.b_coeff * speed_measure_y + self.last_speed_y * self.b_coeff
        self.speed_filter_z=self.a_coeff * self.speed_filter_z + self.b_coeff * speed_measure_z + self.last_speed_z * self.b_coeff
        self.last_speed_x=speed_measure_x
        self.last_speed_y=speed_measure_y
        self.last_speed_z=speed_measure_z
        odom.twist.twist.linear.x =self.speed_filter_x
        odom.twist.twist.angular.z =self.speed_filter_z 
        odom.twist.twist.linear.y =-self.speed_filter_y
        self.odom_pub.publish(odom)

        # Publish TF
        
        self.tf_broadcaster.sendTransform(
            (odom_x, odom_y, 0),
            q,
            rospy.Time.now(),
            "base_link",
            "odom"
        )
            # print(f"{self.theta_now}")
        self.last_time_encod=current_time

    def calSpeed(self, vx, dtheta):
        # vx w --> 
        # 
        #
        ## 1 toa do (x,y) dich --> toa do hien tai (x1,y1) --> path planning --> path tracking
        # dx = x-x1 --> t =0 --> vx =0 --> x =1 --> v=v0+v*t --> v0 v1 v2 v3 v4 = --> dx/dt =v hien tai
        ## 5 hz 0.2s 
        ## vx w --> speed dong co
        speed_cm_s = np.zeros(self.NMOTORS)
        speed_cm_s[self.M_LEFT] = (vx  - dtheta * self.total_length)
        speed_cm_s[self.M_RIGHT] = (vx  + dtheta * self.total_length)
        #
        # print(speed_cm_s)
        for i in range(self.NMOTORS):
            # self.speed_desired[i] = speed_cm_s[i]*1.5 # to pulse / 10ms
            self.speed_desired[i] = speed_cm_s[i] *(1.0/(self.cmPerCount*(1000.0/self.ms_pid)))
        # print(self.speed_desired)
    def runRobot(self):
        speed_wheel=[0,0,0,0]
        for i in range(self.NMOTORS):
            # if(i in self.inverseDir):
            #     self.speed_desired[i]=-self.speed_desired[i]
            speed_wheel[i]=int(self.speed_desired[i])
            if(speed_wheel[i]>50):
                speed_wheel[i]=50
            elif speed_wheel[i]<-50:
                speed_wheel[i]=-50
        serial_data = "{}/{}&0*0;".format(speed_wheel[self.M_LEFT], speed_wheel[self.M_RIGHT])
        self.moving=True
        # Gửi dữ liệu xuống serial
        # print(serial_data)
        self.serial_port.write(serial_data.encode())
        # print(speed_wheel)
        # self.motorUp.set_rpm(speed_wheel[self.M_RIGHT_UP],speed_wheel[self.M_LEFT_UP])
        # self.motorDown.set_rpm(speed_wheel[self.M_LEFT_DOWN],speed_wheel[self.M_RIGHT_DOWN])
    def pose_callback(self, msg):
        # Thời gian hiện tại
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        if self.last_pose is not None and dt > 0:
            # Tính toán sự thay đổi về vị trí và góc quay
            x_now=msg.x
            y_now=msg.y
            theta_now_=msg.theta
            delta_x =x_now - self.last_pose.x
            delta_y =y_now - self.last_pose.y
            delta_yaw =theta_now_ - self.last_pose.theta

            # Tính vận tốc tuyến tính
            linear_velocity_x =delta_x / dt
            linear_velocity_y =delta_y / dt
            if abs(linear_velocity_x)<0.005:
                linear_velocity_x=0
            if abs(linear_velocity_y)<0.005:
                linear_velocity_y=0
            
            # Tính vận tốc góc (yaw velocity)
            angular_velocity = delta_yaw / dt
            if abs(angular_velocity)<0.005:
                angular_velocity=0
            quaternion = quaternion_from_euler(0, 0,theta_now_)
            print(f"X:{linear_velocity_x} theta:{angular_velocity}")
            diff_x=0
            diff_theta=0
            if(self.speed_filter_x!=0):
                diff_x=linear_velocity_x/self.speed_filter_x
            if(self.speed_filter_z!=0):
                diff_theta=angular_velocity/self.speed_filter_z
            print(f"Calib X:{diff_x} Calib theta:{diff_theta}")
            # Tạo thông điệp Odometry
            # odom = Odometry()
            # odom.header.stamp = current_time
            # odom.header.frame_id = "odom"
            # odom.child_frame_id = "base_link"
            # odom.pose.covariance = [
            # 0.1, 0,    0,    0,    0,    0,
            # 0,    0.1, 0,    0,    0,    0,
            # 0,    0,    1e6, 0,    0,    0,
            # 0,    0,    0,    1e6, 0,    0,
            # 0,    0,    0,    0,    1e6, 0,
            # 0,    0,    0,    0,    0,    0.1
            # ]
            # odom.twist.covariance = [
            # 0.05, 0,    0,    0,    0,    0,
            # 0,    0.05, 0,    0,    0,    0,
            # 0,    0,    1e6, 0,    0,    0,
            # 0,    0,    0,    1e6, 0,    0,
            # 0,    0,    0,    0,    1e6, 0,
            # 0,    0,    0,    0,    0,    0.1
            # ]
            # # Tọa độ và góc quay
        
            # odom.pose.pose.position.x = x_now
            # odom.pose.pose.position.y = y_now
            # odom.pose.pose.orientation.x = quaternion[0]
            # odom.pose.pose.orientation.y = quaternion[1]
            # odom.pose.pose.orientation.z = quaternion[2]
            # odom.pose.pose.orientation.w = quaternion[3]
            # # Vận tốc
            # odom.twist.twist.linear.x = linear_velocity_x
            # odom.twist.twist.linear.y = linear_velocity_y

            # odom.twist.twist.angular.z = angular_velocity
            # self.tf_broadcaster.sendTransform(
            #     (x_now,y_now, 0),  # Position
            #     quaternion,          # Orientation (quaternion)
            #     current_time,        # Time
            #     "base_link",         # Child frame
            #     "odom"               # Parent frame
            # )
            # # Publish Odometry message
            # self.odom_pub.publish(odom)

        # Cập nhật giá trị Pose và thời gian
        self.last_pose = msg
        self.last_time = current_time

  
       
    def run(self):
        count_to_stop=0
        
        print("Encoder:"+str(self.ENCODER_TOTAL))
        print("Mode: "+str(self.test_mode))
        print("Wheel: "+str(self.WHEEL_DIAMETER))
        print("x_offset:"+str(self.x_offset))
        count_publish=0
        count_print=0
        t=time.time()
        t2=time.time()
        t3=time.time()
        send_cmd_cylen=True
        # rospy.spin()
        

        while not rospy.is_shutdown():
            try:
                if self.serial_port.in_waiting > 0:
                    # Đọc một dòng dữ liệu từ STM32
                    data_line = self.serial_port.readline().decode('utf-8').strip()
                    # Giả sử định dạng dữ liệu từ STM32: "x/y&z*w\r\n"
                    parts = data_line.split('/')
                    if len(parts) == 2:
                        self.encoder_total[0] = int(parts[0])
                        self.encoder_total[1] = int(parts[1])
                        self.updatePos()
                        # rospy.loginfo(f"Encoders: {self.encoder_total}")
            except Exception as e:
                rospy.logwarn(f"Error reading serial data: {e}")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = MecanumRobot()
        robot.run()
    except rospy.ROSInterruptException:
        robot.calSpeed(0,0,0)
        robot.runRobot()
        # robot.disableMor()
        rospy.loginfo("ROS node interrupted.")
    finally:
        rospy.loginfo("Exiting program.")
