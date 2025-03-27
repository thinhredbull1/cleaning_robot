#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3Stamped
from math import cos
from math import sin
m_per_count_l=0.000812481*(1.0085)
m_per_count_r=0.000812481*(1.0-0.0085)
count_print=0
def run():
    # Khởi tạo node ROS
    
    
    rospy.init_node('odom_test_node',anonymous=True)
    
    # Đăng ký subscriber cho topic /speed
    
    # Tạo publisher cho topic /cmd_control
    pub = rospy.Publisher("/cmd_vel_2", Vector3Stamped, queue_size=10)
    
    cmd_msg = Vector3Stamped()
    
    # Thiết lập dữ liệu cho message mới
    rate_hz=10.0
    r=rospy.Rate(rate_hz)
    dt=1/rate_hz
    m_move_straight=0.6
    pi=3.141592653
    angle_turn=90
    speed_linear=0.12
    speed_turn=0.4
    rospy.sleep(1)
    time_stop_linear=(m_move_straight/speed_linear)
    time_stop_angular=(angle_turn*pi/180.0)/(speed_turn)
    count_to_stop_linear=0
    #count_to_stop_linear=(int)(time_stop_linear/dt)
    #count_to_stop_angular=0
    count_to_stop_angular=(int)(time_stop_angular/dt)
    print("start")
    while not rospy.is_shutdown():
        if(count_to_stop_linear>=0):
            cmd_msg.vector.x=speed_linear
            cmd_msg.vector.y=0
            cmd_msg.vector.z=0
            count_to_stop_linear-=1
            #print("linear")
        else:
            if(count_to_stop_angular>=0):
                cmd_msg.vector.x=0
                cmd_msg.vector.y=0
                cmd_msg.vector.z=speed_turn
                count_to_stop_angular-=1
            else:
                cmd_msg.vector.y=0
                cmd_msg.vector.x=0
                cmd_msg.vector.z=0
                print("done")
        pub.publish(cmd_msg)
        r.sleep()
    # Vòng lặp chính để giữ node ROS hoạt động

if __name__ == '__main__':
    try:
    	run()
    	#print("hello world")
    except rospy.ROSInterruptException:
    	pass

