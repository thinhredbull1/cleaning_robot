#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class CmdVelTimeout:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('cmd_vel_timeout', anonymous=True)
        
        # Thời gian timeout (0.3 giây)
        self.timeout = 0.5
        
        # Biến lưu thời gian nhận message cmd_vel cuối cùng
        self.last_cmd_vel_time = rospy.get_time()
        
        # Biến để kiểm tra xem đã publish 0 chưa
        self.has_published_zero = False
        
        # Subscriber cho topic cmd_vel
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publisher cho topic cmd_vel_timeout
        self.pub = rospy.Publisher('cmd_vel_timeout', Twist, queue_size=10)
        
        # Tần suất kiểm tra timeout (10Hz)
        self.rate = rospy.Rate(20)
        
    def cmd_vel_callback(self, msg):
        # Khi nhận được message mới, publish ngay lập tức
        self.pub.publish(msg)
        self.last_cmd_vel_time = rospy.get_time()
        self.has_published_zero = False  # Reset trạng thái publish 0
        
    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            
            # Kiểm tra timeout
            if not self.has_published_zero and (current_time - self.last_cmd_vel_time) > self.timeout:
                # Nếu quá 0.3s và chưa publish 0, publish message với tất cả giá trị 0
                zero_msg = Twist()
                zero_msg.linear.x = 0.0
                zero_msg.linear.y = 0.0
                zero_msg.linear.z = 0.0
                zero_msg.angular.x = 0.0
                zero_msg.angular.y = 0.0
                zero_msg.angular.z = 0.0
                self.pub.publish(zero_msg)
                self.has_published_zero = True  # Đánh dấu đã publish 0
                # rospy.loginfo("Timeout: Published zero velocity")
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CmdVelTimeout()
        node.run()
    except rospy.ROSInterruptException:
        pass