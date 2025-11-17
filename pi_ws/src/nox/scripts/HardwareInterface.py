#!/usr/bin/env python3
import rospy
import serial
import select
from enum import Enum
from std_msgs.msg import String
class State(Enum):
    WAITING=1
    DOING =2
    SUCCESS =3
class DualSerialNode:
    def __init__(self):
        # Lấy thông số từ ROS param
        port1 = rospy.get_param('~port_arduino', '/dev/ttyUSB0')
        port2 = rospy.get_param('~port_sim', '/dev/ttyUSB1')
        baudrate_ard = rospy.get_param('~baudrate_ard', 57600)
        baudrate_sim = rospy.get_param('~baudrate_sim', 115200)
        # Mở hai cổng serial
        try:
            self.ser_ard = serial.Serial(port1, baudrate_ard, timeout=0)
            self.ser_sim = serial.Serial(port2, baudrate_sim, timeout=0)
        except serial.SerialException as e:
            rospy.logerr("Không mở được cổng serial: %s", e)
            rospy.signal_shutdown("Serial error")
            return

        rospy.Subscriber('state_cmd', String, self.state_callback)
        self.state_publish=rospy.Publisher('state_cmd', String, queue_size=1)
        #state init
        self.state = State.WAITING
        self.pending_action = None

        # Subscriber cho dữ liệu gửi xuống
        rospy.Subscriber('serial1_tx', String, self.send_to_port1)
        rospy.Subscriber('serial2_tx', String, self.send_to_port2)

        # Đăng ký shutdown hook
        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("DualSerialNode started with ports: %s and %s", port1, port2)
    
    def state_callback(self, msg):
        if self.state == State.WAITING and msg.data =="DOING":
            # get doing --> recv from arduino --> done --> moving
            rospy.loginfo("Get start transfer to DOING")
            self.state = State.DOING

        if self.state == State.DOING and msg.data =="SUCCESS":
            rospy.loginfo("Done navigation process SUCCESS action")
            self.state = State.SUCCESS
    def send_to_port1(self, msg):
        if self.ser_ard.is_open:
            rospy.loginfo("[Port1 TX] %s", msg.data)
            self.ser_ard.write((msg.data + '\n').encode('utf-8'))

    def send_to_port2(self, msg):
        if self.ser_sim.is_open:
            rospy.loginfo("[Port2 TX] %s", msg.data)
            self.ser_sim.write((msg.data + '\n').encode('utf-8'))
    def processDoing(self):
        if self.state == State.DOING and self.pending_action != "sent_doing":
            rospy.loginfo("DOING init")
            if self.ser_ard.is_open:
                self.ser_ard.write(b'DOING_CMD\n')
            else:
                rospy.logwarn("arduino not open")
            # self.pending_action = "sent_doing"
    def processSucess(self):
        if self.state == State.SUCCESS and self.pending_action != "sent_success":
            rospy.loginfo("SUCCESS STATE WAIT ARDUINO")
            self.ser_ard.write(b'SUCCESS_CMD\n')
            
            # self.pending_action = "sent_success"
    def spin(self):
        rate = rospy.Rate(10)  # 50 Hz để tránh busy-wait
        i = 0
        while not rospy.is_shutdown():
            rlist, _, _ = select.select([self.ser_ard, self.ser_sim], [], [], 0.01)
            for ser in rlist:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    port_name = "Port_sim" if ser == self.ser_sim else "port_ard"
                    rospy.loginfo("[%s RX] %s", port_name, data)

                    if self.state == State.SUCCESS and port_name == "port_ard":
                        rospy.loginfo("GET arduino sucess transform cmd")
                        if data=="OK":
                            self.state = State.WAITING
                            self.pending_action = "sent_success"
                            self.ser_sim.write(b'SUCCESS_CMD\n')
                            rospy.loginfo("WAITING TRANSFER")
                    if self.state == State.DOING and port_name == "port_ard":
                        rospy.loginfo("GET arduino doing transform cmd")
                        if data=="OK":
                            self.pending_action = "sent_doing"
                            self.state_publish.publish("MOVING")
                            self.ser_sim.write(b'SUCCESS_CMD\n')
                            rospy.loginfo("WAITING DOING")
            if self.state == State.WAITING:
                self.pending_action = None
            i+=1
            if(i>=2):
                self.processDoing()
                self.processSucess()
                i=0

            rate.sleep()

    def cleanup(self):
        rospy.loginfo("Đang đóng cổng serial...")
        if self.ser1.is_open:
            self.ser1.close()
        if self.ser2.is_open:
            self.ser2.close()
        rospy.loginfo("Đã đóng cổng serial thành công.")

if __name__ == '__main__':
    rospy.init_node('dual_serial_node')
    node = DualSerialNode()
    node.spin()