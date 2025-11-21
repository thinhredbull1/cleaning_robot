#!/usr/bin/env python3
import time
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
        self.phone_number="+84123456789"  # Số điện thoại nhận SMS
        self.use_sim = False
        self.ser_sim = None
        # Mở hai cổng serial
        try:
            self.ser_ard = serial.Serial(port1, baudrate_ard, timeout=0)
            if self.use_sim:
                self.ser_sim = serial.Serial(port2, baudrate_sim, timeout=0)
        except serial.SerialException as e:
            rospy.logerr("Không mở được cổng serial: %s", e)
            rospy.signal_shutdown("Serial error")
            return
        if self.use_sim:
        ## check sim module ready

            if not self.wait_for_module_ready():
                rospy.signal_shutdown("SIM7600 not ready")
                return
            ## check signal
            if not self.check_signal():
                rospy.signal_shutdown("No signal")
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
    def send_command(self, command):
        print(f"Send to sim: {command}")
        self.ser_sim.write((command + "\r\n").encode('utf-8'))
    def send_command_and_check(self, command,timeout=2,wait_ok="OK"):
        self.send_command(command)
        time_start = time.time()
        while (time.time() - time_start) < timeout:
            if self.ser_sim.in_waiting > 0:
                    line = self.ser_sim.readline().decode('utf-8', errors='ignore').strip()
                    # print(f"Nhận: {line}")
                    response += line + "\n"
                    if wait_ok in line:
                        print("sim7600! OK")
                        return True, response
            time.sleep(0.15)
        return False, response
    def wait_for_module_ready(self, max_attempts=3):
   
        print("WaitSim..")
        self.ser_sim.flushInput()
        for i in range(max_attempts):
            success,response = self.send_command_and_check("AT",1)
            if(success):
                print("SIM7600 is ready!")
                return True
            
        print("cannot connect SIM7600!")
        return False
    def send_sms(self, message):
      
        success, _ = self.send_command_and_check("AT+CMGF=1")
        if not success:
            print("Cannot set text mode")
            return False

        success, _ = self.send_command_and_check("AT+CSCS=\"UCS2\"")
        if not success:
            print("Cannot set UCS2")
            return False

        def to_ucs2(text):
            return ''.join(f'{ord(c):04X}' for c in text)
        phone_ucs2 = to_ucs2(self.phone_number)
        message_ucs2 = to_ucs2(message)

        # Gửi lệnh AT+CMGS
        cmd = f'AT+CMGS="{phone_ucs2}"'
        success, resp = self.send_command_and_check(cmd,1,wait_ok=">")  # Đợi dấu > để nhập nội dung
        if not success:
            print("Cannot see > prompt")
            return False

        # Gửi nội dung tin nhắn + Ctrl+Z (0x1A)
        print(f"Sending message...")
        self.ser_sim.write((message_ucs2 + "\r").encode('utf-8'))
        time.sleep(1)
        self.ser_sim.write(bytes([0x1A]))  # Ctrl+Z
        self.ser_sim.flush()
        # Đợi phản hồi +CMGS: và OK
        time_start = time.time()
        while (time.time() - time_start) < 30: 
            if self.ser_sim.in_waiting:
                line = self.ser_sim.readline().decode('utf-8', errors='ignore').strip()
                print(f"Get: {line}")
                if "+CMGS:" in line:
                    print("Message has been successfully sent t")
                if "OK" in line:
                    print("SMS sent successfully!")
                    return True
            time.sleep(0.5)
        print("Timeout  SMS")
        return False
 
    def check_signal(self):
  
        success,response = self.send_command_and_check("AT+CSQ",2)
        if not success:
            print("Waving command failed")
            return False
        for line in response.split('\n'):
            if "+CSQ:" in line:
                try:
                    csq_value = int(line.split(':')[1].split(',')[0].strip())
                    print(f"Quality: {csq_value}/31")
                    if csq_value >= 10:
                        print("Good signal, can send SMS")
                        return True
                    else:
                        print("Weak signal, still try to send...")
                        return True
                except:
                    pass
        return False
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
    def state_callback(self, msg):
        if self.state == State.WAITING and msg.data =="DOING":
            # get doing --> recv from arduino --> done --> moving
            rospy.loginfo("Get start transfer to DOING")
            self.state = State.DOING

        if self.state == State.DOING and msg.data =="SUCCESS":
            rospy.loginfo("Done navigation process SUCCESS action")
            self.state = State.SUCCESS
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