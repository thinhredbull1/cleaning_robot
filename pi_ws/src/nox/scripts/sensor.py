#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading

from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension


class ESP32SerialNode:

    def __init__(self):

        rospy.init_node("esp32_serial_node")

        # =====================================
        # SERIAL CONFIG
        # =====================================

        self.port = rospy.get_param(
            "~port", "/dev/ttyUSB0"
        )

        self.baudrate = rospy.get_param(
            "~baudrate", 57600
        )

        self.timeout = rospy.get_param(
            "~timeout", 0.1
        )

        self.serial_lock = threading.Lock()

        self.ser = None

        self.connect_serial()

        # =====================================
        # ROS PUBLISHERS
        # =====================================

        self.sensor_pub = rospy.Publisher(
            "/esp32/sensor_data",
            Float32MultiArray,
            queue_size=20
        )

        # =====================================
        # ROS SUBSCRIBERS
        # =====================================

        self.command_sub = rospy.Subscriber(
            "/esp32/command",
            String,
            self.command_callback,
            queue_size=20
        )

        rospy.loginfo("================================")
        rospy.loginfo(" ESP32 SERIAL NODE STARTED")
        rospy.loginfo("================================")

        rospy.loginfo("Serial: %s", self.port)
        rospy.loginfo("Baudrate: %d", self.baudrate)

    # =========================================
    # SERIAL CONNECTION
    # =========================================

    def connect_serial(self):

        try:

            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )

            # ESP32 co the reset khi mo Serial
            rospy.sleep(2.0)

            self.ser.reset_input_buffer()

            rospy.loginfo("ESP32 SERIAL CONNECTED!")

        except serial.SerialException as e:

            self.ser = None

            rospy.logerr(
                "Serial connection failed: %s", str(e)
            )

    # =========================================
    # SEND COMMAND TO ESP32
    # =========================================

    def command_callback(self, msg):

        command = msg.data.strip()

        if not command:
            return

        if self.ser is None or not self.ser.is_open:

            rospy.logerr("ESP32 SERIAL NOT CONNECTED!")
            return

        try:

            with self.serial_lock:

                data = (command + "\n").encode("utf-8")

                self.ser.write(data)

                self.ser.flush()

            rospy.loginfo(
                "[TX ESP32] %s", command
            )

        except serial.SerialException as e:

            rospy.logerr(
                "Serial write error: %s", str(e)
            )

    # =========================================
    # PARSE SENSOR DATA
    # =========================================

    def parse_sensor(self, line):

        line = line.strip()
        print(line)
        if not line.startswith("SENSOR,"):
            return

        try:

            values = line.split(",")

            # SENSOR + 8 gia tri
            if len(values) != 9:
                rospy.logwarn(
                    "Invalid sensor data: %s", line
                )
                return

            temperature = float(values[1])
            humidity = float(values[2])
            mq_adc = float(values[3])
            battery_adc = float(values[4])
            battery_voltage = float(values[5])
            dust_adc = float(values[6])
            dust_voltage = float(values[7])
            dust_density = float(values[8])

            # =================================
            # PUBLISH FLOAT32 ARRAY
            # =================================

            msg = Float32MultiArray()

            msg.layout.dim.append(
                MultiArrayDimension(
                    label="temperature,humidity,mq_adc,"
                          "battery_adc,battery_voltage,"
                          "dust_adc,dust_voltage,dust_density",
                    size=8,
                    stride=8
                )
            )

            msg.layout.data_offset = 0

            msg.data = [
                temperature,
                humidity,
                mq_adc,
                battery_adc,
                battery_voltage,
                dust_adc,
                dust_voltage,
                dust_density
            ]

            self.sensor_pub.publish(msg)

            rospy.loginfo_throttle(
                2.0,
                "[RX ESP32] T=%.1f H=%.1f MQ=%.1f "
                "BAT=%.2f DUST=%.2f",
                temperature,
                humidity,
                mq_adc,
                battery_voltage,
                dust_density
            )

        except (ValueError, IndexError) as e:

            rospy.logwarn(
                "Sensor parsing error: %s | %s",
                str(e), line
            )

    # =========================================
    # READ SERIAL
    # =========================================

    def read_serial(self):

        if self.ser is None or not self.ser.is_open:
            return

        try:

            with self.serial_lock:

                if self.ser.in_waiting > 0:

                    line = self.ser.readline().decode(
                        "utf-8",
                        errors="ignore"
                    ).strip()

                else:
                    line = ""

            if line:

                self.parse_sensor(line)

        except serial.SerialException as e:

            rospy.logerr_throttle(
                2.0,
                "Serial read error: %s", str(e)
            )

    # =========================================
    # MAIN LOOP
    # =========================================

    def run(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            self.read_serial()

            rate.sleep()

        if self.ser is not None:

            self.ser.close()

            rospy.loginfo("ESP32 SERIAL CLOSED")


if __name__ == "__main__":

    try:

        node = ESP32SerialNode()

        node.run()

    except rospy.ROSInterruptException:

        pass
