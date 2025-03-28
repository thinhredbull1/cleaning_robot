#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import serial
# import re
import json
import threading
from datetime import datetime
import paho.mqtt.client as mqtt
import time

# MQTT settings
broker = "broker.emqx.io"  # Broker
port = 1883
topic_subscribe = "gps_rbotss/device1_recv"
topic_publish = "gps_rbotss/device1_send"

# ROS topic settings
ros_topic = "gps/fix"
# mqtt -> pi  pi -> cac chuong trinh ros khac
# Global variables
lat = 21.03627055
lon = 105.7869495
bat = 76
speed = 0
ros_publish_desired = None
def on_message(client, userdata, message):
    global ros_publish_desired
    data_rcv = json.loads(message.payload.decode())
    new_lat = data_rcv["lat"]
    new_lon = data_rcv["lon"]
    state = data_rcv["state"]
    rospy.loginfo(f"New loc: {new_lat}, {new_lon}")
    rospy.loginfo(f"State: {state}")
    nav_sat_msg = NavSatFix()
    nav_sat_msg.header.stamp = rospy.Time.now()
    nav_sat_msg.header.frame_id = "gps_frame"
    nav_sat_msg.latitude = new_lat
    nav_sat_msg.longitude = new_lon
    nav_sat_msg.altitude = 0.0  # Set to 0 or provide the correct altitude if available
    nav_sat_msg.position_covariance = [4.0, 0.0, 0.0,
                                      0.0, 4.0, 0.0,
                                      0.0, 0.0, 16.0]  # Z-axis uncertainty larger
    nav_sat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    # Publish the new coordinates to gps/desired
    ros_publish_desired.publish(nav_sat_msg)
    rospy.loginfo(f"Published to gps/desired: {nav_sat_msg}")
def mqtt_subscriber():
    try:
        client = mqtt.Client()
        client.on_message = on_message
        client.connect(broker, port)
        client.subscribe(topic_subscribe)
        client.loop_forever()
    except Exception as e:
        rospy.logerr(f"Error in MQTT subscriber: {e}")

def mqtt_publisher(ros_publisher):
    try:
        client = mqtt.Client()
        client.connect(broker, port)

        serial_port = '/dev/ttyUSB1'  # Update with your USB port
        baud_rate = 115200
        count_=0
        # Open the serial connection once
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        #  giao tiep gps voi pi usb serial uart 
        while not rospy.is_shutdown():
            rospy.loginfo("Reading GPS data...")
            count_+=1
            # Read a line of data from the serial port
            line = ser.readline().decode('utf-8').strip()
            # line=""
            rospy.loginfo(f"Data received: {line}")
            #+CGPSINFO: 3,17,,,,2059.20778,N,10551.45592,E,180125,162511.00,36.8,3.374,,4.14,3.11,2.74
            if "+CGPSINFO" in line:
                latitude, longitude = parse_gps_data(line)
                if latitude and longitude:
                    global lat, lon
                    lat, lon = latitude, longitude

            datetime_send = datetime.now()
            message = {
                "lat": lat,
                "lon": lon,
                "speed": speed,
                "time": datetime_send.strftime("%Y-%m-%d %H:%M:%S"),
                "bat": bat
            }
            if count_>=5:
                data_send = json.dumps(message)
                client.publish(topic_publish, data_send)
                rospy.loginfo(f"Sent to MQTT: {data_send}")
                count_=0

            # Publish to ROS topic
            navsat_fix_msg = NavSatFix()
            navsat_fix_msg.header.stamp = rospy.Time.now()
            navsat_fix_msg.header.frame_id = "gps_frame"
            navsat_fix_msg.latitude = lat
            navsat_fix_msg.longitude = lon
            navsat_fix_msg.altitude = 0.0  # Đặt giá trị cố định hoặc tính toán nếu có
            navsat_fix_msg.position_covariance = [4.0, 0.0, 0.0,
                                                  0.0, 4.0, 0.0,
                                                  0.0, 0.0, 16.0]  # Z-axis uncertainty larger
            navsat_fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            ros_publisher.publish(navsat_fix_msg)
            # rospy.loginfo(f"Published to ROS: {navsat_fix_msg}")

            rospy.sleep(1)
    except Exception as e:
        rospy.logerr(f"Error in MQTT publisher: {e}")

def parse_gps_data(data):
    try:
        # Regex to extract GPS data
        # match = re.search(r'(\d+,\d+,,,,(\d+\.\d+),(N|S),(\d+\.\d+),(E|W),)', data)
        # if not match:
        #     return None, None

        # lat_raw = match.group(2)  # Latitude raw value
        # lat_dir = match.group(3)  # N or S
        # lon_raw = match.group(4)  # Longitude raw value
        # lon_dir = match.group(5)  # E or W
        data = data.split(":")[1].strip()
        if not data or data == ",,,,,,,,":  
            return None, None
        #+CGNSSINFO: 3,17,,,,2059.20778,N,10551.45592,E,180125,162511.00,36.8,3.374,,4.14,3.11,2.74
        lat_raw, lat_dir, lon_raw, lon_dir = data.split(",")[0], data.split(",")[1], data.split(",")[2], data.split(",")[3]
        # Convert raw GPS values to degrees
        
        lat_deg = int(float(lat_raw) / 100)
        lat_min = float(lat_raw) % 100
        latitude = lat_deg + lat_min / 60
        if lat_dir == 'S':
            latitude = -latitude

        lon_deg = int(float(lon_raw) / 100)
        lon_min = float(lon_raw) % 100
        longitude = lon_deg + lon_min / 60
        if lon_dir == 'W':
            longitude = -longitude

        return latitude, longitude
    except Exception as e:
        rospy.logerr(f"Error parsing GPS data: {e}")
        return None, None

if __name__ == "__main__":
    rospy.init_node("gps_mqtt_node", anonymous=True)
    ros_publish_desired = rospy.Publisher('gps/desired', NavSatFix, queue_size=1)
    # Create ROS publisher
    ros_publisher = rospy.Publisher(ros_topic, NavSatFix, queue_size=1)

    # Start MQTT subscriber in a separate thread
    thread_subscriber = threading.Thread(target=mqtt_subscriber, daemon=True)
    thread_subscriber.start()

    # Run MQTT publisher
    mqtt_publisher(ros_publisher)
