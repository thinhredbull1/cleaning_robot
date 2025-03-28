#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from geographiclib.geodesic import Geodesic
import math
class GPSGoalConverter:
    def __init__(self):
        rospy.init_node('gps_goal_converter', anonymous=True)

        # Current GPS position and desired goal
        self.current_latitude = None
        self.current_longitude = None
        self.current_heading = None
        self.desired_latitude = None
        self.desired_longitude = None

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Publishers and Subscribers
        rospy.Subscriber('/gps', NavSatFix, self.current_gps_callback)
        rospy.Subscriber('/gps/desired', NavSatFix, self.desired_gps_callback)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        rospy.loginfo("GPS Goal Converter Node Initialized")
        rospy.spin()
    # gmapping --> toa do encoder --> laser --> laser to gridmap --> bresenham --> danh dau cac cai laser thanh tuong va chia nho luoi ra
    # gmapping --> odom de cap nhat vi tri qua thoi gian va pose graph slam  so sanh cac tuong  + grid mapping --> 
    def current_gps_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude

        # Get current heading from tf (map to base_link)
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            self.current_heading = euler[2]  # Yaw in radians
            rospy.loginfo(f"Current heading: {self.current_heading} rad")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")
            self.current_heading = 0.0  # Fallback to default

        rospy.loginfo(f"Current GPS position: ({self.current_latitude}, {self.current_longitude})")

    def desired_gps_callback(self, msg):
        self.desired_latitude = msg.latitude
        self.desired_longitude = msg.longitude

        rospy.loginfo(f"Desired GPS position received: ({self.desired_latitude}, {self.desired_longitude})")
        self.convert_and_publish_goal()
# latitude longitude --> x y he toa do UTM (GPS) --> he toa do UTM goc 0 la o  tay ban nha. --> robot dang o viet nam -->  0 0 --> 
# vi tri hien tai o toa do UTM la x = 50 y =60 0 0 --> vi tri dich latitude longtitude x = 80 y= 90 --> toa do robot can di chuyen
# dwa A* --> di chuyen toa do day
    def convert_and_publish_goal(self):
        if self.current_latitude is None or self.current_longitude is None:
            rospy.logwarn("Current GPS position is not available yet.")
            return

        if self.desired_latitude is None or self.desired_longitude is None:
            rospy.logwarn("Desired GPS position is not available yet.")
            return

        # Use GeographicLib to calculate the distance and bearing
        geod = Geodesic.WGS84
        result = geod.Inverse(self.current_latitude, self.current_longitude,
                              self.desired_latitude, self.desired_longitude)
        distance = result['s12']  # Distance in meters
        azimuth = result['azi1']  # Initial bearing in degrees

        # Convert distance and azimuth to x, y in map frame
        x = distance * math.cos(math.radians(azimuth))
        y = distance * math.sin(math.radians(azimuth))

        rospy.loginfo(f"Converted goal position in map frame: (x: {x}, y: {y})")

        # Transform to map frame using tf
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            map_x = trans[0] + x
            map_y = trans[1] + y
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")
            return

        # Publish the goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"  # Map frame

        goal_msg.pose.position.x = map_x
        goal_msg.pose.position.y = map_y
        goal_msg.pose.position.z = 0

        # Use current heading for orientation
        quaternion = quaternion_from_euler(0, 0, self.current_heading)
        goal_msg.pose.orientation.x = quaternion[0]
        goal_msg.pose.orientation.y = quaternion[1]
        goal_msg.pose.orientation.z = quaternion[2]
        goal_msg.pose.orientation.w = quaternion[3]

        self.goal_publisher.publish(goal_msg)
        rospy.loginfo("Published goal to /move_base_simple/goal")

if __name__ == '__main__':
    try:
        GPSGoalConverter()
    except rospy.ROSInterruptException:
        pass
