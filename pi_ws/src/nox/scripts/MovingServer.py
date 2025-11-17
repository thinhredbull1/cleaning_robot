#!/usr/bin/env python

import rospy
import actionlib
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
from tf import TransformListener
import tf
import math
import time

# Global variables
waypoints = []
start_moving = False

# Change Pose to the correct frame
def changePose(waypoint, target_frame):
    if waypoint.header.frame_id == target_frame:
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()

class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id', 'map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.0)

    def execute(self, userdata):
        global waypoints
        for waypoint in waypoints:
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                          (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            self.client.send_goal(goal)
            if not self.distance_tolerance > 0.0:
                self.client.wait_for_result()
                rospy.loginfo("Waiting for %f sec..." % self.duration)
                time.sleep(self.duration)
            else:
                distance = 10
                while distance > self.distance_tolerance:
                    now = rospy.Time.now()
                    self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    trans, rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
                    distance = math.sqrt(pow(waypoint.pose.pose.position.x - trans[0], 2) +
                                         pow(waypoint.pose.pose.position.y - trans[1], 2))
        return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id', 'map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        self.waypoints_topic = rospy.get_param('~waypoints_topic', '/waypoints')
        self.start_moving_topic = rospy.get_param('~start_moving_topic', '/start_moving')
        self.posearray_topic = rospy.get_param('~posearray_topic', '/waypoints_viz')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)
        self.frame_id = rospy.get_param('~goal_frame_id', 'map')

        # Subscribe to waypoints and start moving topics
        rospy.Subscriber(self.waypoints_topic, PoseArray, self.waypoints_callback)
        rospy.Subscriber(self.start_moving_topic, String, self.start_moving_callback)

    def waypoints_callback(self, msg):
        global waypoints
        waypoints = []
        for pose in msg.poses:
            pose_cov = PoseWithCovarianceStamped()
            pose_cov.header.frame_id = msg.header.frame_id
            pose_cov.pose.pose = pose
            waypoints.append(changePose(pose_cov, self.frame_id))
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
        rospy.loginfo("Received %d waypoints", len(waypoints))

    def start_moving_callback(self, msg):
        global start_moving
        if msg.data == "MOVING":
            start_moving = True
            rospy.loginfo("Received MOVING command")

    def execute(self, userdata):
        global waypoints, start_moving
        waypoints = []
        start_moving = False
        rospy.loginfo("Waiting for waypoints on topic %s and MOVING message on %s" %
                      (self.waypoints_topic, self.start_moving_topic))

        # Wait until both waypoints and MOVING message are received
        while not rospy.is_shutdown():
            if waypoints and start_moving:
                return 'success'
            rospy.sleep(0.1)

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        return 'success'

def main():
    rospy.init_node('follow_waypoints')
    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                         transitions={'success': 'FOLLOW_PATH'},
                         remapping={'waypoints': 'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                         transitions={'success': 'PATH_COMPLETE'},
                         remapping={'waypoints': 'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                         transitions={'success': 'GET_PATH'})
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()