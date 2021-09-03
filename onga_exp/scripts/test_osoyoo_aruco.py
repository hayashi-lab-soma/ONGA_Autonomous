#!/usr/bin/env python                                                            
# -*- coding: utf-8 -*-                                                          

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi
from std_msgs.msg import Bool,Float32MultiArray
from math import radians

class ArucoNavi():
    def __init__(self):
        rospy.init_node('wp_navi', anonymous=True)
        self.way_point = [999, 999, 999]
        self.listner = tf.TransformListener()
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.loginfo("The robot was terminated.")                               
        self.ac.cancel_goal()

    def process(self):
        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up.")
        rospy.loginfo("The server comes up.")

        target = rospy.wait_for_message("markerXYZRPY", Float32MultiArray)
        rot_rad_list = [radians(float(angle)) for angle in target.data[3:6]]
        quat = tf.transformations.quaternion_from_euler(rot_rad_list[0], rot_rad_list[1], rot_rad_list[2])
        
        aruco_pose = PoseStamped()
        aruco_pose.header.frame_id = "d400_color_optical_frame"
        aruco_pose.pose.position.x = target.data[0]
        aruco_pose.pose.position.y = target.data[1]
        aruco_pose.pose.position.z = target.data[2]
        aruco_pose.pose.orientation.x = quat[0]
        aruco_pose.pose.orientation.y = quat[1]
        aruco_pose.pose.orientation.z = quat[2]
        aruco_pose.pose.orientation.w = quat[3]

        aruco_transformed = self.listner.transformPose("t265_odom_frame", aruco_pose)
        self.way_point[0] = aruco_transformed.pose.position.x
        self.way_point[1] = aruco_transformed.pose.position.y
        self.way_point[2] = 0.0


        self.goal.target_pose.header.frame_id = 't265_odom_frame'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = self.way_point[0]
        self.goal.target_pose.pose.position.y = self.way_point[1]

        if self.way_point[0] != 999:
            q = tf.transformations.quaternion_from_euler(0, 0, self.way_point[2])
            self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            rospy.loginfo("Sending goal")
            self.ac.send_goal(self.goal)

            rospy.loginfo(aruco_pose)
            rospy.loginfo(aruco_transformed)

        succeeded = self.ac.wait_for_result(rospy.Duration(30))
        state = self.ac.get_state()
        if succeeded:
            rospy.loginfo("Succeeded")
        else:
            rospy.loginfo("Failed")

if __name__ == '__main__':
    try:
        wp_navi = ArucoNavi()
        wp_navi.process()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")