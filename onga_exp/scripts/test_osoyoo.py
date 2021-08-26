#!/usr/bin/env python                                                            
# -*- coding: utf-8 -*-                                                          

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi

class WpNavi():
    def __init__(self):
        self.way_point = [[1.0, 0.0, 0], [999, 999, 999]]
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

        i = 0
        while not rospy.is_shutdown():
            self.goal.target_pose.header.frame_id = 'base_footprint'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = self.way_point[i][0]
            self.goal.target_pose.pose.position.y = self.way_point[i][1]

            if self.way_point[i][0] == 999:
                break
            q = tf.transformations.quaternion_from_euler(0, 0, self.way_point[i][2])
            self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            rospy.loginfo("Sending goal: No" + str(i+1))
            self.ac.send_goal(self.goal)

            succeeded = self.ac.wait_for_result(rospy.Duration(30))
            state = self.ac.get_state()
            if succeeded:
                rospy.loginfo("Succeeded: No."+str(i+1)+"("+str(state)+")")
            else:
                rospy.loginfo("Failed: No."+str(i+1)+"("+str(state)+")")

            i = i + 1

if __name__ == '__main__':
    try:
        rospy.init_node('wp_navi', anonymous=True)
        wp_navi = WpNavi()
        wp_navi.process()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")