#!/usr/bin/env python                                                            
# -*- coding: utf-8 -*-                                                          

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import Float32

class WpNavi():
    def __init__(self):
        self.GOAL = {'topic': '/object/centroid', 'msg': Marker}
        self.ARG = {'topic': '/argument', 'msg': Float32}
        self.way_point = [[2.0, 3.0,0.0 * pi], [ 3.0, -3.0, 0.0 * pi], [0.0, 0.0, 0.0 * pi], [999, 999, 999]]
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()

        self.goal_x = 3 #TEMPORAL METHOD
        self.goal_y = 0
        self.goal_z = 0

        self.arg = 180

        rospy.on_shutdown(self.shutdown)

    def centroid_callback(self,msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = msg.pose.position.z

    def arg_callback(self,msg):
        self.arg = msg.data
        # print(self.arg)

    def shutdown(self):
        rospy.loginfo("The robot was terminated.")                               
        self.ac.cancel_goal()

    def process(self):
        rospy.Subscriber(self.GOAL['topic'], self.GOAL['msg'], self.centroid_callback)
        rospy.Subscriber(self.ARG['topic'], self.ARG['msg'], self.arg_callback)

        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up.")
        rospy.loginfo("The server comes up.")

        i = 2
        while not rospy.is_shutdown():
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = self.goal_z
            self.goal.target_pose.pose.position.y = -self.goal_x

            if self.goal.target_pose.pose.position.x != 3:  #TEMPORAL METHOD

                if self.way_point[i][0] == 999:
                    break
                print(self.arg)
                q = tf.transformations.quaternion_from_euler(0, 0, self.arg)#self.way_point[i][2])
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