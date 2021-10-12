#!/usr/bin/env python                                                            
# -*- coding: utf-8 -*-                                                          

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi
import argparse

class WpNavi():
    def __init__(self):
        # self.way_point = [[0.5, 0,0.0 * pi], [ 3.0, -3.0, 0.0 * pi], [0.0, 0.0, 0.0 * pi], [999, 999, 999]]
        # self.way_point = [[float(args.x), float(args.y), float(args.angle) * pi], [ 3.0, -3.0, 0.0 * pi], [0.0, 0.0, 0.0 * pi], [999, 999, 999]]
        x = rospy.get_param("x",-0.2)
        y = rospy.get_param("y",0.0)
        angular = rospy.get_param("angular",0)
        self.way_point = [[x, y, angular * pi], [ 3.0, -3.0, 0.0 * pi], [0.0, 0.0, 0.0 * pi], [999, 999, 999]]
        # self.way_point = [[0.3, 0, 0.0 * pi], [ 3.0, -3.0, 0.0 * pi], [0.0, 0.0, 0.0 * pi], [999, 999, 999]]
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

        self.goal.target_pose.header.frame_id = 't265_odom_frame'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = self.way_point[0][0]
        self.goal.target_pose.pose.position.y = self.way_point[0][1]
        print(self.way_point)

        q = tf.transformations.quaternion_from_euler(0, 0, self.way_point[0][2])
        self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        # rospy.loginfo("Sending goal: No" + str(i+1))
        self.ac.send_goal(self.goal)

        succeeded = self.ac.wait_for_result(rospy.Duration(30))
        state = self.ac.get_state()
        if succeeded:
            rospy.loginfo("Succeeded")
            
        else:
            rospy.loginfo("Failed")

if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument('x', help='x_coordinate')
    # parser.add_argument('y', help='y_coordinate')
    # parser.add_argument('angle', help='angle')
    # args = parser.parse_args()
    # print('(x,y)=('+args.x+','+args.y+')')
    try:
        rospy.init_node('wp_navi', anonymous=True)
        wp_navi = WpNavi()
        wp_navi.process()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")