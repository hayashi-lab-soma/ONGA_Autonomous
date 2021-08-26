#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist

class Test():
    def __init__(self):
        self.pub = rospy.Publisher('/onga_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.stop = Twist()
        self.stop.linear.x = 0
        self.stop.angular.z = 0

    # go straight
    def pub_linear(self):
        dist = 1.0 # [m]
        speed = 0.5 # [m/s]
        target_time = dist / speed # [s]

        t = Twist()
        t.linear.x = speed
        t.angular.z = 0            
        start_time = time.time()
        end_time = time.time()

        rate = rospy.Rate(30)
        while end_time - start_time <= target_time:
            self.pub.publish(t)
            end_time = time.time()
            rate.sleep()
        self.pub.publish(self.stop)

    #rotate
    def pub_angular(self):
        theta = 180.0 # [deg]
        speed = 120.0 # [deg/s]
        target_time = theta / speed # [s]

        t = Twist()
        t.linear.x = 0
        t.angular.z = speed * 3.1415 / 180.0 # [rad]
        start_time = time.time()
        end_time = time.time()

        rate = rospy.Rate(30)
        while end_time - start_time <= target_time:
            self.pub.publish(t)
            end_time = time.time()
            rate.sleep()
        self.pub.publish(self.stop)


if __name__ == '__main__':
    rospy.init_node('tcmdvel_publisher')
    test = Test()
    test.pub_linear()
    # test.pub_angular()