#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

class Twist2int64():
    def __init__(self):
        self.rpm_left = Int64()
        self.rpm_right = Int64()
        self.duty_left = Int64()
        self.duty_right = Int64()
        self.received_twist = None
        rospy.init_node("twist2int", anonymous= True)
        rospy.Subscriber('/onga_velocity_controller/cmd_vel', Twist, self.callback)
        self.pub_rpm_right = rospy.Publisher('/right_motor/cmd_rpm', Int64, queue_size=10)#name, topic_type, size
        self.pub_rpm_left = rospy.Publisher('/left_motor/cmd_rpm', Int64, queue_size=10)#name, topic_type, size
        self.pub_duty_right = rospy.Publisher('/right_motor/cmd_duty', Int64, queue_size=10)
        self.pub_duty_left = rospy.Publisher('/left_motor/cmd_duty', Int64, queue_size=10)

        

    def callback(self, message):
        rospy.loginfo("sub")
        self.received_twist = message #input data
        self.rpm_right, self.rpm_left = self.twist2rpm(self.received_twist)
        self.duty_right, self.duty_left = self.rpm2duty(self.rpm_right, self.rpm_left)

        self.pub_rpm_right.publish(self.rpm_right)
        self.pub_rpm_left.publish(self.rpm_left)
        self.pub_duty_right.publish(self.duty_right)
        self.pub_duty_left.publish(self.duty_left)

    def twist2rpm(self, data):#convert to speed
        # rospy.loginfo("input twist(linear.x, angular.z) : ", data.linear.x, data.angular.z)
        #(m/s, rad/s)
        wheeles_size = 0.075#wheel size
        axle_length = 0.35#axle_size(2d)

        v = data.linear.x#(m/s)
        omega = data.angular.z#(rad/s)

        v_r = (omega*axle_length + 2*v)/2
        v_l = (omega*axle_length - 2*v)/(-2)

        v_r = v_r/(wheeles_size * 2 * 3.14) #wheel_speed(1/s)
        v_l = v_l/(wheeles_size * 2 * 3.14) #wheel_speed(1/s)
        r_rpm = 60 * v_r * 19 #gear rate
        l_rpm = 60 * v_l * 19 #gear rate
        # rospy.loginfo("output rpm(right, left) :", r_rpm, l_rpm)
        return r_rpm, l_rpm

    def rpm2duty(self, r_rpm, l_rpm):
        max_rpm = 6000 #規定値90%
        max_duty = 90
        min_rpm = 0 #規定値10%
        min_duty = 10
        rpm_rate = (max_rpm-min_rpm)/(max_duty-min_duty)#1%に対するrpm
        
        ##right duty
        if r_rpm >= 0:
            r_duty = r_rpm / rpm_rate + min_duty
        else:
            r_duty = r_rpm / rpm_rate - min_duty  
        #速度制約
        if r_duty > 90:
            rospy.loginfo('Speed limit!!')
            r_duty = 90
        elif r_duty < -90:
            rospy.loginfo('Speed limit!!')
            r_duty = -90

        ##left duty
        if l_rpm >= 0:
            l_duty = l_rpm / rpm_rate + min_duty
        else:
            l_duty = l_rpm / rpm_rate - min_duty  
        #速度制約
        if l_duty > 90:
            rospy.loginfo('Speed limit!!')
            l_duty = 90
        elif l_duty < -90:
            rospy.loginfo('Speed limit!!')
            l_duty = -90

        # rospy.loginfo("output duty(right, left) :", r_duty, l_duty)
        return r_duty, l_duty


#pub memo
# rostopic pub /onga_velocity_controller/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

if __name__ == "__main__":
    try:
        twist2int64 = Twist2int64()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



