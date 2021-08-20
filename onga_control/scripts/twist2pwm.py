#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Float64

class Twist2duty():
    def __init__(self):
        rospy.init_node("twist2duty", anonymous= True)
        self.rpm_left = Int64()
        self.rpm_right = Int64()
        self.duty_left = Int64()
        self.duty_right = Int64()
        self.pwm_left = Int64()
        self.pwm_right = Int64()
        self.received_twist = None
        rospy.Subscriber('/onga_velocity_controller/cmd_vel', Twist, self.callback)
        self.pub_rpm_right = rospy.Publisher('/right_motor/cmd_rpm', Int64, queue_size=10)
        self.pub_duty_right = rospy.Publisher('/right_motor/cmd_duty', Int64, queue_size=10)
        self.pub_pwm_left = rospy.Publisher('/left_motor/pwm', Int64, queue_size=10)
        self.pub_rpm_left = rospy.Publisher('/left_motor/cmd_rpm', Int64, queue_size=10)
        self.pub_duty_left = rospy.Publisher('/left_motor/cmd_duty', Float64, queue_size=10)
        self.pub_pwm_right = rospy.Publisher('/right_motor/pwm', Float64, queue_size=10)

    def callback(self, message):
        rospy.loginfo("subscribed cmd_vel")
        self.received_twist = message #input data
        self.rpm_right, self.rpm_left = self.twist2rpm(self.received_twist)
        self.duty_right, self.duty_left = self.rpm2duty(self.rpm_right, self.rpm_left)
        rospy.loginfo(self.duty_right)
        self.pwm_right, self.pwm_left = self.duty2pwm(self.duty_right, self.duty_left)
        self.publish()

    def publish(self):
        self.pub_rpm_right.publish(self.rpm_right)
        self.pub_rpm_left.publish(self.rpm_left)
        self.pub_duty_right.publish(self.duty_right)
        self.pub_duty_left.publish(self.duty_left)
        self.pub_pwm_left.publish(self.pwm_left)
        self.pub_pwm_right.publish(self.pwm_right)

    def duty2pwm(self, r_duty, l_duty):
        # l_pwm = int(255*(l_duty/100))
        # r_pwm = int(255*(r_duty/100))
        l_pwm = (255*l_duty)/100
        r_pwm = (255*r_duty)/100
        # rospy.loginfo("output pwm(right, left) :", l_pwm, r_pwm)
        return r_pwm, l_pwm

    def rpm2duty(self, r_rpm, l_rpm):
        max_rpm = 6500 #規定値90%
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
        if r_duty > max_duty:
            rospy.loginfo('Speed limit!!')
            r_duty = 90
        elif r_duty < (-1*max_duty):
            rospy.loginfo('Speed limit!!')
            r_duty = -90

        ##left duty
        if l_rpm >= 0:
            l_duty = l_rpm / rpm_rate + min_duty
        else:
            l_duty = l_rpm / rpm_rate - min_duty  
        #速度制約
        if l_duty > max_duty:
            rospy.loginfo('Speed limit!!')
            l_duty = 90
        elif l_duty < (-1*max_duty):
            rospy.loginfo('Speed limit!!')
            l_duty = -90

        # rospy.loginfo("output duty(right, left) :", r_duty, l_duty)
        return r_duty, l_duty

    def twist2rpm(self, data):
        # rospy.loginfo("input twist(linear.x, angular.z) : ", data.linear.x, data.angular.z)
        #(m/s, rad/s)
        wheeles_size = 0.1#wheel size
        axle_length = 0.365#axle_size(2d)
        gear_rate = 721

        v = data.linear.x#(m/s)
        omega = data.angular.z#(rad/s)

        v_r = (omega*axle_length + 2*v)/2
        v_l = (omega*axle_length - 2*v)/(-2)

        v_r = v_r/(wheeles_size * 2 * 3.14) #wheel_speed(1/s)
        v_l = v_l/(wheeles_size * 2 * 3.14) #wheel_speed(1/s)
        r_rpm = 60 * v_r * gear_rate
        l_rpm = 60 * v_l * gear_rate
        # rospy.loginfo("output rpm(right, left) :", r_rpm, l_rpm)
        return r_rpm, l_rpm


#pub memo
# rostopic pub /onga_velocity_controller/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

if __name__ == "__main__":
    try:
        twist2duty = Twist2duty()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



