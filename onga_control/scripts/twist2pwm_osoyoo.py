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
        self.pub_rpm_right = rospy.Publisher('/right_motor/rpm', Int64, queue_size=10)
        self.pub_duty_right = rospy.Publisher('/right_motor/duty', Int64, queue_size=10)
        self.pub_pwm_left = rospy.Publisher('/left_motor/pwm', Int64, queue_size=10)
        self.pub_rpm_left = rospy.Publisher('/left_motor/rpm', Int64, queue_size=10)
        self.pub_duty_left = rospy.Publisher('/left_motor/duty', Float64, queue_size=10)
        self.pub_pwm_right = rospy.Publisher('/right_motor/pwm', Float64, queue_size=10)

    def callback(self, message):
        rospy.loginfo("subscribed cmd_vel")

        self.received_twist = message #input data
        self.twist2rpm(self.received_twist)
        self.rpm2duty(self.rpm_right, self.rpm_left)
        self.duty2pwm(self.duty_right, self.duty_left)

        self.publish()

    def twist2rpm(self, data):
        wheeles_size = 0.066#wheel diameter
        axle_length = 0.1#axle_size(2d)
        gear_rate = 1.0

        v = data.linear.x#(m/s)
        omega = data.angular.z#(rad/s)
        v_r = (omega*axle_length + 2*v)/2
        v_l = (omega*axle_length - 2*v)/(-2)

        v_r = v_r/(wheeles_size * 3.14) #wheel_speed(1/s)
        v_l = v_l/(wheeles_size * 3.14) #wheel_speed(1/s)
        r_rpm = 60 * v_r * gear_rate
        l_rpm = 60 * v_l * gear_rate

        self.rpm_right = r_rpm
        self.rpm_left = l_rpm


    def rpm2duty(self, r_rpm, l_rpm):
        max_rpm = 28.9 #規定値90%
        max_duty = 100
        min_rpm = 0 #規定値10%
        min_duty = 0
        rpm_rate = (max_rpm-min_rpm)/(max_duty-min_duty)#1%に対するrpm
        
        ##right duty
        if r_rpm >= 0:
            r_duty = r_rpm / rpm_rate + min_duty
        else:
            r_duty = r_rpm / rpm_rate - min_duty  
        #速度制約
        if r_duty > max_duty:
            rospy.loginfo('Speed limit!!')
            r_duty = max_duty
        elif r_duty < (-1*max_duty):
            rospy.loginfo('Speed limit!!')
            r_duty = -max_duty

        ##left duty
        if l_rpm >= 0:
            l_duty = l_rpm / rpm_rate + min_duty
        else:
            l_duty = l_rpm / rpm_rate - min_duty  
        #速度制約
        if l_duty > max_duty:
            rospy.loginfo('Speed limit!!')
            l_duty = max_duty
        elif l_duty < (-1*max_duty):
            rospy.loginfo('Speed limit!!')
            l_duty = -max_duty

        self.duty_right = r_duty
        self.duty_left = l_duty

    def duty2pwm(self, r_duty, l_duty):
        self.pwm_left = (255*l_duty)/100
        self.pwm_right = (255*r_duty)/100

    def publish(self):
        self.pub_rpm_right.publish(self.rpm_right)
        self.pub_rpm_left.publish(self.rpm_left)
        self.pub_duty_right.publish(self.duty_right)
        self.pub_duty_left.publish(self.duty_left)
        self.pub_pwm_left.publish(self.pwm_left)
        self.pub_pwm_right.publish(self.pwm_right)


#pub memo
# rostopic pub /onga_velocity_controller/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

if __name__ == "__main__":
    try:
        twist2duty = Twist2duty()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



