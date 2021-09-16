#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Float64
import numpy as np
import math

class Twist2duty():
    def __init__(self):
        # FrontRight...0, FrontLeft...1, RearRight...2, RearLeft...3
        rospy.init_node("twist2duty", anonymous= True)
        self.rpm = [0, 0, 0, 0]
        self.duty = [0, 0, 0, 0]
        self.pwm = [0, 0, 0, 0]
        self.received_twist = None
        self.min_duty = 35
        self.wheel_radius = 0.050

        self.ww=0.13  #width/2 [m]
        self.lw=0.115  #length/2 [m]
        self.alpha_deg = 45 #degree of mecanum roller
        self.alpha_rad = math.radians(self.alpha_deg)

        self.a = np.zeros((4,3)) #determinant
        self.a[0][0] = 1/(self.wheel_radius*math.tan(self.alpha_rad)) 
        self.a[0][1] = 1/self.wheel_radius
        self.a[0][2] = (-1*(self.lw+self.ww)/(self.wheel_radius))
        self.a[1][0] = 1/(self.wheel_radius*math.tan(self.alpha_rad))
        self.a[1][1] = -1/self.wheel_radius
        self.a[1][2] = (self.lw+self.ww)/(self.wheel_radius)
        self.a[2][0] = 1/(self.wheel_radius*math.tan(self.alpha_rad))
        self.a[2][1] = -1/self.wheel_radius
        self.a[2][2] = (-1*(self.lw+self.ww)/(self.wheel_radius))
        self.a[3][0] = 1/(self.wheel_radius*math.tan(self.alpha_rad))
        self.a[3][1] = 1/self.wheel_radius
        self.a[3][2] = (self.lw+self.ww)/(self.wheel_radius)

        rospy.Subscriber('/onga_velocity_controller/cmd_vel', Twist, self.callback)
        self.pub_fr_rpm = rospy.Publisher('/fr/rpm', Int64, queue_size=10)
        self.pub_fr_duty = rospy.Publisher('/fr/duty', Int64, queue_size=10)
        self.pub_fr_pwm = rospy.Publisher('/fr/pwm', Int64, queue_size=10)
        self.pub_fl_rpm = rospy.Publisher('/fl/rpm', Int64, queue_size=10)
        self.pub_fl_duty = rospy.Publisher('/fl/duty', Int64, queue_size=10)
        self.pub_fl_pwm = rospy.Publisher('/fl/pwm', Int64, queue_size=10)
        self.pub_rr_rpm = rospy.Publisher('/rr/rpm', Int64, queue_size=10)
        self.pub_rr_duty = rospy.Publisher('/rr/duty', Int64, queue_size=10)
        self.pub_rr_pwm = rospy.Publisher('/rr/pwm', Int64, queue_size=10)
        self.pub_rl_rpm = rospy.Publisher('/rl/rpm', Int64, queue_size=10)
        self.pub_rl_duty = rospy.Publisher('/rl/duty', Int64, queue_size=10)
        self.pub_rl_pwm = rospy.Publisher('/rl/pwm', Int64, queue_size=10)


    def callback(self, message):
        rospy.loginfo("subscribed cmd_vel")
        self.received_twist = message #input data
        self.twist2rpm()
        self.rpm2duty()
        self.duty2pwm()
        self.publish()


    def twist2rpm(self):
        gear_rate = 1.0
        v_x = self.received_twist.linear.x#(m/s)
        v_y = self.received_twist.linear.y#(m/s)
        omega = self.received_twist.angular.z#(rad/s)

        v_fr = (self.a[0][0]*v_x + self.a[0][1]*v_y + self.a[0][2]*omega) *self.wheel_radius*math.pi  #[m/s]
        v_fl = (self.a[1][0]*v_x + self.a[1][1]*v_y + self.a[1][2]*omega) *self.wheel_radius*math.pi  #[m/s]
        v_rr = (self.a[3][0]*v_x + self.a[3][1]*v_y + self.a[3][2]*omega) *self.wheel_radius*math.pi  #[m/s]
        v_rl = (self.a[2][0]*v_x + self.a[2][1]*v_y + self.a[2][2]*omega) *self.wheel_radius*math.pi  #[m/s]
        self.rpm[0] = 60 * v_fr * gear_rate
        self.rpm[1] = 60 * v_fl * gear_rate
        self.rpm[2] = 60 * v_rr * gear_rate
        self.rpm[3] = 60 * v_rl * gear_rate


    def rpm2duty(self):
        fr_rpm = self.rpm[0]
        fl_rpm = self.rpm[1]
        rr_rpm = self.rpm[2] 
        rl_rpm = self.rpm[3]
        max_rpm = 231 #100%
        max_duty = 100
        min_rpm =  231*self.min_duty/100
        min_duty = self.min_duty
        rpm_rate = (max_rpm-min_rpm)/(max_duty-min_duty)#1%に対するrpm

        ##FrontRight duty
        if fr_rpm > 0:
            fr_duty = fr_rpm / rpm_rate + min_duty
        elif fr_rpm < 0:
            fr_duty = fr_rpm / rpm_rate - min_duty 
        elif fr_rpm == 0:
            fr_duty = 0
        if fr_duty >= max_duty:
            rospy.loginfo('Speed limit!!')
            fr_duty = max_duty
        elif fr_duty < (-1*max_duty):
            rospy.loginfo('Speed limit!!')
            fr_duty = -1*max_duty
        ##FrontLeft duty
        if fl_rpm > 0:
            fl_duty = fl_rpm / rpm_rate + min_duty
        elif fl_rpm < 0:
            fl_duty = fl_rpm / rpm_rate - min_duty 
        elif fl_rpm == 0:
            fl_duty = 0
        if fl_duty > max_duty:
            rospy.loginfo('Speed limit!!')
            fl_duty = max_duty
        elif fl_duty < (-1*max_duty):
            rospy.loginfo('Speed limit!!')
            fl_duty = -1*max_duty
        ##RearRight duty
        if rr_rpm > 0:
            rr_duty = rr_rpm / rpm_rate + min_duty
        elif rr_rpm < 0:
            rr_duty = rr_rpm / rpm_rate - min_duty 
        elif rr_rpm == 0:
            rr_duty = 0
        if rr_duty >= max_duty:
            rospy.loginfo('Speed limit!!')
            rr_duty = max_duty
        elif rr_duty < (-1*max_duty):
            rospy.loginfo('Speed limit!!')
            rr_duty = -1*max_duty
        ##RearLeft duty
        if rl_rpm > 0:
            rl_duty = rl_rpm / rpm_rate + min_duty
        elif rl_rpm < 0:
            rl_duty = rl_rpm / rpm_rate - min_duty 
        elif rl_rpm == 0:
            rl_duty = 0
        if rl_duty >= max_duty:
            rospy.loginfo('Speed limit!!')
            rl_duty = max_duty
        elif rl_duty < (-1*max_duty):
            rospy.loginfo('Speed limit!!')
            rl_duty = -1*max_duty

        self.duty[0] = fr_duty
        self.duty[1] = fl_duty
        self.duty[2] = rr_duty
        self.duty[3] = rl_duty


    def duty2pwm(self):
        self.pwm[0] = (255*self.duty[0])/100
        self.pwm[1] = (255*self.duty[1])/100
        self.pwm[2] = (255*self.duty[2])/100
        self.pwm[3] = (255*self.duty[3])/100


    def publish(self):
        self.pub_fr_rpm.publish(self.rpm[0])
        self.pub_fr_duty.publish(self.duty[0])
        self.pub_fr_pwm.publish(self.pwm[0])
        self.pub_fl_rpm.publish(self.rpm[1])
        self.pub_fl_duty.publish(self.duty[1])
        self.pub_fl_pwm.publish(self.pwm[1])
        self.pub_rr_rpm.publish(self.rpm[2])
        self.pub_rr_duty.publish(self.duty[2])
        self.pub_rr_pwm.publish(self.pwm[2])
        self.pub_rl_rpm.publish(self.rpm[3])
        self.pub_rl_duty.publish(self.duty[3])
        self.pub_rl_pwm.publish(self.pwm[3])

#pub memo
# rostopic pub /onga_velocity_controller/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

if __name__ == "__main__":
    try:
        twist2duty = Twist2duty()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



