#!/usr/bin/env python
from math import trunc
import rospy
import rospkg
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Float64
import serial

class SerialTwist():
    def __init__(self):
        rospy.init_node("serialtwist", anonymous=True)
        rospy.Subscriber("/onga_velocity_controller/cmd_vel", Twist, self.callback)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)

    def callback(self, message):
        rospy.loginfo("subscribe cmd_vel")
        v_x = str(message.linear.x)
        v_y = str(message.linear.y)
        omega = str(message.angular.z)
        print(v_x, v_y, omega)
        self.ser.write(v_x+','+v_y+','+omega+'\n\n')


if __name__ == "__main__":
    try:
        serialtwist = SerialTwist()
        rospy.spin()

    except rospy.ROSInitException:
        pass

