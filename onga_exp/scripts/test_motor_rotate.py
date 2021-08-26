import os
import time
import rospy
from threading import Thread
from geometry_msgs.msg import Quaternion, Twist
from math import pi

cond = True

def cond_switch():
    global cond
    time.sleep(1)
    cond = False

if __name__ == '__main__':
    try:
        rospy.init_node('wp_navi', anonymous=True)
        thread = Thread(target=cond_switch)
        thread.start()
        while cond:
            

        


        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")
