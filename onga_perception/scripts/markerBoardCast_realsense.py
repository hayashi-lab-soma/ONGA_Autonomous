#!/usr/bin/env python  
import roslib
import rospy
from math import radians

from std_msgs.msg import Bool,Float32MultiArray
import tf
import turtlesim.msg

def handle_array_pose(msg):
    br = tf.TransformBroadcaster()
    pos_list,rot_list = msg.data[0:3],msg.data[3:6]
    
    rot_rad_list = [radians(float(angle)) for angle in rot_list]

    br.sendTransform((pos_list[0], pos_list[1],pos_list[2]),
                     tf.transformations.quaternion_from_euler(rot_rad_list[0], rot_rad_list[1], rot_rad_list[2]),
                     rospy.Time.now(),
                     "world",
                     "camera_color_optical_frame")
    print(tf.transformations.quaternion_from_euler(rot_rad_list[0], rot_rad_list[1], rot_rad_list[2]))

if __name__ == '__main__':
    rospy.init_node('marker_tf_broadcaster')
    rospy.Subscriber('markerXYZRPY',
                     Float32MultiArray,
                     handle_array_pose)
    rospy.spin()