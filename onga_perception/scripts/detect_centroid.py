#!/usr/bin/env python
import cv2
import numpy as np
import math
from rosgraph.names import anonymous_name
import rospy
import rosparam
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import ros_numpy as rosnp

class DetectCentroid(object):
    def __init__(self):
        rospy.loginfo("initialize DetectionCentroid")
        # self.sub_rgb = message_filters.Subscriber("/camera/color/image_raw",Image)
        # self.sub_depth = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw",Image)
        # self.bridge = CvBridge()

    def RGBCallback(self, rgb_data):
        rospy.loginfo("success sub")
        # color_image = self.bridge.imgmsg_to_cv2(rgb_data)
        # color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        color_image = rosnp.numpify(rgb_data)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

        bgrLower = np.array([30, 80, 150])    
        bgrUpper = np.array([120, 150, 255])  
        mask_image = cv2.inRange(color_image, bgrLower, bgrUpper)
        result = cv2.bitwise_and(color_image, color_image, mask=mask_image)
        
        M = cv2.moments(mask_image)
        if M["m00"] == 0 or M["m00"] == 0 :
            cX = -1
            cY = -1       
        else :
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(color_image, (cX, cY), 5, (0, 0, 0), -1)

        contours = cv2.findContours(mask_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        
        area_thresh = 10000
        contours = list(filter(lambda x: cv2.contourArea(x) > area_thresh, contours))

        
        for cnt in contours:
        
            x, y, width, height = cv2.boundingRect(cnt)
            
            cv2.rectangle(color_image, (x, y), (x + width, y + height), color=(0, 255, 0), thickness=2)

        cv2.imshow("image", color_image)
        cv2.imshow("mask", result)
        # cv2.imwrite("ret.png", color_image)
        cv2.waitKey(1)

        rospy.loginfo("centroid", cX, cY)

    def Process(self):
        rospy.init_node("detect_centroid", anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.RGBCallback)
        rospy.spin()
        # rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['depth_data'], self.DepthCallback)
            
  
if __name__ == '__main__':
    try:
        detect_centroid = DetectCentroid()
        detect_centroid.Process()

    except rospy.ROSInterruptException: pass


