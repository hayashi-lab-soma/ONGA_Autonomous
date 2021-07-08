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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
import ros_numpy as rosnp
import pyrealsense2 as rs
from std_msgs.msg import Header

class DetectCentroid(object):
    def __init__(self):
        rospy.loginfo("initialize DetectionCentroid")

        self.CAMINFO = {'topic': '/camera/color/camera_info', 'msg': CameraInfo}
        self.COLOR = {'topic': '/camera/color/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/camera/aligned_depth_to_color/image_raw', 'msg': Image}
        self.H = 720
        self.W = 1280
        self.header = Header()
        self.fields = [PointField('x', 0, 7, 1), PointField('y', 4, 7, 1), PointField('z', 8, 7, 1), PointField('rgb', 16, 7, 1)]
        self.points = []
        self.color_image = np.empty((self.H, self.W ,3), dtype=np.uint8)
        self.depth_image = np.empty((self.H, self.W), dtype=np.uint16)
        self.aligned_image  = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask           = np.empty((self.H, self.W), dtype=np.bool)
        self.mask_image     = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask_depth     = np.empty((self.H, self.W), dtype=np.uint8)

    def colorCallback(self, msg):
        self.color_image = rosnp.numpify(msg)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

    def depthCallback(self,msg):
        self.depth_image = rosnp.numpify(msg).astype(np.uint16)

    def camInfoCallback(self,msg):
        self.header = msg.header
        self.K = msg.K
        self.width = msg.width  
        self.height = msg.height
        self.ppx = msg.K[2]
        self.ppy = msg.K[5]
        self.fx = msg.K[0]
        self.fy = msg.K[4] 
        self.model = msg.distortion_model

    def centroid(self,img):
        M   = cv2.moments(img)
        if  M["m00"] == 0 or M["m00"] == 0 :
            return
        
        cX  = int(M["m10"] / M["m00"])
        cY  = int(M["m01"] / M["m00"])
        return cX, cY

    def get_hsvcentroid(self):
        bgrLower = np.array([30, 80, 150])    
        bgrUpper = np.array([120, 150, 255])
        color_image = self.color_image
        mask_image = cv2.inRange(color_image, bgrLower, bgrUpper)
        result = cv2.bitwise_and(color_image, color_image, mask=mask_image)
        
        if self.centroid(mask_image) is None :
            cX, cY = -1, -1
        else : 
            cX, cY = self.centroid(mask_image)
            cv2.circle(color_image, (cX, cY), 5, (0, 0, 0), -1)
            print(f'x = {cX}, y = {cY}')

        contours = cv2.findContours(mask_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        
        area_thresh = 10000
        contours = list(filter(lambda x: cv2.contourArea(x) > area_thresh, contours))

        for cnt in contours:
        
            x, y, width, height = cv2.boundingRect(cnt)
            
            cv2.rectangle(color_image, (x, y), (x + width, y + height), color=(0, 255, 0), thickness=2)

        cv2.imshow("image", color_image)
        cv2.imshow("mask", result)
        cv2.waitKey(1)

        # rospy.loginfo("centroid", cX, cY)

    def Process(self):
        rospy.init_node("detect_centroid", anonymous=True)
        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)
        # rospy.spin()
        # rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['depth_data'], self.DepthCallback)
        while not rospy.is_shutdown():
            self.get_hsvcentroid()
            
  
if __name__ == '__main__':
    try:
        detect_centroid = DetectCentroid()
        detect_centroid.Process()

    except rospy.ROSInterruptException:
        pass


