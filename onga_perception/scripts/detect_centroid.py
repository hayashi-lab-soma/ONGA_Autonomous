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
from std_msgs.msg import Header
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point,Pose
# from tf.msg import tfMessage

def send_traj_point_marker(marker_pub, pose):
    marker = Marker()
    # marker.header.frame_id = "/camera_color_optical_frame"
    marker.header.frame_id = "/base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "traj_point" 
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0.3,0)
    marker_pub.publish(marker)

class DetectCentroid(object):
    def __init__(self):
        rospy.loginfo("initialize DetectionCentroid")

        ## For real device
        # self.CAMINFO = {'topic': '/camera/color/camera_info', 'msg': CameraInfo}
        # self.COLOR = {'topic': '/camera/color/image_raw', 'msg': Image}
        # self.DEPTH = {'topic': '/camera/aligned_depth_to_color/image_raw', 'msg': Image}
        
        ## For simulation
        self.CAMINFO = {'topic': '/realsense/color/camera_info', 'msg': CameraInfo}
        self.COLOR = {'topic': '/realsense/color/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/realsense/depth/image_rect_raw', 'msg': Image}
        self.H = 640
        self.W = 480

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

    def posecallback(self,msg):
        pass

    def centroid(self,img):
        M   = cv2.moments(img)
        cX  = int(M["m10"] / M["m00"])
        cY  = int(M["m01"] / M["m00"])
        return cX, cY

    def depthToPoints(self, depth_image, U, V):      
        x = (U - self.K[2])/self.K[0]
        y = (V - self.K[5])/self.K[4]      
        
        z = depth_image[V,U]*self.depth_scale
        x *= z
        y *= z
        point = [x,y,z]
        return point

    def empty(self,a):
        pass

    # create new window with trackbar for HSV Color
    def window(self):
        cv2.namedWindow("Range HSV")
        cv2.resizeWindow("Range HSV", 500, 350)
        cv2.createTrackbar("HUE Min", "Range HSV", 105,180, self.empty)
        cv2.createTrackbar("HUE Max", "Range HSV", 168,180, self.empty)
        cv2.createTrackbar("SAT Min", "Range HSV", 22,255, self.empty)
        cv2.createTrackbar("SAT Max", "Range HSV", 149,255, self.empty)
        cv2.createTrackbar("VALUE Min", "Range HSV", 0,255, self.empty)
        cv2.createTrackbar("VALUE Max", "Range HSV", 50,255, self.empty)

    def get_hsvcentroid(self):
        h_min = cv2.getTrackbarPos("HUE Min", "Range HSV")
        h_max = cv2.getTrackbarPos("HUE Max", "Range HSV")
        s_min = cv2.getTrackbarPos("SAT Min", "Range HSV")
        s_max = cv2.getTrackbarPos("SAT Max", "Range HSV")
        v_min = cv2.getTrackbarPos("VALUE Min", "Range HSV")
        v_max = cv2.getTrackbarPos("VALUE Max", "Range HSV")

        # define range of some color in HSV
        lower_range = np.array([h_min,s_min,v_min])
        upper_range = np.array([h_max, s_max, v_max])
        
        color_image = self.color_image
        mask_image = cv2.inRange(color_image, lower_range, upper_range)
        depth_image = np.asanyarray(self.depth_image)
        result = cv2.bitwise_and(color_image, color_image, mask=mask_image)

        cv2.imshow("RGB", color_image)
        cv2.imshow("MASK", result)
        cv2.waitKey(1)
        
        contours, hierarchy = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            try:
                c = max(contours, key = cv2.contourArea)
                cX, cY = self.centroid(c)
                cv2.circle(color_image, (cX, cY), 5, (0, 0, 0), -1)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(color_image, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)
                depth_point = self.depthToPoints(depth_image=depth_image,U=cX,V=cY)

                ##--- Marker
                new_pose = Pose()
                new_pose.position.y     = -depth_point[0]*1000  
                new_pose.position.z     = abs(depth_point[1]*1000)
                new_pose.position.x     = depth_point[2]*1000
                new_pose.orientation.x  = 0.0
                new_pose.orientation.y  = 0.0
                new_pose.orientation.z  = 0.0
                new_pose.orientation.w  = 1
                ##--- END : Marker

                print(f'x = {new_pose.position.x}, y = {new_pose.position.y}, z = {new_pose.position.z}')
                send_traj_point_marker(marker_pub=self.object_pub, pose=new_pose)
            except:
                pass
                
    def Process(self):
        rospy.init_node("detect_centroid", anonymous=True)
        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)

        self.depth_scale    = 0.0010000000474974513
        self.object_pub     = rospy.Publisher( '/object/centroid', Marker, queue_size=10)
        # rospy.spin()
        while not rospy.is_shutdown():
            self.get_hsvcentroid()
            
  
if __name__ == '__main__':
    try:
        detect_centroid = DetectCentroid()
        detect_centroid.window()
        detect_centroid.Process()

    except rospy.ROSInterruptException:
        pass


