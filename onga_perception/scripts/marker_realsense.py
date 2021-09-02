#!/usr/bin/env python


import numpy as np
import os
import math
import time
import cv2

import struct
# from cv_bridge import CvBridge
# bridge = CvBridge()

import rospy
import ros_numpy
from std_msgs.msg import Header,String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Image
from std_msgs.msg import Bool,Float32MultiArray
from visualization_msgs.msg import Marker,MarkerArray

import glob
from geometry_msgs.msg import Point,Pose,PoseStamped,PoseArray
import ctypes

import open3d as o3d

import matplotlib.pyplot as plt

import cv2.aruco as aruco



class DepthImageHandler(object):
    def __init__(self):
        self.CAMINFO = {'topic': '/camera/color/camera_info', 'msg': CameraInfo}
        self.isCamInfo = False

        self.COLOR = {'topic': '/camera/color/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/camera/aligned_depth_to_color/image_raw', 'msg': Image}
        self.PC = {'topic': '/camera/depth/color/points', 'msg': PointCloud2}

        # self.CAMINFO = {'topic': '/realsense/color/camera_info', 'msg': CameraInfo}
        # self.isCamInfo = False

        # self.COLOR = {'topic': '/realsense/color/image_raw', 'msg': Image}
        # self.DEPTH = {'topic': '/realsense/depth/image_rect_raw', 'msg': Image}
        # self.PC = {'topic': '/camera/depth/color/points', 'msg': PointCloud2}
        
        self.H = 480#720
        self.W = 640#1280
        self.cropSize = 4
        self.header = Header()
        self.fields = [PointField('x', 0, 7, 1), PointField('y', 4, 7, 1), PointField('z', 8, 7, 1), PointField('rgb', 16, 7, 1)]
        self.points = []
        self.pc = PointCloud2()

        self.color_image = np.empty((self.H, self.W ,3), dtype=np.uint8)
        self.depth_image = np.empty((self.H, self.W), dtype=np.uint16)
        self.aligned_image  = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask           = np.empty((self.H, self.W), dtype=np.bool)
        self.mask_image     = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask_depth     = np.empty((self.H, self.W), dtype=np.uint8)
        self.cropPointCloud = o3d.geometry.PointCloud() 

        self.camera_matrix = np.array([[0.0, 0, 0.0], [0, 0.0, 0.0], [0, 0, 1]], dtype=np.float32) 
        self.camera_distortion = np.array([0,0,0,0,0], dtype=np.float32)



    def camInfoCallback(self, msg):
        self.header = msg.header
        self.K = msg.K
        self.width = msg.width  
        self.height = msg.height
        self.ppx = msg.K[2]
        self.ppy = msg.K[5]
        self.fx = msg.K[0]
        self.fy = msg.K[4] 
        self.model = msg.distortion_model
        self.isCamInfo = True

        self.camera_matrix = np.array([[self.fx, 0, self.ppx], [0, self.fy, self.ppy], [0, 0, 1]], dtype=np.float32) 
        self.camera_distortion = np.array([0,0,0,0,0], dtype=np.float32)

        


    def colorCallback(self, msg):
        self.color_image = ros_numpy.numpify(msg)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

    def depthCallback(self, msg):
        self.depth_image = ros_numpy.numpify(msg).astype(np.uint16)
    

        # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    def rotationMatrixToEulerAngles(self,R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def poseCalculate(self,rvec, tvec):
        rotation_flip = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]], dtype=np.int8)
        bufferMatrix = np.array([[0,0,0,1]])

        ## marker position relative to camera frame
        rotation_cameratomarker = np.matrix(cv2.Rodrigues(rvec)[0])
        rotation_markertocamera = rotation_cameratomarker.T

        #-- Get the orientation in terms of euler 321 (Needs to be flipped first)
        # rotationMatrix_marker = rotation_flip*rotation_markertocamera
        # roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(rotationMatrix_marker)

        #-- Get the orientation in terms of euler 321 (Needs to be flipped first)
        rotationMatrix_camera = rotation_flip*rotation_cameratomarker
        roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(rotationMatrix_camera)

        xyz_camera = np.array(tvec, dtype=np.float32)/1000
        rpy_camera = np.array([math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker)], dtype=np.float32)
        marker_pos = np.concatenate((xyz_camera, rpy_camera), axis=0)
        # ht_marker = np.concatenate((rotationMatrix_marker,np.array([xyz_marker]).T ) , axis=1)
        # ht_marker = np.concatenate((ht_marker , bufferMatrix), axis=0)

        ## camera position relative to marker
        # xyz_camera = np.array(-rotation_markertocamera*np.matrix(tvec).T, dtype=np.float32)
        # xyz_camera = xyz_camera.reshape(3,)
        # roll_camera, pitch_camera, yaw_camera = self.rotationMatrixToEulerAngles(rotation_flip*rotation_markertocamera)
        # rpy_camera = np.array([math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera)], dtype=np.float32)
        # camera_pos = np.concatenate((xyz_camera, rpy_camera), axis=0)

        return marker_pos#, camera_pos, ht_marker

    def getMarkerPose(self,img, box, marker_size, camera_matrix, camera_distortion, drawID=True):
        ret = aruco.estimatePoseSingleMarkers(box, marker_size, camera_matrix, camera_distortion)
        #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] # rotation and translation vector

        # aruco.drawDetectedMarkers(img, boxs)
        aruco.drawAxis(img, camera_matrix, camera_distortion, rvec, tvec, 10)
        #marker_pos, camera_pos, ht_marker = self.poseCalculate(rvec, tvec)
        marker_pos = self.poseCalculate(rvec, tvec)

        if drawID:
            marker_pos_text = "Marker position x=%4.4f y=%4.4f z=%4.4f roll=%4.4f pitch=%4.4f yaw=%4.4f"%(marker_pos[0], marker_pos[1], marker_pos[2], marker_pos[3], marker_pos[4], marker_pos[5])
            cv2.putText(img, marker_pos_text, (10, 50), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,255), 2)

        return marker_pos#ht_marker#marker_pos, camera_pos, ht_marker

    # finding all the aruco markers in the image while drawing a bounding box around the markers and return bounding box and id
    def findArucoMarkers(self,img, arucoDict, arucoParam, camera_matrix, camera_distortion, draw=True):
        imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
        boxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam, 
                                                cameraMatrix=camera_matrix, distCoeff=camera_distortion)
        
        if draw:
            aruco.drawDetectedMarkers(img, boxs)

        return [boxs, ids]    

    def boardCastMarkerFromCameraTF(self,markerPose):
        markerMessage = Float32MultiArray()
        markerMessage.data = markerPose.tolist()
        self.markerXYZRPY_pub.publish(markerMessage)
        print(markerMessage)

    

        
    
    def GetMarker(self):

        color_image, depth_image    = self.color_image, np.asanyarray(self.depth_image)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
        #print(color_image.shape)
        depthForCloud = depth_image
        # ================================================================================ CHANGED ============
        # converted to 16bit image
        depth_image = depth_image.astype(np.uint16)

        markerID  = 33
        marker_size  = 150
        # markerID  = 5
        # marker_size  = 50
        arucoDict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        # arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        arucoParam = aruco.DetectorParameters_create()
        arucoFound = self.findArucoMarkers(color_image, arucoDict, arucoParam, self.camera_matrix, self.camera_distortion)

        Camera_to_Marker_Matrix = np.asmatrix(np.identity(4))
        print(type(Camera_to_Marker_Matrix))

        # loop through all the markets and augment each one
        if len(arucoFound[0]) != 0:
            for box, id in zip(arucoFound[0], arucoFound[1]):
                if int(id) == markerID:
                    markerPoseFromCam = self.getMarkerPose(color_image, box, marker_size, self.camera_matrix, self.camera_distortion)
                    self.boardCastMarkerFromCameraTF(markerPoseFromCam)
        
        cv2.imshow("image", color_image)
        cv2.waitKey(1)




    def process(self):
        rospy.init_node('pointcloud_masking', anonymous=True)
        #r = rospy.Rate(60)
        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)
        #rospy.Subscriber(self.PC['topic'],      self.PC['msg'],         self.pcCallback)
        
        #self.depth_scale    = 0.0010000000474974513

        ###publisher
        self.markerXYZRPY_pub = rospy.Publisher("markerXYZRPY", Float32MultiArray, queue_size=50)


        while not rospy.is_shutdown():
                self.GetMarker()
         
if __name__ == '__main__':
    try:
        _depthImageHandler = DepthImageHandler()
        _depthImageHandler.process()

    except rospy.ROSInterruptException:
        pass
