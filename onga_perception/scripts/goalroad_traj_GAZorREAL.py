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
from std_msgs.msg import Header, Float32
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point,Pose
from sensor_msgs import point_cloud2
import struct
import open3d as o3d
# from tf.msg import tfMessage

class DetectCentroid(object):
    def __init__(self, use_gazebo=bool):
        rospy.loginfo("initialize DetectionCentroid")

        self.use_gazebo = use_gazebo

        if self.use_gazebo:
            "roslaunch onga_gazebo product_sim.launch"

            rospy.loginfo("using gazebo")
            ## For simulation
            self.CAMINFO = {'topic': '/realsense/color/camera_info', 'msg': CameraInfo}
            self.COLOR = {'topic': '/realsense/color/image_raw', 'msg': Image}
            self.DEPTH = {'topic': '/realsense/depth/image_rect_raw', 'msg': Image}
            self.frame_id = "/camera_realsense_gazebo"
            self.H = 480
            self.W = 640
            self.scale = 1000
            self.marker_scale = 0.2
        else:
            "roslaunch realsense2_camera rs_rgbd.launch"

            rospy.loginfo("using realsense D400 series")
            ## For real device
            self.CAMINFO = {'topic': '/camera/color/camera_info', 'msg': CameraInfo}
            self.COLOR = {'topic': '/camera/color/image_raw', 'msg': Image}
            self.DEPTH = {'topic': '/camera/aligned_depth_to_color/image_raw', 'msg': Image}
            self.frame_id = "/camera_color_optical_frame"
            self.H = 720
            self.W = 1280
            self.scale = 1
            self.marker_scale = 0.02

        self.header = Header()
        self.fields = [PointField('x', 0, 7, 1), PointField('y', 4, 7, 1), PointField('z', 8, 7, 1), PointField('rgb', 16, 7, 1)]
        self.points = []
        self.pc = PointCloud2()
        self.color_image = np.empty((self.H, self.W ,3), dtype=np.uint8)
        self.depth_image = np.empty((self.H, self.W), dtype=np.uint16)
        self.aligned_image  = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask           = np.empty((self.H, self.W), dtype=np.bool)
        self.mask_image     = np.empty((self.H, self.W), dtype=np.uint8)
        self.max_mask   = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask_depth     = np.empty((self.H, self.W), dtype=np.uint8)
        self.rgb_red = 0xff0000

    def colorCallback(self, msg):
        self.color_image = rosnp.numpify(msg)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

    def depthCallback(self,msg):
        self.depth_image = rosnp.numpify(msg)#.astype(np.uint16)

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

    def send_traj_point_marker(self, marker_pub, pose):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        # marker.header.frame_id = "/camera_realsense_gazebo"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "traj_point" 
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 0.04
        marker.color.g = 0.68
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.3,0)
        marker_pub.publish(marker)

    def send_traj_line_marker(self, marker_pub, pose, points):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "traj_point" 
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose = pose
        marker.points = points
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 1.1
        marker.color.g = 1.1
        marker.color.b = 1.1
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.3,0)
        marker_pub.publish(marker)

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

    def depthToPointcloud(self, depth_image):
        [height, width] = depth_image.shape
        nx = np.linspace(0, width-1, width)
        ny = np.linspace(0, height-1, height)
        u, v = np.meshgrid(nx, ny)
        x = (u.flatten() - self.K[2])/self.K[0]
        y = (v.flatten() - self.K[5])/self.K[4]

        z = depth_image.flatten() * self.depth_scale #/ 1000.0
        x = np.multiply(x,z)
        y = np.multiply(y,z)

        x = x[np.nonzero(z)]
        y = y[np.nonzero(z)]
        z = z[np.nonzero(z)]
        
        points = np.stack((x, y, z), axis = -1)

        return points

    def publishPointcloud(self, pc_pub, points, color):
        points_color = self.addColorToPoints(points, color)
        header = self.header
        header.frame_id = self.frame_id
        header.stamp = rospy.Time.now()
        pc = point_cloud2.create_cloud(header, self.fields, points_color)
        pc_pub.publish(pc)

    def addColorToPoints(self, points, color):
        float_rgb = self.rgbToFloat(color)
        rgb = np.full((len(points),), float_rgb)
        points_color = np.c_[points, rgb]
        return points_color

    def rgbToFloat(self, rgb):
        return struct.unpack('f', struct.pack('i', rgb))[0]

    def empty(self,a):
        pass

    def publish_notmalVector(self, arg_pub, vec_x, vec_y):
        arg = math.degrees(math.atan(vec_y/vec_x))
        arg_pub.publish(arg)

    # create new window with trackbar for HSV Color
    def window(self):
        cv2.namedWindow("Range HSV")
        cv2.resizeWindow("Range HSV", 500, 350)

        if self.use_gazebo:
            ######################gray_object######################
            # cv2.createTrackbar("HUE Min", "Range HSV", 2,180, self.empty)#105,180 orange corn
            # cv2.createTrackbar("HUE Max", "Range HSV", 180,180, self.empty)#168,180
            # cv2.createTrackbar("SAT Min", "Range HSV", 0,255, self.empty)#22,255
            # cv2.createTrackbar("SAT Max", "Range HSV", 235,255, self.empty)#149,255
            # cv2.createTrackbar("VALUE Min", "Range HSV",13,255, self.empty)#0,255
            # cv2.createTrackbar("VALUE Max", "Range HSV", 147,255, self.empty)#50,255
            ######################gray_object######################

            # ######################yellow_landmark######################
            cv2.createTrackbar("HUE Min", "Range HSV", 22,180, self.empty)#105,180 orange corn
            cv2.createTrackbar("HUE Max", "Range HSV", 180,180, self.empty)#168,180
            cv2.createTrackbar("SAT Min", "Range HSV", 24,255, self.empty)#22,255
            cv2.createTrackbar("SAT Max", "Range HSV", 237,255, self.empty)#149,255
            cv2.createTrackbar("VALUE Min", "Range HSV",0,255, self.empty)#0,255
            cv2.createTrackbar("VALUE Max", "Range HSV", 19,255, self.empty)#50,255
            # ######################yellow_landmark######################

        else:
            # ######################green_jetson_case######################
            # cv2.createTrackbar("HUE Min", "Range HSV", 64,180, self.empty)#105,180 orange corn
            # cv2.createTrackbar("HUE Max", "Range HSV", 180,180, self.empty)#168,180
            # cv2.createTrackbar("SAT Min", "Range HSV", 182,255, self.empty)#22,255
            # cv2.createTrackbar("SAT Max", "Range HSV", 255,255, self.empty)#149,255
            # cv2.createTrackbar("VALUE Min", "Range HSV",96,255, self.empty)#0,255
            # cv2.createTrackbar("VALUE Max", "Range HSV", 186,255, self.empty)#50,255
            # ######################green_jetson_case######################

            #####################under_graduate_cirtification######################
            cv2.createTrackbar("HUE Min", "Range HSV", 15,180, self.empty)#105,180 orange corn
            cv2.createTrackbar("HUE Max", "Range HSV", 92,180, self.empty)#168,180
            cv2.createTrackbar("SAT Min", "Range HSV", 71,255, self.empty)#22,255
            cv2.createTrackbar("SAT Max", "Range HSV", 156,255, self.empty)#149,255
            cv2.createTrackbar("VALUE Min", "Range HSV",0,255, self.empty)#0,255
            cv2.createTrackbar("VALUE Max", "Range HSV", 30,255, self.empty)#50,255
            #####################under_graduate_cirtification######################

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
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (6,6))
        mask_image = cv2.erode(mask_image, kernel, iterations = 1)
        mask_image = cv2.dilate(mask_image, kernel, iterations = 1)
        depth_image = self.depth_image#np.asanyarray(self.depth_image)
        result = cv2.bitwise_and(color_image, color_image, mask=mask_image)

        cv2.imshow("RGB", color_image)
        cv2.imshow("MASK IN RGB", result)
        cv2.imshow("MASK", mask_image)
        cv2.waitKey(1)
        
        contours, hierarchy = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # max_mask = self.max_mask
        max_mask = np.zeros_like(mask_image)

        if len(contours) != 0:
            try:
                c = max(contours, key = cv2.contourArea)
                cX, cY = self.centroid(c)
                cv2.circle(color_image, (cX, cY), 5, (0, 0, 0), -1)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(color_image, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)
                depth_point = self.depthToPoints(depth_image=depth_image,U=cX,V=cY)
                cv2.drawContours(max_mask, [max(contours, key = cv2.contourArea)], -1, (255, 255, 255), -1)
                cv2.imshow('MAX MASK', max_mask)
                cv2.waitKey(1)

                ##--- Marker
                new_pose = Pose()
                new_pose.position.x     = depth_point[0]*self.scale
                new_pose.position.y     = depth_point[1]*self.scale
                new_pose.position.z     = depth_point[2]*self.scale

                new_pose.orientation.x  = 0.0
                new_pose.orientation.y  = 0.0
                new_pose.orientation.z  = 0.0
                new_pose.orientation.w  = 1
                ##--- END : Marker

                if depth_point[0]!=0 and depth_point[1]!=0 and depth_point[2]!=0:
                    print(f'x = {new_pose.position.x}, y = {new_pose.position.y}, z = {new_pose.position.z}')
                    self.send_traj_point_marker(marker_pub=self.object_pub, pose=new_pose)

                    masked_depth = np.zeros((self.H, self.W), dtype=np.uint16)
                    mask_int = mask_image.astype(np.uint16)*255
                    # mask_int = max_mask.astype(np.uint16)*255
                    ret, thresh = cv2.threshold(mask_int, 127, 255, 0)
                    masked_depth[np.nonzero(thresh)] = depth_image[np.nonzero(thresh)]
                    points = self.depthToPointcloud(masked_depth) * self.scale
                    self.publishPointcloud(self.pcmasking_pub, points, self.rgb_red)

                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(points)
                    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
                    # pcd = o3d.io.read_point_cloud(points)
                    plane_model, inliers = downpcd.segment_plane(distance_threshold=0.5,
                                                            ransac_n=5,#gazebo : 50
                                                            num_iterations=1000)
                    [a, b, c, d] = plane_model
                    # o3d.visualization.draw_geometries([downpcd])
                    self.publish_notmalVector(self.arg_pub,a,c)

                    print(f"Plane equation: {a:.5f}x + {b:.5f}y + {c:.5f}z + {d:.5f} = 0")

                    z1 = depth_point[2]*self.scale
                    z2 = 0
                    x1 = depth_point[0]*self.scale
                    # x2 = -(b*(z1-z2)/a+x1)
                    # x2 = (x1-a)/(c-z1)+a
                    x2 = x1 - a*z1/c
                    ##--- LINE Marker
                    line_pose = Pose()
                    line_pose.position.y     = 0.0
                    line_pose.position.z     = 0.0
                    line_pose.position.x     = 0.0

                    line_pose.orientation.x  = 0.0
                    line_pose.orientation.y  = 0.0
                    line_pose.orientation.z  = 0.0
                    line_pose.orientation.w  = 1

                    # marker line points
                    line = []
                    # first point
                    first_line_point = Point()
                    first_line_point.y = 0#0.755
                    first_line_point.z = z1
                    first_line_point.x = x1
                    line.append(first_line_point)
                    # second point
                    second_line_point = Point()
                    second_line_point.y = 0#0.755
                    second_line_point.z = z2
                    second_line_point.x = x2
                    line.append(second_line_point)
                    ##--- END : LINE Marker
                    self.send_traj_line_marker(marker_pub=self.line_pub, pose=line_pose, points=line)
                    print("PROCESS SUCCEEDED")

            except:
                print("PROCESS FAILED")
                pass
                
    def Process(self):
        rospy.init_node("detect_centroid", anonymous=True)
        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)

        self.depth_scale    = 0.0010000000474974513
        self.object_pub     = rospy.Publisher( '/object/centroid', Marker, queue_size=10)
        self.line_pub       = rospy.Publisher( '/object/line', Marker, queue_size=10)
        self.pcmasking_pub = rospy.Publisher('/pointcloud_masking', PointCloud2, queue_size=10)
        self.arg_pub = rospy.Publisher('/argument', Float32, queue_size=10)
        # rospy.spin()
        while not rospy.is_shutdown():
            self.get_hsvcentroid()
            
  
if __name__ == '__main__':
    try:
        detect_centroid = DetectCentroid(use_gazebo=False)
        detect_centroid.window()
        detect_centroid.Process()

    except rospy.ROSInterruptException:
        pass


