import pyrealsense2 as rs
import cv2
import numpy as np
import math

def  cameraSetting(width=1280, height=720):
    # Get device product line for setting a supporting resolution
    pipeline = rs.pipeline()
    config = rs.config() # Configure streams

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    # device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

    cfg = pipeline.start(config)

    # Camera calibration data
    profile = cfg.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
    intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics

    camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float32) 
    camera_distortion = np.array([0,0,0,0,0], dtype=np.float32)

    return pipeline, camera_matrix, camera_distortion,intr

def cameraProcess(pipeline):
    align_to = rs.stream.color
    align = rs.align(align_to)

    frames = pipeline.wait_for_frames()
        
    # Align the depth frame to color frame
    aligned_frames = align.process(frames)
    
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not depth_frame or not color_frame: pass

    hole_filling = rs.hole_filling_filter()
    filled_frame = hole_filling.process(depth_frame)
    
    # Convert images to numpy arrays
    filled_depth_image = np.asanyarray(filled_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    return filled_depth_image, color_image,filled_frame

def main():
    width=640 ; height=480
    pipeline, camera_matrix, camera_distortion,intr = cameraSetting(width, height)

    while True:
        depth_image, color_image,depth_frame = cameraProcess(pipeline)
        cv2.imshow("image", color_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    main()
    
    