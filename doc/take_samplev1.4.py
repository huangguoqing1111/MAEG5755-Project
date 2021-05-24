#!/usr/bin/env python3
#coding=utf-8
import rospy
import math
# from std_msgs.msg import String
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
import cv2
import sys
import pyrealsense2 as rs
import numpy as np

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 
# Start streaming
pipeline.start(config)

def take_sample():
    rospy.init_node('Take_sample', anonymous=True)
    # rospy.Subscriber('/tf', TFMessage, PoseCallback)
    # rospy.Subscriber('/camera/depth/image_rect_raw', Image, ImgCallback)

    cnt = 0
    flag_cap = 0

    while not rospy.is_shutdown():
        # # show video stream
        # Wait for a coherent pair of frames: color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to np arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        depthImage = depth_image * 255.0 / 1000.0
        depthImage = np.nan_to_num(depthImage, nan=0.0)
        depthImage = depthImage.astype(np.uint8)
        # depthImage = cv2.cvtColor(depthImage, cv2.COLOR_GRAY2BGR) # 8UC1
        # print(depthImage)
        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))
 
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)

        if flag_cap:
            # # Take samle
            # 1. Image
            cnt = cnt + 1
            cv2.imwrite('Img_marker'+str(cnt) + '.png', depthImage)
            flag_cap = 0 # reset flag

        key = cv2.waitKey(50)
        if key & 0xFF == ord('t'):
            flag_cap = 1 # set flag
        elif key & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    take_sample()
