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
import tf
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
from tf.transformations import quaternion_matrix
from baxter_interface import Limb

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 
# Start streaming
pipeline.start(config)

# def PoseCallback(tf_msg):
#     global baxter_ee_pose
    
#     # Update all joint states
#     tf_baxter = tf.TransformerROS()
#     temp_msg = TransformStamped()
#     for i in range(0,len(tf_msg.transforms)):
#             temp_msg = tf_msg.transforms[i]
#             tf_baxter.setTransform(temp_msg)
        
#     # Lookup transform from base to grippers
#     try:
#         trans_left = tf_baxter.lookupTransform('torso','left_gripper',rospy.Time())
#         trans_right = tf_baxter.lookupTransform('torso','right_gripper',rospy.Time())
#         rospy.loginfo("got gripper's pose successfully")
#         baxter_ee_pose = trans_left
        
#     except:
#         rospy.loginfo("trying to get connect with tf \n")


# def ImgCallback(Image):
#     global cv_img
#     try:
#         # cv_img = CvBridge.imgmsg_to_cv2(Image, "rgb8")
#         cv_img = CvBridge.imgmsg_to_cv2(Image, "passthrough")
#         cv2.imshow('Img_marker', cv_img)
#     except CvBridgeError as e:
#         print(e)


def take_sample():
    rospy.init_node('Take_sample', anonymous=True)
    # rospy.Subscriber('/tf', TFMessage, PoseCallback)
    # rospy.Subscriber('/camera/depth/image_rect_raw', Image, ImgCallback)
    right_arm = Limb('right')
    left_arm = Limb('left')

    cnt = 0
    flag_cap = 0

    print('start')
    while not rospy.is_shutdown():
        # # show video stream
        # Wait for a coherent pair of frames: color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
 
        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
 
        # # Stack both images horizontally
        # images = np.hstack(color_image)
 
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)

        if flag_cap:
            # # Take samle
            # 1. Image
            cnt = cnt + 1
            cv2.imwrite('Img_marker'+str(cnt) + '.jpg', color_image)

            # 2. End effector Pose
            ee_pose = left_arm.endpoint_pose()
            ee_position = ee_pose['position']
            ee_orientation = ee_pose['orientation']

            quat = [ee_orientation.x,  ee_orientation.y, ee_orientation.z, ee_orientation.w] 
            T_matrix = quaternion_matrix(quat) # order need to be checked, here is xyzw
            transl = np.array([ee_position.x, ee_position.y, ee_position.z])
            T_matrix[0:3, 3] = transl.T
            print('id', cnt, 'EE_pose',T_matrix)

            T_matrix = np.array(T_matrix)
            f_name = str('EE_pose' + str(cnt) + '.txt')
            np.savetxt(f_name, T_matrix, fmt = '%.6f', delimiter = ',')
            # fo = open('EE_pose' + str(cnt) + '.txt', 'w')
            # fo.close
            flag_cap = 0 # reset flag

        key = cv2.waitKey(50)
        if key & 0xFF == ord('t'):
            flag_cap = 1 # set flag
        elif key & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    take_sample()
