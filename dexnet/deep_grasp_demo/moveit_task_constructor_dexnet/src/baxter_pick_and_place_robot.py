#!/usr/bin/env python3

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse, GQCNNGraspRequest
from baxter_pykdl import baxter_kinematics
import numpy

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        #ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        #self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        #rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self._kin = baxter_kinematics('left')

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        joint_pos = {'left_w0': 0, 'left_w1': 0, 'left_w2': 0, 'left_e0': 0, 'left_e1': 0, 'left_s0': 0, 'left_s1': 0}
        xpos = [pose.position.x, pose.position.y, pose.position.z]
        xori = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        jpos = self._kin.inverse_kinematics(xpos, xori)  
        if jpos:
            joint_pos['left_s0'] = jpos.item(0)[0]
            joint_pos['left_s1'] = jpos.item(0)[1]
            joint_pos['left_e0'] = jpos.item(0)[2] 
            joint_pos['left_e1'] = jpos.item(0)[3]
            joint_pos['left_w0'] = jpos.item(0)[4]
            joint_pos['left_w1'] = jpos.item(0)[5]
            joint_pos['left_w2'] = jpos.item(0)[6]
            return joint_pos
        else:
            return None

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import pyrealsense2 as rs
import geometry_msgs.msg
from tf.transformations import quaternion_matrix

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 
# Start streaming
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align = rs.align(rs.stream.color)

def main():
    print('INFO: Start python script!!!')
    global bridge, saveImage, saveImageFinish
    rospy.init_node("baxter_pick_and_place_demo")
    print('INFO: Start ros node !!!')
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame

    # Wait for the All Clear from emulator startup
    # rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(0, 1, 0, 0)
                             
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.

    print('INFO: Start read images !!!')
    getImage = False
    while not getImage:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame or not color_frame:
            continue
        else:
            getImage = True

    # Convert images to np arrays
    colorImage = numpy.asanyarray(color_frame.get_data())
    grayImage = cv2.cvtColor(colorImage, cv2.COLOR_BGR2GRAY)
    depthImage = numpy.asanyarray(depth_frame.get_data())
    depthImage = (depthImage < 900) * depthImage
    segmask = numpy.zeros(grayImage.shape)
    segmask[150:400, 80:540] = 1
    depthImage = segmask * depthImage
    depthImage = (grayImage > 25) * depthImage
    depthImage = (grayImage < 100) * depthImage
    depthImage = depthImage * 255.0 / 1000.0
    depthImage = numpy.nan_to_num(depthImage, nan=0.0)
    depthImage = depthImage.astype(numpy.uint8)
    # depthImage = cv2.cvtColor(depthImage, cv2.COLOR_GRAY2BGR) # 8UC1
    print('save image')
    cv2.imwrite('./color.png', colorImage)
    cv2.imwrite('./depth.png', depthImage)
    rospy.sleep(1)
    saveImageFinish = True
    print('calculate pose')

    try: 
        rospy.wait_for_service('gqcnn_grasp')
        gqcnn_grasp = rospy.ServiceProxy('gqcnn_grasp', GQCNNGrasp)
        req = GQCNNGraspRequest()
        req.color_img_file_path = './color.png'
        req.depth_img_file_path = './depth.png'
        res = gqcnn_grasp(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    rTc = numpy.array([[ 0.9960, -0.0281, -0.0848,  0.9039], 
                       [ 0.0347, -0.7526,  0.6576, -0.2598],
                       [-0.0823, -0.6579, -0.7486,  0.4249],
                       [ 0.0000,  0.0000,  0.0000,  1.0000]])
    res_size = 0
    if res:
        res_size = len(res.q_values)
    if res_size:
        q_values = 0.0
        grasps = copy.deepcopy(res.grasps[0])
        canGrasp = False
        for i in range(0, res_size):
            res.grasps[i].pose.orientation = overhead_orientation
            x = res.grasps[i].pose.position.x
            y = res.grasps[i].pose.position.y
            z = res.grasps[i].pose.position.z
            cPo = numpy.array([x, y, z, 1])
            bPo = numpy.dot(rTc, cPo)
            print('cPo', cPo)
            print('bPo', bPo)
            res.grasps[i].pose.position.x = bPo[0]
            res.grasps[i].pose.position.y = bPo[1]
            res.grasps[i].pose.position.z = bPo[2]
            
            canSolveIK1 = pnp.ik_request(res.grasps[i].pose)
            res.grasps[i].pose.position.z += pnp._hover_distance
            canSolveIK2 = pnp.ik_request(res.grasps[i].pose)
            res.grasps[i].pose.position.z -= pnp._hover_distance
            canSolveIK = False
            if canSolveIK1 and canSolveIK2:
                canSolveIK = True
            if canSolveIK and q_values < res.q_values[i]:
                q_values = res.q_values[i]
                grasps = copy.deepcopy(res.grasps[i])
                canGrasp = True
        print("INFO: Get pose from gqcnn_grasp with max q_values")
        print(grasps, '\n q_value = ', q_values)
        print(pnp.ik_request(grasps.pose))
        grasps.pose.position.z += pnp._hover_distance
        print(pnp.ik_request(grasps.pose))
        grasps.pose.position.z -= pnp._hover_distance
    else:
        print('ERROR: No return from gqcnn!')
    if not canGrasp:
        print('ERROR: Can not find the grasping pose that can solve IK')
        grasps.pose = Pose(
        position=Point(x=0.70, y=0.15, z=-0.129),
        orientation=overhead_orientation)

    block_poses = list()
    pose1 = copy.deepcopy(grasps.pose)
    # pose1.position.z = -0.02
    block_poses.append(pose1)
    pose2 = copy.deepcopy(grasps.pose)
    # pose2.position.z = -0.02
    # pose2.position.y += 0.20
    #if pose2.position.y > 0.5:
    pose2.position.y = 0.5
        
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    pnp.pick(pose1)
    pnp.place(pose2)
    pnp.move_to_start(starting_joint_angles)
    return 0

if __name__ == '__main__':
    sys.exit(main())
