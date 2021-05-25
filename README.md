#  MAEG5755 Robotics Project 2 Bin Picking
------
## Summary
Team members
Guoqing Huang  , Benyun Zhao , Zimeng Guo ,  Ziheng Huang
This is our MAEG5755 Project,our team spend one month doing it 
## Overview
First,we know camera get the depth image and will sent to the Dex-Net 4.0 to get the grasp candidates.Then combine with inverse kinematics, the most approrpiate candidate is selected as the grasp pose.at last, run the robot by using inverse kinematics to achieve the pick and place task.

## QuickStart

### Baxter simulation

```bash
roslaunch deep_grasp_task baxter_simulation.launch
```

```bash
roslaunch moveit_task_constructor_dexnet dexnet_baxter_demo.launch
```

### real robot
```bash
./baxter.sh
roslaunch moveit_task_constructor_dexnet dexnet_baxter.launch 
```
calibration we follow chenwei and I will put it on the internet.
The  network  question follow Kenson
## Structure
+ dex-net 4.0
+ moveit support packages
+ baxter related packages
+ baxter_pykdl


## Video
simulation:
https://www.bilibili.com/video/BV1kq4y1E72k/
https://www.bilibili.com/video/BV1ag41137jW/

real robot:
https://www.bilibili.com/video/BV14q4y1E7Ds/

## FAQ

### 1. Platfrom
+ Ubuntu 20.04 LTS
+ ROS :  Noetic 
+ NVIDIA : 460
+ CUDA :  10.0
+ CUDNN : 7.4
+ Tensorflow_gpu-1.13.1

### 2. Dependencies
we need to add a lot dependence files in our system , such as python3.6 , due to we use the 20.04 and we may have 
python3.8 , and we should get the kdl follow the   http://github.com/orocos/orocos_kinematics_dynamics . 
and some dependences we need follow the https://github.com/PickNikRobotics/deep_grasp_demo#Install-Dex-Net to 
set.  and some packages in  https://github.com/mfkenson/MAEG5755-2021-Team-PARK.   and  https://github.com/robot-chenwei/MAEG5755_Robotics_Project.


