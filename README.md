MAEG5755 Project




Team members

Guoqing Huang  , Benyun Zhao , Zimeng Guo ,  Ziheng Huang





Platfrom

Ubuntu 20.04 LTS
ROS :  Noetic 
NVIDIA : 460
CUDA :  10.0
CUDNN : 7.4
Tensorflow_gpu-1.13.1





Dependences

we need to add a lot dependence files in our system , such as python3.6 , due to we use the 20.04 and we may have 
python3.8 , and we should get the kdl follow the   http://github.com/orocos/orocos_kinematics_dynamics . 
and some dependences we need follow the https://github.com/PickNikRobotics/deep_grasp_demo#Install-Dex-Net to 
set.  and some packages in  https://github.com/mfkenson/MAEG5755-2021-Team-PARK.   and  https://github.com/robot-chenwei/MAEG5755_Robotics_Project.









Run the simulation 

roslaunch deep_grasp_task baxter_simulation.launch

roslaunch moveit_task_constructor_dexnet dexnet_baxter_demo.launch
This simulation have a video below.
https://www.bilibili.com/video/BV1kq4y1E72k/
https://www.bilibili.com/video/BV1ag41137jW/




 Real robot question 

calibration we follow chenwei and I will put it on the internet.
The  network  question follow Kenson ,  and  our Team do some experiments on real robot.  This will have a video below.
https://www.bilibili.com/video/BV14q4y1E7Ds/
