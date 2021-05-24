# 2020R2 MAEG57550 Robotics Project 2 Bin Picking
=====

## Preparation
### 1 Install nvidia driver
```bash
sudo apt purge nvidia-*
sudo apt autoremove
reboot
```
```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo apt-get install nvidia-driver-460 nvidia-cuda-toolkit
```

### 2 Install ROS
```
git clone https://github.com/rojas70/learning_ros_setup_scripts.git
cd learning_ros_setup_scripts/melodic/
chmod +x *.sh
./install_ros_and_tools_melodic.sh 
./setup_workspace_learning_ros_melodic.sh [user_name] [email]
```

### 3 Install dependencies
``` 
sudo apt install ros-melodic-eigen-stl-containers 
sudo apt install ros-melodic-eigenpy
sudo apt install ros-melodic-graph-msgs 
sudo apt install ros-melodic-octomap
sudo apt install ros-melodic-ompl
sudo apt install ros-melodic-pybind11-catkin 
sudo apt install ros-melodic-random-numbers
sudo apt install ros-melodic-srdfdom

sudo apt install ros-melodic-controller-manager
sudo apt install ros-melodic-controller
sudo apt install ros-melodic-four-wheel-steering-msgs 
sudo apt install ros-melodic-gazebo-plugins
sudo apt install ros-melodic-gazebo-ros
sudo apt install ros-melodic-gazebo-ros-control
sudo apt install ros-melodic-rosparam-shortcuts
sudo apt install ros-melodic-ros-controller
sudo apt install ros-melodic-ros-controllers 
sudo apt install ros-melodic-ros-control
sudo apt install ros-melodic-urdf-geometry-parser
sudo apt install ros-melodic-warehouse-ros
sudo apt install ros-$ROS_DISTRO-gazebo-ros-control ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander ros-${ROS_DISTRO}-moveit-visual-tools

sudo apt install libglew-dev
sudo apt install python3-wstool
sudo apt install python3-pip
python3 -m pip install --upgrade pip
pip3 install setuptools


sudo apt install libqhull-dev
sudo apt install libqhull-r7 
```


### 4. Install other dependencies from source code
```
git clone git://github.com/flexible-collision-library/fcl.git
mkdir fcl/build && cd fcl/build
cmake ..
make
sudo make install
```

```
git clone https://github.com/atenpas/gpd.git
mkdir gpd/build && cd gpd/build
cmake ..
make
sudo make install
```

```
clone https://github.com/qhull/qhull
mkdir qhull/build && cd qhull/build
cmake ..
make 
sudo make install
```
## Installation

```
roscd
git clone https://github.com/rojas70/learning_ros_external_pkgs_noetic.git
git clone https://github.com/ros-planning/moveit_core
git clone https://github.com/ros-planning/geometric_shapes
git clone https://github.com/ros-planning/moveit_ros_planning_interface
git clone https://github.com/ros-planning/moveit  
git clone https://github.com/tahsinkose/panda_moveit_config.git -b melodic-devel
git clone https://github.com/tahsinkose/franka_ros.git -b simulation
git clone https://github.com/robot-chenwei/moveit_task_constructor.git
git clone https://github.com/PickNikRobotics/deep_grasp_demo.git
```

```
cd deep_grasp_demo
chmod +x pcl_install.sh
sudo ./pcl_install.sh
chmod +x opencv_install.sh
sudo ./opencv_install.sh
wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/dexnet_install.sh
wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/dexnet_requirements.txt
chmod +x dexnet_install.sh
./dexnet_install.sh gpu
./dexnet_deps/gqcnn/scripts/downloads/models/download_models.sh
```

