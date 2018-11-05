NRSL Autonomous UAV Project
===============================

## Description
This repository contains the code for UAV to implement autonomous navigation in an unknow enviorment. 

### Features

1. Using visual-inertial odometer to estimate uav pose.
2. Build a octree map in real time using stereo camera, and more importantly, this process dose not require GPU support.
3. Implemented a frotier-based exploration strategy and RRT*-based trajectory planning algorithm.
4. The established octree map and the trajectory of the uav can be optimized offline.
5. Once the exploration is complete, the uav can be relocalized to improve location accuracy and use the optimized octree map for path planning 

### Related Paper
- Autonomous Navigation of MAVs in Unkonwn Environments With Onboard Stereo Camera [pdf]()

### videos
<a href="https://www.youtube.com/watch?v=CR_JxXpjgnQ"> <img src="https://img.youtube.com/vi/CR_JxXpjgnQ/maxresdefault.jpg" alt="video" width="600"/>
</a>

## Installation

### Install ROS Kinetic
Ubuntu  16.04.
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
### Install [maplab](https://github.com/ethz-asl/maplab)

**You must install maplab and maplab_dependence in branch "pre_release_public/july-2018", not master branch**

### Install maplab plugin: octomap-maplab-plugin

```
    cd ~/maplab_ws/src
    git clone https://github.com/songjin321/octomap-maplab-plugin.git
    catkin build --no-deps octomap_maplab_plugin
``` 

### Install Our Autonomous Navigation WorkSpace
Clone the repository and catkin build:
```
    cd ~
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
    cd ./uav_navigation
    catkin build
    source ~/uav_navigation/devel/setup.bash
``` 

## How to Run it

### Prerequisite
- sensors
    - a stereo camera such as zed camera, publish raw image in topic **/cam0/image_raw** and **/cam1/image_raw**
    - a imu sensor such as xsen imu, publish raw imu information in topic **/imu0**
- move_base, provide a service, let uav fly to goal pose
- validate device, motion capture system, publish true pose in topic **/true_pose** 

### exploration

```
# terminal 1 
source ~/Project/uav_navigation/devel/setup.bash
roslaunch application exploration.launch

# terminal 2 
source ~/Project/maplab_ws/devel/setup.bash
./Project/maplab_ws/src/maplab/applications/rovioli/scripts/tutorials/huang_live ~/Documents/maps

# terminal 3 
rosbag record -j -b 0 /cam0/image_raw /cam1/image_raw /imu0 /vrpn_pose -O ~/Documents/exploration.bag

# terminal 4
rviz

```

### off-line optimization

```
sh ${PATH_UAV_NAVIGATION}/script/exploration.sh
```

### relocalization demo

```
sh ${PATH_UAV_NAVIGATION}/script/exploration.sh
```
