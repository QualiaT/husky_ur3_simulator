# husky_ur3_simulator Noetic version

## Overview
This is a mobile manipulator simulator package using Gazebo, RViz, MoveIt, move_base.

The model of the mobile manipulator robot was created by combining Universal Robots's UR3 and Clearpath Robotics's Husky.

This package is for ROS Noetic, but it also works normally in ROS Melodic.

### Author:
- **[SungWoo Yang](https://github.com/Sungwwoo), p1112007@khu.ac.kr**
- **[TaeHyeon Kim](https://github.com/QualiaT), qualiatxr@gmail.com**

**[SungWoo Yang](https://github.com/Sungwwoo) was in charge of the urdf version up, and [TaeHyeon Kim](https://github.com/QualiaT) modified the rest of some settings(navigation parameters, gazebo world files etc...).**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**



## Installation

### Dependencies
This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

- For Husky mobile robot control & navigation install
```
$ sudo apt-get install ros-noetic-husky-*
$ sudo apt-get install ros-noetic-ddynamic-reconfigure
$ sudo apt-get install ros-noetic-imu-filter-madgwick
$ sudo apt-get install ros-noetic-imu-transformer
```

- For [MoveIt](https://moveit.ros.org/) install
```
$ sudo apt-get install ros-noetic-moveit
```

- For [ar_track_alvar package](https://github.com/QualiaT/ar_track_alvar) install
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/QualiaT/ar_track_alvar.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

### husky_ur3_simulator install
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/QualiaT/husky_ur3_simulator.git

$ echo "export GAZEBO_MODEL_PATH=${HOME}/catkin_ws/src/husky_ur3_simulator/models" >> ~/.bashrc
$ source ~/.bashrc

$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

### Installation of dependencies for using gripper
```
- robotis_controller_msgs install
$ git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git

- dynamixel_sdk install
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git

- robotis_controller install
$ git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git

- ros_controllers
$ sudo apt-get install ros-noetic-ros-controllers
```

### For Realsense SDK 2.0 Installation
```
$ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
$ sudo add-apt-repository "deb http://librealsense.intel.com/Debian/apt-repo focal main" -u

$ sudo apt-get install librealsense2-dkms
$ sudo apt-get install librealsense2-utils
$ sudo apt-get install librealsense2-dev
$ sudo apt-get install librealsense2-dbg

# test
$ realsense-viewer
```

### For Realsense ROS package Installation
```
$ export ROS_VER=noetic
$ sudo apt-get install ros-noetic-realsense2-camera
$ cd ~/catkin_ws/src/
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
$ cd ~/catkin_ws/
$ catkin_make
```

## How to start?
```
- Bring up Gazebo with the robot model
$ roslaunch husky_ur3_gazebo husky_ur3_HRI_lab.launch

- Bring up MoveIt & RViz
$ roslaunch husky_ur3_gripper_moveit_config Omni_control.launch

- If you want to navigation using map, type the following command.
$ roslaunch husky_ur3_navigation husky_ur3_in_HRI_lab_amcl.launch
```

## How to use gripper?
```
- gripper open
$ rostopic pub -1 /rh_p12_rn_position/command std_msgs/Float64 "data: 0.0"

- gripper close
$ rostopic pub -1 /rh_p12_rn_position/command std_msgs/Float64 "data: 1.05"
```

## Demo
- Bring up Gazebo with the robot model
![01](https://user-images.githubusercontent.com/87522493/126894178-fff15a46-084b-467d-ab79-00342c11b3d9.png)

- Bring up RViz
![02](https://user-images.githubusercontent.com/87522493/126894179-931a6e86-1f23-4c39-a117-6e848778c900.png)

- Use navigation stack and control the husky_ur3 robot by 2D Nav Goal
![03](https://user-images.githubusercontent.com/87522493/126894180-eee58562-234c-4c83-94c1-aa34b27d8c7f.png)
![04](https://user-images.githubusercontent.com/87522493/126894175-82393fef-d536-472d-97c4-1f9745dc5dee.png)
![05](https://user-images.githubusercontent.com/87522493/126894176-69413f38-e58f-4528-adea-48e183a290ef.png)

- Using gripper
![06](https://user-images.githubusercontent.com/87522493/135451260-1c6f8d0a-3add-4038-a1b6-956e9d5c8c1c.png)
![07](https://user-images.githubusercontent.com/87522493/135451273-42ccdfc3-d666-4838-9202-be8769894c86.png)
