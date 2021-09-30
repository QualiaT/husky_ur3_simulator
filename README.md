# husky_ur3_simulator

## Overview
This is a mobile manipulator simulator package using Gazebo, RViz, MoveIt, move_base.

The model of the mobile manipulator robot was created by combining Universal Robots's UR3 and Clearpath Robotics's Husky.

### Issue
Unfortunately, the gripper part of the robot is currently not working properly.(2021-07-23)


### Author:
- **[TaeHyeon Kim](https://github.com/QualiaT), qualiatxr@gmail.com**
- **[Myunghyun Kim](https://github.com/kmh8667), kmh8667@khu.ac.kr**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**



## Installation

### Dependencies
This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

- For Husky mobile robot control & navigation install
```
$ sudo apt-get install ros-melodic-husky-description
$ sudo apt-get install ros-melodic-husky-gazebo
$ sudo apt-get install ros-melodic-husky-viz
$ sudo apt-get install ros-melodic-husky-navigation
```

- For [MoveIt](https://moveit.ros.org/) install
```
$ sudo apt-get install ros-melodic-moveit
```

- For [ar_track_alvar package](https://github.com/ros-perception/ar_track_alvar) install
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-perception/ar_track_alvar.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

### husky_ur3_simulator install
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/QualiaT/husky_ur3_simulator.git

$ cd ~/catkin_ws/src/husky_ur3_simulator/"add file"
$ mv pr2_indiv_g_d435.launch ~/catkin_ws/src/ar_track_alvar/ar_track_alvar/launch
$ mv pr2_indiv_h_d435.launch ~/catkin_ws/src/ar_track_alvar/ar_track_alvar/launch
$ echo "export GAZEBO_MODEL_PATH=${HOME}/catkin_ws/src/husky_ur3_simulator/models" >> ~/.bashrc
$ source ~/.bashrc

$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
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
## Demo
- Bring up Gazebo with the robot model
![01](https://user-images.githubusercontent.com/87522493/126894178-fff15a46-084b-467d-ab79-00342c11b3d9.png)

- Bring up RViz
![02](https://user-images.githubusercontent.com/87522493/126894179-931a6e86-1f23-4c39-a117-6e848778c900.png)

- Use navigation stack and control the husky_ur3 robot by 2D Nav Goal
![03](https://user-images.githubusercontent.com/87522493/126894180-eee58562-234c-4c83-94c1-aa34b27d8c7f.png)
![04](https://user-images.githubusercontent.com/87522493/126894175-82393fef-d536-472d-97c4-1f9745dc5dee.png)
![05](https://user-images.githubusercontent.com/87522493/126894176-69413f38-e58f-4528-adea-48e183a290ef.png)