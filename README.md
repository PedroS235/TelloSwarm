# Tello swarm

Tello swarm includes multiple ROS nodes that enables the DJI Tello drones in to fly in a swarm manner. It is using VSLAM technology and visual markers to estimate the location of the drones in the environment. Some goals can be added manually, and then the drones shall choose the best goal that suits them the best and then land on them.
***

# Table of contents
+ [Disclaimer](#disclaimer)
+ [ROS packages](#ros-packages)
    - [tello_ros](#tello-ros)
        * [source files](#source-files)
        * [launch files](#launch-files)
        * [ROS topics](#ros-topics)
    - [tello_formation](#tello-formation)
        * [source files](#source-files)
        * [launch files](#launch-files)
        * [ROS topics](#ros-topics)
    - [tello_slam_ros](#tello-slam-ros)
        * [source files](#source-files)
        * [launch files](#launch-files)
        * [ROS topics](#ros-topics)
    - [pose_estimaton](#pose_estimation)
        * [source files](#source-files)
        * [launch files](#launch-files)
        * [ROS topics](#ros-topics)
+ [External dependencies](#external-dependencies)
+ [Running instructions](#running-instructions)
***

# Disclaimer
Use this this software at your own risk!
***

# ROS packages
This repository is composed of 4 different ROS packages.
## tello_ros
This package contains all the Python scripts which (you can find inside the folder src) to control the drone and to
capture the video.
### Source files
### Launch files
### ROS topics

## tello_formation
This package contains all the Python scripts (which you can find inside the folder src) that runs on the supervisor
and controllers. It also contains the script to compute the drones trajectories
### Source files
### Launch files
### ROS topics

## tello_slam_ros
### Source files
### Launch files
### ROS topics

## pose_estimation
### Source files
### Launch files
### ROS topics

# External dependencies
- [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) (only tested with noetic distro)
- [Aruco_eye](https://github.com/joselusl/aruco_eye)
- [Ucoslam](http://www.uco.es/investiga/grupos/ava/node/62)
- [ros_numpy](https://github.com/eric-wieser/ros_numpy)
