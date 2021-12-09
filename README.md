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
    - [tello_slam_ros](#tello-slam-ros)
        * [source files](#source-files)
        * [launch files](#launch-files)
    - [pose_estimaton](#pose_estimation)
        * [source files](#source-files)
        * [launch files](#launch-files)
+ [External dependencies](#external-dependencies)
+ [Running instructions](#running-instructions)
***

# Disclaimer
Use this this software at your own risk!
***

# ROS packages
This repository is composed of 4 different ROS packages.
## tello_ros
This package contains all the scripts that sends/receives commands from the drone and captures the video.
### Source files
- **tello_client.py:** This script allows to send commands to the Tello drone and receive their answer.
- **video_stream.py:** This script will retrieve the video from the drone and publish to a ROS topic named ```camera/image_raw```.
- **command_server.py:** This script allows to automatically connect to the drone's WiFi and will send the first commands which allows to communicate with the drone with commands.
- **camera_info_publisher.py**: This script will take a YML file which contains the different camera parameters and will publish these into a ROS topic named ```camera/camera_info```
- **video_recorder.py**: This script allows to record the video being retrieved from the drone and save it into a .mp4 video file. (useful to create a map for Ucoslam)

### Launch files
- **drone.launch**: This launch file initializes the scripts *command_server*, *video_stream* and *camera_info_publisher*.
- **prerecord.launch**: This launch file initializes the scripts *command_server* and *video_recorder*. It's purpose is only to record the video being captured from the drone.

## tello_formation
This package contains the scripts that allows to have the swarm of drones make some task. Currently, the task is to fly to a goal and land.

### Source files
- **capt.py**: This script is responsible to compute the trajectory for each drone in order to land on a goal. The algorithm tries to choose the closes goal to the drone without any collisions with other drones.
- **supervisor.py**: This script is the responsible to send the commands to each controller, that commands a drone.
- **controller.py**: This script controls a unique drone and it receives the commands from the supervisor.

### Launch files
- **static_goals.launch**: This launch file will publish the goals coordinates using ros tf2.
- **single_drone.launch**: This launch file will launch the ```drone.launch```(from the tello_ros package). Note that an id (a number) is required. This id is to distiguish the different drones. 
- **formation.launch**: This launch file will initialize all the necessary files to start the swarm.
- **sensors.launch**: This launch file initializes the tello_slam_ros launch files, also the aruco_eye launch files and the pose_estimation launch files.

## tello_slam_ros
This package is a ROS wrapper for Ucoslam.
### Source files
- **tello_slam_ros_detector_node.cpp**: This node will retrieve the image from the drone and will pass it to ucoslam. After the image was processed, it publishes the pose of the camera to the TF tree.
- **tello_slam_ros_display_node.cpp**: This node will retrieve the image from the drone and will publish the image with the key points drawn in the image to a ROS topic named ```tello_slam/tello_slam_observation_image/image_raw```
### Launch files
- **tello_slam_ros_detector.launch**: This launch file initializes the tello_slam_ros_detector_node.
- **tello_slam_ros_display.launch**: This launch file initializes the tello_slam_ros_display_node.

## pose_estimation
This packages create the required TF frames to localize the drones in the world.
### Source files
- **location_estimation.py**: This scripts creates new TF frames to locate the drones in the world, via the slam thechnology and the aruco_eye software.
- **static_transform.sh**: This script is just to facilitate the static transforms of the markers.
### Launch files
- **static_markes.launch**:
- **naive_estimation.launch**:

# External dependencies
- [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) (only tested with noetic distro)
- [Aruco_eye](https://github.com/joselusl/aruco_eye)
- [Ucoslam](http://www.uco.es/investiga/grupos/ava/node/62)
- [ros_numpy](https://github.com/eric-wieser/ros_numpy)
