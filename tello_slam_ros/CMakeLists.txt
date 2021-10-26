cmake_minimum_required(VERSION 3.0.2)
project(tello_slam_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(ucoslam REQUIRED)
link_libraries(${OpenCV_LIBS} ${ucoslam} ${UCOSLAM_REQUIRED_LIBRARIES} ${ucoslam_LIBS})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tello_slam
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### Declare a C++ library
#add_library(${PROJECT_NAME}
#  src/tello_slam_detector.cpp
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(tello_slam_detector src/tello_slam_detector.cpp)
#add_executable(tello_slam_detector_node src/tello_slam_detector_node.cpp)
add_executable(pose_publisher src/pose_publisher.cpp)
add_executable(camera_subscriber src/camera_subscriber.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(tello_slam_detector ${tello_slam_detector_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(tello_slam_detector_node ${tello_slam_detector_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_publisher ${pose_publisher_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(camera_subscriber ${camera_subscriber_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(tello_slam_detector
  ${catkin_LIBRARIES}
)

target_link_libraries(pose_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(camera_subscriber
  ${catkin_LIBRARIES}
)