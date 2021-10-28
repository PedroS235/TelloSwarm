cmake_minimum_required(VERSION 3.0.2)
project(tello_slam_ros)

set(TELLO_SLAM_ROS_SOURCE_DIR src)
set(TELLO_SLAM_ROS_INCLUDE_DIR include)
set(TELLO_SLAM_ROS_SOURCE_FILES 
  ${TELLO_SLAM_ROS_SOURCE_DIR}/tello_slam_ros_detector.cpp
  ${TELLO_SLAM_ROS_SOURCE_DIR}/tello_slam_ros_display.cpp
)
set(TELLO_SLAM_ROS_HEADER_FILES
  ${TELLO_SLAM_ROS_INCLUDE_DIR}/tello_slam_ros/tello_slam_ros_detector.h
  ${TELLO_SLAM_ROS_INCLUDE_DIR}/tello_slam_ros/tello_slam_ros_display.h
)

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

find_package(OpenCV REQUIRED)
find_package(ucoslam REQUIRED)

catkin_package(
  INCLUDE_DIRS ${TELLO_SLAM_ROS_INCLUDE_DIR} 
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS roscpp message_filters image_transport cv_bridge geometry_msgs tf2 tf2_geometry_msgs tf2_ros perception_msgs
  DEPENDS OpenCV 
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${TELLO_SLAM_ROS_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ucoslam_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${TELLO_SLAM_ROS_SOURCE_FILES} ${TELLO_SLAM_ROS_HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_detector ${TELLO_SLAM_ROS_SOURCE_DIR}/tello_slam_ros_detector_node.cpp)
add_dependencies(${PROJECT_NAME}_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_detector ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_detector ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_detector ${ucoslam_LIBS})
target_link_libraries(${PROJECT_NAME}_detector ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_display ${TELLO_SLAM_ROS_SOURCE_DIR}/tello_slam_ros_display_node.cpp)
add_dependencies(${PROJECT_NAME}_display ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_display ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_display ${ucoslam_LIBS})
target_link_libraries(${PROJECT_NAME}_display ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_display ${catkin_LIBRARIES})
