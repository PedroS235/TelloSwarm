// - TelloSlam
#include <tello_slam_ros/tello_slam_ros_detector.h>

int main(int argc, char **argv){
    TelloSlamRos telloSlamRos(argc, argv);
    std::cout << "[ROSNODE] starting " << ros::this_node::getName() << std::endl;

    telloSlamRos.open();
    telloSlamRos.run();
    telloSlamRos.saveMapFile();
    return 0;
}