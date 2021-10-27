// - TelloSlam
#include <tello_slam_ros/tello_slam_display.h>

int main(int argc, char **argv){
    TelloSlamDisplayRos telloSlamDisplayRos(argc, argv);
    std::cout << "[ROSNODE] starting " << ros::this_node::getName() << std::endl;

    telloSlamDisplayRos.open(argc, argv);
    telloSlamDisplayRos.run();
    telloSlamDisplayRos.saveMapFile();
    return 0;
}