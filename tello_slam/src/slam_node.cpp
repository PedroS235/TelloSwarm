#include <slam_node.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "Slam_node");
    SlamNode sn(argv);
    sn.visualizer();
    ros::spin();
    return 0;
}