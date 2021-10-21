#include <tello_slam_ros/tello_slam_detector.h>

TelloSlamRos::TelloSlamRos(int argc, char **argv){
    // - Initialize ROS
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    imageTransport = new image_transport::ImageTransport(nh);
    tfTransformBroadcaster = new tf2_ros::TransformBroadcaster();

    return;
}

TelloSlamRos::~TelloSlamRos(){
    if(imageTransport) delete imageTransport;
    if(tfTransformBroadcaster) delete tfTransformBroadcaster;
    close();
    return;
}

void TelloSlamRos::close(){
    return;
}

int TelloSlamRos::openRos(){
    ros::NodeHandle nh;

    // - Image subscriber
    imageSub = imageTransport -> subscribe(imageTopicName, 1, &TelloSlamRos::imageCallback, this);
    return 0;
}

int TelloSlamRos::open(){
    // - configure Ucoslam
    configureUcoslam();

    openRos();
    
    return 0;
}


void TelloSlamRos::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    // - Transform image to OpenCV compatible
    try{
        cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imageMat = cvImage -> image;

    /*
     * ToDo:
     * Add the ucoslam process method here!
     */

}

void TelloSlamRos::configureUcoslam(){
}

int TelloSlamRos::run(){
    try{
        ros::spin();
    }catch(std::exception& e){
        std::cout << "[ROSNODE] Exception: " << e.what() << std::endl;
    }

    return 0;
}

int main(int argc, char **argv){
    TelloSlamRos telloSlamRos(argc, argv);
    std::cout << "[ROSNODE] starting " << ros::this_node::getName() << std::endl;

    telloSlamRos.open();
    telloSlamRos.run();

    return 0;
}