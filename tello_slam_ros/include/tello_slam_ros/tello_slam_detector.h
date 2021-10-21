// - I/O
#include <iostream>

// - Ucoslam
#include <ucoslam/ucoslam.h>

// - ROS
#include <ros/ros.h>

// - Image
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// - TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/QuadWord.h>

// - Messages
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>


class TelloSlamRos{
private:
    ucoslam::UcoSlam ucoslam;
    ucoslam::Params ucoslamParams;
    ucoslam::ImageParams ucoslamCameraParams;

private:
    void loadConfigFiles(std::string cameraInfoFile, std::string vocabularyFile); 

// - Ucoslam map
protected:
    std::shared_ptr<ucoslam::Map> worldMap = std::make_shared<ucoslam::Map>();

protected:
    std::string cameraCalibrationName;

// - Images received
protected:
    std::string imageTopicName;
    cv_bridge::CvImagePtr cvImage;
    cv::Mat imageMat;
    image_transport::Subscriber imageSub;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

// - Images
protected:
    image_transport::ImageTransport* imageTransport;

// - TF2
protected:
    tf2_ros::TransformBroadcaster* tfTransformBroadcaster;

// - Constructors
public:
    TelloSlamRos(int argc, char **argv);
    ~TelloSlamRos();

public:
    int open();
    int run();
    void init();
    void close();

public:
    int openRos();
    void configureUcoslam();

};