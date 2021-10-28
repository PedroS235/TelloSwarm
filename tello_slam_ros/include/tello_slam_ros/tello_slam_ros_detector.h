// - I/O
#include <iostream>

// - Ucoslam
#include <ucoslam/ucoslam.h>

// - ROS
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// - TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/QuadWord.h>

// - Messages
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>


class TelloSlamRos{
private:
    ucoslam::UcoSlam ucoslam;
    ucoslam::Params ucoslamParams;
    ucoslam::ImageParams ucoslamCameraParams;

// - Ucoslam map
protected:
    std::shared_ptr<ucoslam::Map> worldMap = std::make_shared<ucoslam::Map>();

protected:
    int frameNumber;
    std::string cameraCalibrationName;
    std::string vocabularyFileName;
    std::string worldMapName;

protected:
    std::string camera_info_topic_name;
    ros::Subscriber cameraInfoSub;
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);

protected:
    bool runSequential;
    bool detectMarkers;

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

protected: 
    cv::Mat cameraPose;

// - TF2
protected:
    tf2_ros::TransformBroadcaster* tfTransformBroadcaster;
    geometry_msgs::Transform cameraPose2Tf();

// - Constructors
public:
    TelloSlamRos(int argc, char **argv);
    ~TelloSlamRos();

public:
    int open(int argc, char **argv);
    int run();
    void close();
    void saveMapFile();

public:
    int openRos();
    void configureUcoslam(int argc, char **argv);
};