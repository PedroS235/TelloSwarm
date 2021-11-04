// - I/O
#include <iostream>

// - Ucoslam
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>

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

class TelloSlamDisplayRos{
private:
    ucoslam::UcoSlam ucoslam;
    ucoslam::Params ucoslamParams;
    ucoslam::ImageParams ucoslamCameraParams;
    ucoslam::MapViewer mapViewer;

// - Ucoslam map
protected:
    std::shared_ptr<ucoslam::Map> worldMap = std::make_shared<ucoslam::Map>();

protected:
    int frameNumber;
    std::string cameraCalibrationFileName;
    std::string vocabularyFileName;
    std::string inputWorldMapFileName;
    std::string outputWorldMapFileName;

protected:
    std::string cameraInfoTopicName;
    ros::Subscriber cameraInfoSub;
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);

protected:
    bool runSequential;
    bool detectMarkers;
    bool showDisplay;
    bool cameraInfoReceived = false;

protected:
    int arucoIdGlobalReference;

// - Images received
protected:
    std::string imageTopicName;
    cv_bridge::CvImagePtr cvImage;
    image_transport::Subscriber imageSub;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

// - Images
protected:
    image_transport::ImageTransport* imageTransport;

protected: 
    cv::Mat cameraPose;

// - Images output
protected:
    std::string outputImageTopicName;
    cv::Mat outputImageMat;
    image_transport::Publisher outputImagePub;

// - Constructors
public:
    TelloSlamDisplayRos(int argc, char **argv);
    ~TelloSlamDisplayRos();

public:
    int open();
    int run();
    void saveMapFile();
    void loadMapFromFile();
    void setCenterReferenceOfMap();

public:
    int openRos();
    void configureUcoslam();
private:
    void readParameters();
};