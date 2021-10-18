// - Ucoslam
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>

// - Opencv
#include <opencv2/opencv.hpp>

// - ROS
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

// - imageSubscriber
#include "camera_subscriber.h"


class SlamNode{
private:
    ucoslam::UcoSlam slam;
    ucoslam::Params ucoslamParams;                                           // processing parameters 
    ucoslam::ImageParams cameraParams;                                      // camera parameters
    ucoslam::MapViewer mapViewer;                                           // map viewer and images
    std::shared_ptr<ucoslam::Map> map = std::make_shared<ucoslam::Map>();   // create an empty map
    ImageSubsciber is;

private:
    ros::NodeHandle nh;
    cv::Mat currFrame;
    ros::Subscriber cameraInfoSub;
    int frameCounter;


private:
    int framecounter;

private: 
    bool showMap = false;
    bool runInSequential = true;
    bool detectMarkers = true;

public:
    SlamNode(char **args){
        initialize(args);
        //currFrame = is.getCurrFrame();
        frameCounter = is.getFrameCouter();
    }

/*void cameraInfoCallback(const sensor_msgs::CameraInfo& camInfo){
        //cv::Mat cameraMatrix(3, 3, camInfo.K);
        //cv::Mat distortionCoeff(5, 1, camInfo.D);
        cv::Size size(camInfo.width, camInfo.height);
    }*/

        
public:
    void initialize(char **args){
        framecounter = 1;
        // TODO: now the parameters needs to be passed by the args.
        //       so next create a topic where the camera parameters will be
        //       Then, subscribe to the topic to get the parameters 
        cameraParams.readFromXMLFile(args[1]);
        ucoslamParams.runSequential = runInSequential;
        ucoslamParams.detectMarkers = detectMarkers;
        // TODO: also create a topic to get the where the vocabulary will be published
        //       and then subscribe to it
        slam.setParams(map, ucoslamParams, args[2]);
    }

    void visualizer(){
        char keyPressed;
            //currFrame = is.getCurrFrame();
            frameCounter = is.getFrameCouter();
            std::cout << frameCounter << std::endl;
            //std::cout << currFrame.cols << std::endl;
            /*
            cv::Mat posef2g = slam.process(currFrame, cameraParams, framecounter);
            
            if(posef2g.empty()) std::cerr << "Frame " << framecounter << "pose not found" << std::endl;
            else std::cerr << "Frame " << framecounter << "pose " << posef2g << std::endl;

            keyPressed = mapViewer.show(map, currFrame, posef2g);
            */
    }

};