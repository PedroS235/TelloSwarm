
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>


class SlamNode{
private:
    ucoslam::UcoSlam slam;
    ucoslam::Params ucoslamParams;
    ucoslam::ImageParams cameraParams;
    ucoslam::MapViewer mapViewer;
    std::shared_ptr<ucoslam::Map> map = std::make_shared<ucoslam::Map>();

private:
    ros::NodeHandle nh;
    cv::Mat currFrame;

private:
    int framecounter;
    

private: 
    bool showMap = false;
    bool runInSequential = true;
    bool detectMarkers = true;

public:
    SlamNode(char **args){
        initialize(args);
    }
    SlamNode(){
    }

public:
    void passTheCurrentFrame(cv::Mat frame, int frameNumber){
        currFrame = frame;
        framecounter = frameNumber;
    }

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
        while(keyPressed != 27){
            cv::Mat posef2g = slam.process(currFrame, cameraParams, framecounter);
            
            if(posef2g.empty()) std::cerr << "Frame " << framecounter << "pose not found" << std::endl;
            else std::cerr << "Frame " << framecounter << "pose " << posef2g << std::endl;

            keyPressed = mapViewer.show(map, currFrame, posef2g);
        } 
    }

};