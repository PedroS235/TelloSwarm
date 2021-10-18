#include <ros/ros.h>
#include <iostream>
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <opencv2/highgui/highgui.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

void poseCallback(cv::Mat imagePose){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "Camera_pose";
    transformStamped.transform.translation.x = imagePose.at<double>(3, 0); 
    transformStamped.transform.translation.y = imagePose.at<double>(3, 1);
    transformStamped.transform.translation.z = imagePose.at<double>(3,2);
    tf2::Quaternion q;
    //q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
}

int main(int argc,char **argv){
    try {
        if(argc!=5) throw std::runtime_error("usage : video cameraparameters.yml vocabulary.fbow outputmap.map");

        cv::VideoCapture VideoIn;//video capturer
        ucoslam::UcoSlam SLAM;//The main class
        ucoslam::Params UcoSlamParams;//processing parameters
        ucoslam::ImageParams cameraParams;//camera parameters
        ucoslam::MapViewer MapViwer;//Viewer to see the 3D map and the input images

        //creates an empty map
        std::shared_ptr<ucoslam::Map> map=std::make_shared<ucoslam::Map>();
        //map->loadFromFile(pathToFile);
        //open video
        VideoIn.open(argv[1]);
        if(!VideoIn.isOpened()) throw std::runtime_error("Could not open video:"+string(argv[1]));
        //read camera params
        cameraParams.readFromXMLFile(argv[2]);
        //set the slam params for Kitti using orb descriptor
        UcoSlamParams.runSequential=true;//run in sequential mode to avoid skipping frames
        UcoSlamParams.detectMarkers=false;//no markers in this example.

        //Start UcoSlam
        SLAM.setParams(map,UcoSlamParams,argv[3]);//the last parameter is the path to the vocabulary file of extension .fbow



        cv::Mat inputImage;
        char keyPressed=0;
        while( VideoIn.grab() && keyPressed!=27){//keyPressed ==27 is esc
            VideoIn.retrieve(inputImage);
            int frameNumber=VideoIn.get(cv::CAP_PROP_POS_FRAMES);
            cv::Mat posef2g= SLAM.process(inputImage,cameraParams,frameNumber);
            /*for(int i=0; i<4; i++){
                for(int j=0; i<4; j++){
                    
                }
            }*/
            if(posef2g.empty()){
                std::cerr<<"Frame "<<frameNumber<<" pose not found"<<std::endl;
            }
            else
                std::cerr<<"Frame "<<frameNumber<<" pose "<<posef2g.at<double>(1,1)<<std::endl;
            //draw a mininimal interface in an opencv window
            keyPressed=MapViwer.show(map,inputImage,posef2g);
        }
        //now,  save the map
        map->saveToFile(argv[4]);
    } catch (std::exception &ex) {
        std::cout<<ex.what()<<std::endl;
    }
}