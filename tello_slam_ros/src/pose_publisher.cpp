#include <ros/ros.h>
#include <iostream>
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <opencv2/highgui/highgui.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

void poseCallback(cv::Mat imagePose){

    cv::Mat rot(3, 3, CV_32FC1);
    cv::Mat origin(1, 3, CV_32FC1);
    for(int row = 0; row<2; row++){
        for(int col = 0; col<2; col++){
            rot.at<float>(row, col) = imagePose.at<float>(row, col);
        }
    } 

    for(int row  = 0; row<3; row++)
        origin.at<float>(0, row) = imagePose.at<float>(row, 3);

    tf2::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                       rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                       rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

    tf2::Vector3 tf_orig(origin.at<float>(0,0), origin.at<float>(0,1), origin.at<float>(0,2));

    tf2::Transform tf2_transform = tf2::Transform(tf_rot, tf_orig);
    geometry_msgs::Transform transform=tf2::toMsg(tf2_transform);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "Camera_pose";
    transformStamped.transform = transform;
    br.sendTransform(transformStamped);
}

int main(int argc,char **argv){
    try {
        if(argc!=5) throw std::runtime_error("usage : video cameraparameters.yml vocabulary.fbow outputmap.map");

        cv::VideoCapture VideoIn (0);//video capturer
        ucoslam::UcoSlam SLAM;//The main class
        ucoslam::Params UcoSlamParams;//processing parameters
        ucoslam::ImageParams cameraParams;//camera parameters
        ucoslam::MapViewer MapViwer;//Viewer to see the 3D map and the input images

        //creates an empty map
        std::shared_ptr<ucoslam::Map> map=std::make_shared<ucoslam::Map>();
        //map->loadFromFile(pathToFile);
        //open video
        //VideoIn.open(argv[1]);
        if(!VideoIn.isOpened()) throw std::runtime_error("Could not open video:"+string(argv[1]));
        //read camera params
        cameraParams.readFromXMLFile(argv[2]);
        //set the slam params for Kitti using orb descriptor
        UcoSlamParams.runSequential=true;//run in sequential mode to avoid skipping frames
        UcoSlamParams.detectMarkers=false;//no markers in this example.

        //Start UcoSlam
        SLAM.setParams(map,UcoSlamParams,argv[3]);//the last parameter is the path to the vocabulary file of extension .fbow

        ros::init(argc, argv, "my_tf2_broadcaster");
        ros::NodeHandle node;


        cv::Mat inputImage;
        char keyPressed=0;
        while( VideoIn.grab() && keyPressed!=27){//keyPressed ==27 is esc
            VideoIn.retrieve(inputImage);
            int frameNumber=VideoIn.get(cv::CAP_PROP_POS_FRAMES);
            cv::Mat posef2g= SLAM.process(inputImage,cameraParams,frameNumber);
            if(posef2g.empty()){
                std::cerr<<"Frame "<<frameNumber<<" pose not found"<<std::endl;
            }
            else{
                std::cout << posef2g << std::endl;
            }

            //draw a mininimal interface in an opencv window
            keyPressed=MapViwer.show(map,inputImage,posef2g);
            poseCallback(posef2g);
        }
        //now,  save the map
        map->saveToFile(argv[4]);
    } catch (std::exception &ex) {
        std::cout<<ex.what()<<std::endl;
    }


    ros::spin();
    return 0;
}