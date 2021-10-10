#include <iostream>
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define log(x) std::cout << x << std::endl;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

private:
    char **args;
public:
  ImageConverter(char **pargv)
    : it_(nh_)
  {
    args = pargv;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

    // image callback
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(27);

	try {
        ucoslam::UcoSlam SLAM;//The main class
        ucoslam::Params UcoSlamParams;//processing parameters
        ucoslam::ImageParams cameraParams;//camera parameters
        ucoslam::MapViewer MapViwer;//Viewer to see the 3D map and the input images

        //creates an empty map
        std::shared_ptr<ucoslam::Map> map=std::make_shared<ucoslam::Map>();
        //read camera params
        cameraParams.readFromXMLFile(args[1]);
        //set the slam params for Kitti using orb descriptor
        UcoSlamParams.runSequential=true;//run in sequential mode to avoid skipping frames
        UcoSlamParams.detectMarkers=false;//no markers in this example.

        //Start UcoSlam
        SLAM.setParams(map,UcoSlamParams, args[2]);//the last parameter is the path to the vocabulary file of extension .fbow

        char keyPressed=0;
        int frameNumber=1;
        while(keyPressed!=27){//keyPressed ==27 is esc
            cv::Mat posef2g= SLAM.process(cv_ptr->image,cameraParams,frameNumber);
            if(posef2g.empty()){
                std::cerr<<"Frame "<<frameNumber<<" pose not found"<<std::endl;
            }
            else
                std::cerr<<"Frame "<<frameNumber<<" pose "<<posef2g<<std::endl;
            //draw a mininimal interface in an opencv window
            keyPressed=MapViwer.show(map,cv_ptr->image,posef2g);
            frameNumber++;
        }
        //now,  save the map
        map->saveToFile(args[3]);
    } catch (std::exception &ex) {
        std::cout<<ex.what()<<std::endl;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic(argv);
  ros::spin();
  return 0;
}