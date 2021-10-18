#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class ImageSubsciber{
private:
    ros::NodeHandle nh;    
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

private:
    int frameCounter;
    cv_bridge::CvImagePtr cvImage;

public:
    ImageSubsciber() : it(nh){
        frameCounter = 1;
        image_sub = it.subscribe("/camera/image_raw", 1, &ImageSubsciber::imageCallBack, this);
    }

    void imageCallBack(const sensor_msgs::ImageConstPtr& msg){
        try{
            cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        frameCounter++;
        std::cout << "in image: " << frameCounter << std::endl;
    }

    cv_bridge::CvImagePtr getCurrFrame(){
        return cvImage;
    }

    int getFrameCouter(){
        return frameCounter;
    }

};
/*
int main(int argc, char **argv){
    ros::init(argc, argv, "camera_subscriber");
    ImageSubsciber is;
    ros::spin();
    return 0;
}*/