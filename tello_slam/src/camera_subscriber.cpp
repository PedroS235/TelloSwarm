#include <ros/ros.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

void displayImage(const sensor_msgs::Image::ConstPtr data){
    cv::VideoCapture videoIn(0);

    if(!videoIn.isOpened()){
       ROS_INFO("Cannot open the video");
    } 

    std::string window_name = "video clip";

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);

    while(true){
        cv::Mat frame;
        bool bSuccess = videoIn.read(frame);

        if(!bSuccess){
            ROS_INFO("found the end of the video");
            break;
        }
        ROS_INFO("I am executing");
        imshow(window_name, frame);

        if(cv::waitKey(10) == 27){
            ROS_INFO("ESC key is pressed by user. Stopping the video");
            break;
        }
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_subscriber");
    ROS_INFO("Initiating camera subscriber");
    ros::NodeHandle nh;

    ros::Subscriber camera_topic = nh.subscribe("camera/image_raw", 2, displayImage);

    ros::spin();
}