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
    ucoslam::UcoSlam slam;
    ucoslam::Params ucoslamParams;
    ucoslam::ImageParams cameraParams;
    ucoslam::MapViewer mapViewer;
    std::shared_ptr<ucoslam::Map> map = std::make_shared<ucoslam::Map>();

private:
    int frameCounter;

public:
    ImageSubsciber(char **args) : it(nh){
        frameCounter = 1;
        cameraParams.readFromXMLFile(args[1]);
        ucoslamParams.runSequential = true;
        ucoslamParams.detectMarkers = false;
        slam.setParams(map, ucoslamParams, args[2]);

        image_sub = it.subscribe("/camera/image_raw", 1, &ImageSubsciber::imageCallBack, this);
    }

    /*
        TODO:
        create another file, where there will be all of the slam related
        also see if there is a way to get the image and pass it to a topic with the points
        also publish the location
    */

    void imageCallBack(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cvImage;

        try{
            cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat currFrame = cvImage->image; 
        cv::Mat posef2g = slam.process(currFrame, cameraParams, frameCounter);
        
        if(posef2g.empty()) std::cerr << "Frame " << frameCounter << "pose not found" << std::endl;
        else std::cerr << "Frame " << frameCounter << "pose " << posef2g << std::endl;

        mapViewer.show(map, currFrame, posef2g);
        frameCounter++;
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_subscriber");
    ImageSubsciber is(argv);
    ros::spin();
    return 0;
}