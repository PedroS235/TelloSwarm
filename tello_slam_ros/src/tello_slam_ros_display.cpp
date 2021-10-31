#include <tello_slam_ros/tello_slam_ros_display.h>

TelloSlamDisplayRos::TelloSlamDisplayRos(int argc, char **argv){
    // - Initialize ROS
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    imageTransport = new image_transport::ImageTransport(nh);

    frameNumber = 0;

    //return;
}

TelloSlamDisplayRos::~TelloSlamDisplayRos(){
    if(imageTransport) delete imageTransport;
    close();
    return;
}

void TelloSlamDisplayRos::close(){
    return;
}

int TelloSlamDisplayRos::openRos(){
    ros::NodeHandle nh;

    // - Image subscriber
    // - hard coded
    imageSub = imageTransport -> subscribe(imageTopicName, 1, &TelloSlamDisplayRos::imageCallback, this);
    cameraInfoSub = nh.subscribe(camera_info_topic_name, 1, &TelloSlamDisplayRos::cameraInfoCallback, this);
    outputImagePub = imageTransport -> advertise(outputImageTopicName, 1);
    return 0;
}

int TelloSlamDisplayRos::open(){
    readParameters();

    configureUcoslam();

    openRos();
    
    return 0;
}

void TelloSlamDisplayRos::readParameters(){
    std::cout << "======================================" << std::endl;
    std::cout << "[INFO] - Setting parameters" << std::endl;
    // - Topic names
    ros::param::param<std::string>("~image_topic_name", imageTopicName, "/camera/image_raw");
    std::cout << " -> Image topic name: " << imageTopicName << std::endl;

    ros::param::param<std::string>("~camera_info_topic_name", camera_info_topic_name, "camera/camera_info");
    std::cout << " -> Camera infot topic name: " << camera_info_topic_name << std::endl;

    ros::param::param<std::string>("~output_image_topic_name", outputImageTopicName, "tello_slam/tello_slam_observation_image/image_raw");
    std::cout << " -> Outpuc image topic name: " << outputImageTopicName << std::endl;

    // - Ucoslam params
    ros::param::param<bool>("~runSequential", runSequential, true);
    std::cout << " -> Running sequential = " << runSequential << std::endl;

    ros::param::param<bool>("~detectMarkers", detectMarkers, true);
    std::cout << " -> Detecting markers = " << detectMarkers << std::endl;

    ros::param::param<bool>("~showDisplay", showDisplay, true);
    std::cout << " -> Show display = " << showDisplay << std::endl;

    // - Files names
    ros::param::param<std::string>("~cameraCalibrationName", cameraCalibrationName, "");
    std::cout << " -> Camera calibration file name: " << cameraCalibrationName << std::endl;

    ros::param::param<std::string>("~vocabularyFileName", vocabularyFileName, "/home/pedros/ucoslam/3rdparty/vocabularies/orb.fbow");
    std::cout << " -> Vocabulary file name: " << vocabularyFileName << std::endl;

    ros::param::param<std::string>("~worldMapName", worldMapName, "world.map");
    std::cout << " -> World map file name: " << worldMapName << std::endl;

    std::cout << "[INFO] - Finished setting parameters" << std::endl;
    std::cout << "======================================" << std::endl;
}

void TelloSlamDisplayRos::cameraInfoCallback(const sensor_msgs::CameraInfo &msg){
    cv::Size camSize(msg.width, msg.height);
    cv::Mat cameraMatrix(3, 3, CV_32FC1);
    cv::Mat distortionCoefficients(5, 1, CV_32FC1);

    // - Camera matrix
    for(int col = 0; col<msg.K.size(); ++col)
        cameraMatrix.at<float>(col%3, col-(col%3)*3) = msg.K[col];
    
    // - Distortion coefficient matrix
    for(int col = 0; col<msg.K.size(); ++col)
        distortionCoefficients.at<float>(col, 0) = msg.D[col];

    // - Setting the camera parameters
    ucoslamCameraParams.CamSize = camSize;
    ucoslamCameraParams.CameraMatrix = cameraMatrix;
    ucoslamCameraParams.Distorsion = distortionCoefficients;
    cameraInfoSub.shutdown();
    cameraInfoReceived = true;
    std::cout << "Camera calibration parameters received" << std::endl;
}

void TelloSlamDisplayRos::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    if(cameraInfoReceived){
        // - Transform image to OpenCV compatible
        try{
            cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cameraPose = ucoslam.process(cvImage->image, ucoslamCameraParams, frameNumber);
        frameNumber++;
        if(showDisplay){
            int keyPressed = 0;
            keyPressed = mapViewer.show(worldMap, cvImage -> image, cameraPose);
            if(keyPressed == 27)
                ros::shutdown(); 
        }
        outputImagePub.publish(cvImage -> toImageMsg());
    }
}

void TelloSlamDisplayRos::configureUcoslam(){
    if(cameraCalibrationName != ""){
        // - Load camera parameters
        ucoslamCameraParams.readFromXMLFile(cameraCalibrationName);
    }

    // - Set Ucoslam parameters
    ucoslam.setParams(worldMap, ucoslamParams, vocabularyFileName);

    ucoslamParams.runSequential = runSequential;
    ucoslamParams.detectMarkers = detectMarkers;
}

int TelloSlamDisplayRos::run(){
    try{
        ros::spin();
    }catch(std::exception& e){
        std::cout << "[ROSNODE] Exception: " << e.what() << std::endl;
    }

    return 0;
}

void TelloSlamDisplayRos::saveMapFile(){
    worldMap -> saveToFile(worldMapName);
}
