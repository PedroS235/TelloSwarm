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
    return;
}

int TelloSlamDisplayRos::openRos(){
    ros::NodeHandle nh;

    // - Image subscriber
    imageSub = imageTransport -> subscribe(imageTopicName, 1, &TelloSlamDisplayRos::imageCallback, this);
    // - Camera info subscriber
    cameraInfoSub = nh.subscribe(cameraInfoTopicName, 1, &TelloSlamDisplayRos::cameraInfoCallback, this);
    // - Image publisher
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
    std::cout << "|         Setting parameters         |" << std::endl;
    std::cout << "======================================" << std::endl;

    // - Topic names
    ros::param::param<std::string>("~image_topic_name", imageTopicName, "/camera/image_raw");
    std::cout << " -> Image topic name: " << imageTopicName << std::endl;

    ros::param::param<std::string>("~camera_info_topic_name", cameraInfoTopicName, "camera/camera_info");
    std::cout << " -> Camera info topic name: " << cameraInfoTopicName<< std::endl;

    ros::param::param<std::string>("~output_image_topic_name", outputImageTopicName, "tello_slam/tello_slam_observation_image/image_raw");
    std::cout << " -> Outpuc image topic name: " << outputImageTopicName << std::endl;

    // - Ucoslam params
    ros::param::param<bool>("~run_sequential", runSequential, true);
    std::cout << " -> Running sequential = " << runSequential << std::endl;

    ros::param::param<bool>("~detect_markers", detectMarkers, true);
    std::cout << " -> Detecting markers = " << detectMarkers << std::endl;

    ros::param::param<bool>("~show_display", showDisplay, true);
    std::cout << " -> Show display = " << showDisplay << std::endl;

    ros::param::param<int>("~arudo_id_global_reference", arucoIdGlobalReference, -1);
    std::cout << " -> The aruco marker " << arucoIdGlobalReference << " will be set as the middle of the world." << std::endl;

    // - File names
    ros::param::param<std::string>("~camera_calibration_name", cameraCalibrationFileName, "");
    std::cout << " -> Camera calibration file name: " << cameraCalibrationFileName << std::endl;

    ros::param::param<std::string>("~vocabulary_file_name", vocabularyFileName, "/home/pedros/ucoslam/3rdparty/vocabularies/orb.fbow");
    std::cout << " -> Vocabulary file name: " << vocabularyFileName << std::endl;

    ros::param::param<std::string>("~input_world_map_file_name", inputWorldMapFileName, "");
    std::cout << " -> World map file name: " << inputWorldMapFileName << std::endl;

    ros::param::param<std::string>("~output_world_map_file_name", outputWorldMapFileName, "/home/pedros/catkin_ws/world.map");
    std::cout << " -> World map file name: " << outputWorldMapFileName << std::endl;

    ros::param::param<std::string>("~params_file_name", paramFileName, "");
    std::cout << " -> Parameters file name: " <<  paramFileName << std::endl;

    std::cout << "======================================" << std::endl;
    std::cout << "|     Finished setting parameters    |" << std::endl;
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
    std::cout << "[INFO] - Camera calibration parameters received" << std::endl;
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
    // - Load camera parameters by a file
    if(cameraCalibrationFileName != "")
        ucoslamCameraParams.readFromXMLFile(cameraCalibrationFileName);

    // -= Load the world map
    if(inputWorldMapFileName != ""){
        if(arucoIdGlobalReference >= 0)
            setCenterReferenceOfMap();
        loadMapFromFile();
    }

    if(paramFileName != ""){
        ucoslamParams.readFromYMLFile(paramFileName);
    }else{
        ucoslamParams.runSequential = runSequential; // - when in sequential, it will not drop frames (good to create maps)
        ucoslamParams.detectMarkers = detectMarkers;
    }

    // - Set Ucoslam parameters
    ucoslam.setParams(worldMap, ucoslamParams, vocabularyFileName);
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
    worldMap -> saveToFile(outputWorldMapFileName);
}

void TelloSlamDisplayRos::loadMapFromFile(){
    worldMap -> readFromFile(inputWorldMapFileName);
}

void TelloSlamDisplayRos::setCenterReferenceOfMap(){
    ucoslam::Map ucoslamMap;
    ucoslamMap.readFromFile(inputWorldMapFileName);
    ucoslamMap.centerRefSystemInMarker(arucoIdGlobalReference);
    ucoslamMap.saveToFile(inputWorldMapFileName);
}