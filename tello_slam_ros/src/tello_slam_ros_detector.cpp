#include <tello_slam_ros/tello_slam_ros_detector.h>

TelloSlamRos::TelloSlamRos(int argc, char **argv){
    // - Initialize ROS
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    imageTransport = new image_transport::ImageTransport(nh);
    tfTransformBroadcaster = new tf2_ros::TransformBroadcaster();

    frameNumber = 0;

    return;
}

TelloSlamRos::~TelloSlamRos(){
    if(imageTransport) delete imageTransport;
    if(tfTransformBroadcaster) delete tfTransformBroadcaster;
    return;
}

int TelloSlamRos::openRos(){
    ros::NodeHandle nh;

    // - Camera info subscriber 
    cameraInfoSub = nh.subscribe(cameraInfoTopicName, 1, &TelloSlamRos::cameraInfoCallback, this);

    // - Image subscriber
    imageSub = imageTransport -> subscribe(imageTopicName, 1, &TelloSlamRos::imageCallback, this);
    return 0;
}

int TelloSlamRos::open(){
    readParameters();
    
    configureUcoslam();

    openRos();
    
    return 0;
}

void TelloSlamRos::readParameters(){
    std::cout << "======================================" << std::endl;
    std::cout << "|         Setting parameters         |" << std::endl;
    std::cout << "======================================" << std::endl;

    // - Topic names
    ros::param::param<std::string>("~image_topic_name", imageTopicName, "/camera/image_raw");
    std::cout << " -> Image topic name: " << imageTopicName << std::endl;

    ros::param::param<std::string>("~camera_info_topic_name", cameraInfoTopicName, "camera/camera_info");
    std::cout << " -> Camera infot topic name: " << cameraInfoTopicName << std::endl;

    // - Ucoslam params
    ros::param::param<bool>("~run_sequential", runSequential, true);
    std::cout << " -> Running sequential = " << runSequential << std::endl;

    ros::param::param<bool>("~detect_markers", detectMarkers, true);
    std::cout << " -> Detecting markers = " << detectMarkers << std::endl;

    ros::param::param<int>("~arudo_id_global_reference", arucoIdGlobalReference, -1);
    std::cout << " -> The aruco marker " << arucoIdGlobalReference << "will be set as the middle of the world." << std::endl;

    // - File names
    ros::param::param<std::string>("~camera_calibration_file_name", cameraCalibrationFileName, "");
    std::cout << " -> Camera calibration file name: " << cameraCalibrationFileName << std::endl;

    ros::param::param<std::string>("~vocabulary_file_name", vocabularyFileName, "/home/pedros/ucoslam/3rdparty/vocabularies/orb.fbow");
    std::cout << " -> Vocabulary file name: " << vocabularyFileName << std::endl;

    ros::param::param<std::string>("~input_world_map_file_name", inputWorldMapFileName, "");
    std::cout << " -> Input world map file name: " << inputWorldMapFileName << std::endl;

    ros::param::param<std::string>("~output_world_map_file_name", outputWorldMapFileName, "world.map");
    std::cout << " -> Output world map file name: " << outputWorldMapFileName << std::endl;

    ros::param::param<std::string>("~tello_slam_detector_frame_name", telloSlamDetectorFrameName, "world.map");
    std::cout << " -> Output world map file name: " << outputWorldMapFileName << std::endl;

    std::cout << "======================================" << std::endl;
    std::cout << "|     Finished setting parameters    |" << std::endl;
    std::cout << "======================================" << std::endl;
}

void TelloSlamRos::cameraInfoCallback(const sensor_msgs::CameraInfo &msg){
    cv::Size camSize(msg.width, msg.height);
    cv::Mat cameraMatrix(3, 3, CV_32FC1);
    cv::Mat distortionCoefficients(5, 1, CV_32FC1);

    // - Camera matrix
    for(int col = 0; col<msg.K.size(); ++col)
        cameraMatrix.at<float>(col%3, col-(col%3)*3) = msg.K[col];
    
    // - Distortion coefficient matrix
    for(int col = 0; col<msg.K.size(); ++col)
        distortionCoefficients.at<float>(col, 0) = msg.D[col];
    ucoslamCameraParams.CamSize = camSize;
    ucoslamCameraParams.CameraMatrix = cameraMatrix;
    ucoslamCameraParams.Distorsion = distortionCoefficients;
    cameraInfoSub.shutdown();
    cameraInfoReceived = true;

    std::cout << "[INFO] - Camera calibration parameters received" << std::endl;
}

void TelloSlamRos::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    if(cameraInfoReceived){
        // - Transform image to OpenCV compatible
        try{
            cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        imageMat = cvImage -> image;
        
        // - Calling UcoSLAM
        cameraPose = ucoslam.process(imageMat, ucoslamCameraParams, frameNumber);
        frameNumber++;

        // - Send the pose to TF2
        geometry_msgs::TransformStamped transformedStamped;

        transformedStamped.header.stamp = ros::Time::now();
        transformedStamped.header.frame_id = telloSlamDetectorFrameName;
        if(!cameraPose.empty()){
            transformedStamped.transform = cameraPose2Tf();
            tfTransformBroadcaster -> sendTransform(transformedStamped);
        }
    }
}

geometry_msgs::Transform TelloSlamRos::cameraPose2Tf(){
    cv::Mat RVec(3, 3, CV_32FC1);
    cv::Mat TVec(1, 3, CV_32FC1);

    // - Creating the rotating matrix
    for(int row = 0; row<2; row++){
        for(int col = 0; col<2; col++){
            RVec.at<float>(row, col) = cameraPose.at<float>(row, col);
        }
    } 
    // - Creating the Translation matrix
    for(int row  = 0; row<3; row++)
        TVec.at<float>(0, row) = cameraPose.at<float>(row, 3);

    //cv::Mat rot(3, 3, CV_32FC1);
    //cv::Rodrigues(RVec, rot);

    // - Converting the rotation matrix to a TF2 matrix
    tf2::Matrix3x3 tf_rot(RVec.at<float>(0,0), RVec.at<float>(0,1), RVec.at<float>(0,2),
                       RVec.at<float>(1,0), RVec.at<float>(1,1), RVec.at<float>(1,2),
                       RVec.at<float>(2,0), RVec.at<float>(2,1), RVec.at<float>(2,2));

    // - Converting the rotation matrix to a TF2 matrix
    tf2::Vector3 tf_orig(TVec.at<float>(0,0), TVec.at<float>(0,1), TVec.at<float>(0,2));

    tf2::Transform tf2_transform = tf2::Transform(tf_rot, tf_orig);
    geometry_msgs::Transform transform=tf2::toMsg(tf2_transform);

    return transform;
}

void TelloSlamRos::configureUcoslam(){
    // - Load camera parameters by a file
    if(cameraCalibrationFileName != "")
        ucoslamCameraParams.readFromXMLFile(cameraCalibrationFileName);

    // -= Load the world map
    if(inputWorldMapFileName != ""){
        if(arucoIdGlobalReference >= 0)
            setCenterReferenceOfMap();
        loadMapFromFile();
    }

    // - Set Ucoslam parameters
    ucoslam.setParams(worldMap, ucoslamParams, vocabularyFileName);

    ucoslamParams.runSequential = runSequential; // - when in sequential, it will not drop frames (good to create maps)
    ucoslamParams.detectMarkers = detectMarkers;
}

int TelloSlamRos::run(){
    try{
        ros::spin();
    }catch(std::exception& e){
        std::cout << "[ROSNODE] Exception: " << e.what() << std::endl;
    }

    return 0;
}

void TelloSlamRos::saveMapFile(){
    worldMap -> saveToFile(outputWorldMapFileName);
}

void TelloSlamRos::loadMapFromFile(){
    worldMap -> readFromFile(inputWorldMapFileName);
}

void TelloSlamRos::setCenterReferenceOfMap(){
    ucoslam::Map ucoslamMap;
    ucoslamMap.readFromFile(inputWorldMapFileName);
    ucoslamMap.centerRefSystemInMarker(arucoIdGlobalReference);
    ucoslamMap.saveToFile(inputWorldMapFileName);
}