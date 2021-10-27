#include <tello_slam_ros/tello_slam_display.h>

TelloSlamDisplayRos::TelloSlamDisplayRos(int argc, char **argv){
    // - Initialize ROS
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    imageTransport = new image_transport::ImageTransport(nh);
    tfTransformBroadcaster = new tf2_ros::TransformBroadcaster();

    frameNumber = 0;

    //return;
}

TelloSlamDisplayRos::~TelloSlamDisplayRos(){
    if(imageTransport) delete imageTransport;
    if(tfTransformBroadcaster) delete tfTransformBroadcaster;
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
    imageSub = imageTransport -> subscribe("/camera/image_raw", 1, &TelloSlamDisplayRos::imageCallback, this);
    cameraInfoSub = nh.subscribe(camera_info_topic_name, 1, &TelloSlamDisplayRos::cameraInfoCallback, this);
    outputImagePub = imageTransport -> advertise(outputImageTopicName, 1);
    return 0;
}

int TelloSlamDisplayRos::open(int argc, char **argv){
    configureUcoslam(argc, argv);

    openRos();
    
    return 0;
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
    ucoslamCameraParams.CamSize = camSize;
    ucoslamCameraParams.CameraMatrix = cameraMatrix;
    ucoslamCameraParams.Distorsion = distortionCoefficients;
    cameraInfoSub.shutdown();
    std::cout << "Camera calibration parameters received" << std::endl;
}

void TelloSlamDisplayRos::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    // - Transform image to OpenCV compatible
    try{
        cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imageMat = cvImage -> image;

    cameraPose = ucoslam.process(imageMat, ucoslamCameraParams, frameNumber);
    frameNumber++;
    ucoslam::MapViewer mapviewer;
    cv::Mat imageDrawn = mapviewer.draw(worldMap, imageMat, cameraPose, "", frameNumber);

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = imageDrawn;
    outputImagePub.publish(out_msg.toImageMsg());
}

void TelloSlamDisplayRos::configureUcoslam(int argc, char **argv){
    if(argc<3) throw std::runtime_error("[ERROR] - Usage: cameraparameters.yml vocabulary.fbow (worldmap.map)");
    cameraCalibrationName = argv[1];
    vocabularyFileName = argv[2];
    if(argc == 4){ 
        worldMapName = argv[3];
        worldMap -> readFromFile(worldMapName); // Not sure if this works
    }else{
        worldMapName = "world.map";
    }

    // - Load camera parameters
    ucoslamCameraParams.readFromXMLFile(cameraCalibrationName);

    // - Set Ucoslam parameters
    ucoslam.setParams(worldMap, ucoslamParams, vocabularyFileName);

    runSequential = true;
    detectMarkers = true;
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