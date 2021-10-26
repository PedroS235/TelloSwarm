#include <tello_slam_ros/tello_slam_detector.h>

TelloSlamRos::TelloSlamRos(int argc, char **argv){
    // - Initialize ROS
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    imageTransport = new image_transport::ImageTransport(nh);
    tfTransformBroadcaster = new tf2_ros::TransformBroadcaster();

    frameNumber = 0;

    //return;
}

TelloSlamRos::~TelloSlamRos(){
    if(imageTransport) delete imageTransport;
    if(tfTransformBroadcaster) delete tfTransformBroadcaster;
    close();
    return;
}

void TelloSlamRos::close(){
    return;
}

int TelloSlamRos::openRos(){
    ros::NodeHandle nh;

    // - Image subscriber
    // - hard coded
    imageSub = imageTransport -> subscribe("/camera/image_raw", 1, &TelloSlamRos::imageCallback, this);
    return 0;
}

int TelloSlamRos::open(int argc, char **argv){
    configureUcoslam(argc, argv);

    openRos();
    
    return 0;
}


void TelloSlamRos::imageCallback(const sensor_msgs::ImageConstPtr& msg){
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
    std::cout << cameraPose << std::endl;

    // - Send the pose to TF2
    geometry_msgs::TransformStamped transformedStamped;

    transformedStamped.header.stamp = ros::Time::now();
    transformedStamped.header.frame_id = "WorldMap";
    transformedStamped.child_frame_id = "TelloSlamDectector";
    if(!cameraPose.empty()){
        transformedStamped.transform = cameraPose2Tf();
        tfTransformBroadcaster -> sendTransform(transformedStamped);
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
    std::cout << "This line executed" << std::endl;

    cv::Mat rot(3, 3, CV_32FC1);
    cv::Rodrigues(RVec, rot);

    // - Converting the rotation matrix to a TF2 matrix
    tf2::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                       rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                       rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

    // - Converting the rotation matrix to a TF2 matrix
    tf2::Vector3 tf_orig(TVec.at<float>(0,0), TVec.at<float>(0,1), TVec.at<float>(0,2));

    tf2::Transform tf2_transform = tf2::Transform(tf_rot, tf_orig);
    geometry_msgs::Transform transform=tf2::toMsg(tf2_transform);

    return transform;
}

void TelloSlamRos::configureUcoslam(int argc, char **argv){
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

int TelloSlamRos::run(){
    try{
        ros::spin();
    }catch(std::exception& e){
        std::cout << "[ROSNODE] Exception: " << e.what() << std::endl;
    }

    return 0;
}

void TelloSlamRos::saveMapFile(){
    worldMap -> saveToFile(worldMapName);
}
