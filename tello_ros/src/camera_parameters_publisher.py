#!/usr/bin/python3

import rospy
from sensor_msgs import CameraInfo
import yaml
import argparse

def publisher(camera_info):
    """
        This is a publisher that will publish the camera parameters of the camera
    """
    rospy.INFO("Initiating the camera calibrator node ...")
    publisher_node = rospy.Publisher("camera/camera_info", CameraInfo, queue_size=10)
    rospy.init_node("camera_calibrator")
    rospy.INFO("Camera calibrator node successfully launched")
    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        publisher_node.publish(camera_info)
        rospy.INFO("Parameters published")
        rate.sleep()


def retrieve_data(filename):
    # Load data from file
    with open(filename, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    return camera_info_msg

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file')
    args = parser.parse_args()
    camera_info_msg = retrieve_data(args.file)
    publisher(camera_info_msg)