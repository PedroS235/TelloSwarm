#!/usr/bin/python3

import rospy
from sensor_msgs.msg import CameraInfo
import yaml
import argparse

def publisher(camera_info):
    """
        This is a publisher that will publish the camera parameters of the camera
    """
    rospy.loginfo("Initiating the camera calibrator node ...")
    publisher_node = rospy.Publisher("camera/camera_info", CameraInfo, queue_size=10)
    rospy.init_node("camera_calibrator")
    rospy.loginfo("Camera calibrator node successfully launched")
    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        publisher_node.publish(camera_info)
        rospy.loginfo("Parameters published")
        rate.sleep()


def retrieve_data(filename):
    # Load data from file
    with open(filename, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    camera_info_msg.R = calib_data["rotation"]["data"]
    camera_info_msg.P = calib_data["p"]["data"]

    return camera_info_msg

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', required=True, type=str)
    args, unknown = parser.parse_known_args()
    camera_info_msg = retrieve_data(args.file)
    publisher(camera_info_msg)
