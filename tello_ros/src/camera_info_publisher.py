#!/usr/bin/python3


import rospy
from sensor_msgs.msg import CameraInfo

from datetime import datetime

def publisher():
    """
        This method contains the information to calibrate the tello camera
    """
    print("Starting calibrator node")
    publisher = rospy.Publisher("camera/camera_info", CameraInfo, queue_size=10)
    rospy.init_node("calibrator")
    rate = rospy.Rate(15)
    fixed_msg = CameraInfo(
        height=960,
        width=720,
        distortion_model="plumb_bob",
        D=[-0.013335, -0.018951, 0.000913, -0.005454, 0.000000],
        K=[
            897.137424, 0.000000, 453.939111,
            0.000000, 902.273567, 354.958881,
            0.000000, 0.000000, 1.000000],
        R=[1, 0, 0, 0, 1, 0, 0, 0, 1],
        P=[
            891.148254, 0.000000, 449.250272, 0.000000,
            0.000000, 901.238647, 355.367597, 0.000000,
            0.000000, 0.000000, 1.000000, 0.000000],
    )
    while not rospy.is_shutdown():
        publisher.publish(fixed_msg)
        now = datetime.now().time()
        print(f"{now}: published camera calibration info", end='\r')
        rate.sleep()

# -- the rest of the code will be only runned when execured from the terminal
if __name__ == '__main__':
    publisher()

