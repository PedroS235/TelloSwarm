#!/usr/bin/python3

import threading

import rospy
import ros_numpy
import cv2

from sensor_msgs.msg import Image

class VideoStreamClient:
    """
        This class captures the video from the drone and publishes it
    """

    def __init__(self, url, width=960, height=720):
        """
            Constructor of the VideoStreamClient
            url = url to capture the video
            width = width of the video stream (default value = 960)
            height = height of the video stream (default value = 720)
        """
        self.url = url
        self.width = width
        self.height = height
        self._frame = None
        self._frame_lock = threading.Lock()
        self.running = False
        self.read_capture_thread = None

    @property
    def frame(self):
        with self._frame_lock:
            f = self._frame.copy() if self._frame is not None else None
        return f

    @frame.setter
    def frame(self, new):
        with self._frame_lock:
            self._frame = new

    def start(self):
        self.read_capture_thread = threading.Thread(target=self._run)
        self.read_capture_thread.daemon = True
        self.running = True
        self.read_capture_thread.start()

    def stop(self):
        self.running = False
        self.read_capture_thread.join()

    def _run(self):
        # Creating stream capture object
        rospy.loginfo('Starting the video capture from the tello')
        video_capture = cv2.VideoCapture(self.url)
        try:
            while self.running:
                ret, self._frame  = video_capture.read()
        except Exception as e:
            print(e)


def publish(client, verbose=False, rate=30):
    """

    """
    publisher = rospy.Publisher("camera/image_raw", Image, queue_size=2)
    rospy.init_node('tello_video')
    rospy.loginfo("Started tello_video node")
    rate = rospy.Rate(RATE)
    client.start()
    while not rospy.is_shutdown():
        frame = None
        while frame is None:
            frame = client.frame
        img_msg = ros_numpy.msgify(Image, frame, encoding='rgb8')
        img_msg.header.stamp = rospy.Time.now()
        publisher.publish(img_msg)
        if VERBOSE:
            rospy.loginfo("Published video frame")
        rate.sleep()
    client.stop()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser("Publish video stream from Tello drone to /camera/image_raw")
    parser.add_argument('-u', '--url', default='udp://192.168.10.1:11111')
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('-r', '--rate', type=int, default=30)
    args, unknown = parser.parse_known_args()
    VERBOSE = args.verbose
    RATE = args.rate
    client = VideoStreamClient(args.url)
    publish(client, args.verbose, args.rate)
    client.running = True
