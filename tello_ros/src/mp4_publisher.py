#!/usr/bin/python3


import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import argparse
import datetime

bridge = CvBridge()

def static_frame_gen(mp4_filename, n):
    capture = cv2.VideoCapture(mp4_filename)
    empty_frame_counter = 0
    for i in range(n):
        ret, frame = capture.read()
        print('Read frame', i, ' ' * 20, end='\r')
    capture.release()
    print('Got frame')
    while frame is not None:
        yield frame


def frame_generator(mp4_filename, loop):
    while True:
        capture = cv2.VideoCapture(mp4_filename)
        empty_frame_counter = 0
        while empty_frame_counter < 3:
            ret, frame = capture.read()
            if frame is None:
                empty_frame_counter += 1
            else:
                yield frame
        capture.release()
        if not loop:
            break
        rospy.loginfo("Looping video" + " " * 30)  # spaces to overwrite continuous display


def publisher_node(frames):
    publisher = rospy.Publisher("camera/image_raw", Image, queue_size=2)
    rospy.init_node('mp4_file')
    rate = rospy.Rate(15)
    rospy.loginfo("Started mp4_file node")

    while not rospy.is_shutdown():
        frame = next(frames)
        img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        publisher.publish(img_msg)
        now = datetime.datetime.now().time()
        print("%s: published frame %s" % (now,  object.__repr__(frame)), end='\r')
        rate.sleep()
    print()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="publish a mp4 file to the /camera/image_raw topic")
    parser.add_argument('filename')
    parser.add_argument('--loop', action='store_true', help='loop over mp4 instead of just playing it once')
    parser.add_argument('--frame', type=int, help='publish only a single frame', default=-1)
    args, unknown = parser.parse_known_args()
    if args.frame == -1:
        frames = frame_generator(args.filename, args.loop)
    else:
        frames = static_frame_gen(args.filename, args.frame)
    publisher_node(frames)
