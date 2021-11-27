#!/usr/bin/python3

import rospy
import tf2_ros
import numpy as np
import ros_numpy

from geometry_msgs.msg import Transform, TransformStamped

# Notes on variable naming:
#  t_BA: transform (ROS msg type) from source B to target A
#  H_BA: pose of frame A in coordinates of frame B (4x4 HTM)
#  both are numerically equivalent (if represented in the same format)

def static_lookup(buffer, source, target):
    # timeout of 5s: give time to other node to start
    t_BA = buffer.lookup_transform(source, target, rospy.Time(), rospy.Duration(5))
    H_BA = np.array(ros_numpy.numpify(t_BA.transform))
    return H_BA

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--detector', help='Name of detector frame', default='aruco_detector')
    parser.add_argument('-r', '--robot', help='Name of robot (drone) frame', default='tello')
    parser.add_argument('-w', '--world', help='Name of the world', default='world_odometry')
    parser.add_argument('-c', '--camera', help='Name of true marker frame', default='slam_vision')
    args, unknown = parser.parse_known_args()

    name = "%s_in_%s_via_%s" % (args.robot, args.world, args.camera)
    rospy.init_node(name)
    rospy.loginfo("Started node %s" % name)
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    broadcaster = tf2_ros.TransformBroadcaster()
    # this can and should fail if those transforms are not found
    H_DR = static_lookup(buf, args.detector, args.robot)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            t_MD = buf.lookup_transform(args.camera, args.detector, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        H_MD = ros_numpy.numpify(t_MD.transform)
        H_WR = np.dot(H_MD, H_DR)
        t_WR = TransformStamped(transform=ros_numpy.msgify(Transform, H_WR))
        t_WR.header.stamp = t_MD.header.stamp
        t_WR.header.frame_id = args.world
        t_WR.child_frame_id = args.robot
        broadcaster.sendTransform(t_WR)
        #t_WR.child_frame_id = "%s_via_%s" % (args.robot, args.marker)
        #broadcaster.sendTransform(t_WR)
        rate.sleep()

