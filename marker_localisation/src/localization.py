#!/usr/bin/python3

import rospy
import tf2_ros
import numpy as np
import ros_numpy
import math

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
    parser.add_argument('-d', '--detector', help='Name of detector frame', default='aruco_detector_0')
    parser.add_argument('-r', '--robot', help='Name of robot (drone) frame', default='tello_0')
    parser.add_argument('-w', '--world', help='Name of the world', default='world')
    parser.add_argument('-e', '--marker-estimation', help='Name of estimated marker frame', default='aruco_marker_0')
    parser.add_argument('-m', '--marker', help='Name of true marker frame', default='world_aruco_marker_0')
    #parser.add_argument('-s', '--slam', help='Name of slam detector frame', default='slam_detector_0')
    args, unknown = parser.parse_known_args()

    name = f'{args.robot}_via_slam'
    rospy.init_node(name)
    rospy.loginfo("Started node %s" % name)
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    broadcaster = tf2_ros.TransformBroadcaster()
    # this can and should fail if those transforms are not found
    Pwo_c = static_lookup(buf, 'slam_detector_0', 'slam_vision_0')

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            Pwo_c = buf.lookup_transform('slam_detector_0', 'slam_vision_0', rospy.Time())
            Pc_vm = buf.lookup_transform(args.marker_estimation, args.detector, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        H_slam = ros_numpy.numpify(Pwo_c.transform)
        H_marker = ros_numpy.numpify(Pc_vm.transform)

        slam_x = H_slam[0][3]
        slam_y = H_slam[1][3]
        slam_z = H_slam[2][3]

        marker_x = H_marker[0][3]
        marker_y = H_marker[1][3]
        marker_z = H_marker[2][3]
        
        delta_x = abs(slam_x - marker_x)
        delta_y = abs(slam_y - marker_y)
        delta_z = abs(slam_z - marker_z)

        rospy.loginfo(f'x: {delta_x} | y: {delta_y} | z: {delta_z}')
       # H_WR = np.dot(np.dot(H_WM, H_MD), H_DR)
       # t_WR = TransformStamped(transform=ros_numpy.msgify(Transform, H_WR))
       # t_WR.header.stamp = t_MD.header.stamp
       # t_WR.header.frame_id = args.world
       # t_WR.child_frame_id = args.robot
       # broadcaster.sendTransform(t_WR)
        #t_WR.child_frame_id = "%s_via_%s" % (args.robot, args.marker)
        #broadcaster.sendTransform(t_WR)
        rate.sleep()

