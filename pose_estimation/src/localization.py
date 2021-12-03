#!/usr/bin/python3

import rospy
import tf2_ros
import ros_numpy
import math

from geometry_msgs.msg import Transform, TransformStamped

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--detector', help='Name of detector frame', default='slam_detector_0')
    parser.add_argument('-r', '--robot', help='Name of robot (drone) frame', default='tello_0')
    parser.add_argument('-w', '--world', help='Name of the world', default='world')
    parser.add_argument('-s', '--slam', help='Name of slam detector frame', default='slam_vision_0')
    args, unknown = parser.parse_known_args()

    name = 'world_odometry_to_world'
    rospy.init_node(name)
    rospy.loginfo("Started node %s" % name)
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            Pwo_c = buf.lookup_transform('slam_detector_0', 'slam_vision_0', rospy.Time())
            Pc_vm = buf.lookup_transform('aruco_detector_0', 'aruco_marker_0', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        t_world_odometry = ros_numpy.numpify(Pwo_c.transform)
        t_world = ros_numpy.numpify(Pc_vm.transform)

        slam_x = t_world_odometry[0][3]
        slam_y = t_world_odometry[1][3]
        slam_z = t_world_odometry[2][3]

        marker_x = t_world[0][3]
        marker_y = t_world[1][3]
        marker_z = t_world[2][3]
        
        delta_x = abs(slam_x - marker_x)
        delta_y = abs(slam_y - marker_y)
        delta_z = abs(slam_z - marker_z)

        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = args.world
        static_transformStamped.child_frame_id = 'world_odometry' 
        static_transformStamped.transform.translation.x = delta_x
        static_transformStamped.transform.translation.y = delta_y
        static_transformStamped.transform.translation.z = delta_z

        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = math.pi()/2 
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1

        broadcaster.sendTransform(static_transformStamped)
        rate.sleep()
        break
