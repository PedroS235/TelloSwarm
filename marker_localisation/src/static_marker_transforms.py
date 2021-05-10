#!/usr/bin/python3
import subprocess
import atexit

import rospy

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-w', '--world', help='Name of detector frame', default='world')
    parser.add_argument('-m', '--markers', help='Name of ROS parameter containing marker poses in world', default='aruco_markers')
    args, unknown = parser.parse_known_args()

    # this can and should fail if the parameter does not exist
    markers = rospy.get_param(args.markers)

    children = []
    atexit.register(lambda: [child.kill() for child in children])
    for name, (x, y, z, yaw, pitch, roll) in markers.iteritems():
        x, y, z, yaw, pitch, roll = str(x), str(y), str(z), str(yaw), str(pitch), str(roll)
        publisher = subprocess.Popen([
            "rosrun", "tf2_ros", "static_transform_publisher",
            "__name:=%s_in_%s" % (name, args.world),
            x, y, z, yaw, pitch, roll, args.world, name])
        children.append(publisher)
    name = "markers_in_%s_overseer" % args.world
    rospy.init_node(name)
    rospy.loginfo("Started node %s" % name)
    rospy.spin()

