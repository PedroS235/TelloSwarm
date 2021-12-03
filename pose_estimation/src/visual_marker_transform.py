#!/usr/bin/python3
import ros_numpy
import numpy as np
import rospy
from tf.transformations import rotation_from_matrix
import tf2_ros
import tf

from geometry_msgs.msg import TransformStamped, Transform

def static_transformation():
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "R"
    static_transformStamped.child_frame_id = "Cw" 

    static_transformStamped.transform.translation.x = float(0.04)
    static_transformStamped.transform.translation.y = float(0.0)
    static_transformStamped.transform.translation.z = float(0.0)
    quat = tf.transformations.quaternion_from_euler(
                float(0),float(0),float(0))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    static_broadcaster.sendTransform(static_transformStamped)

def static_lookup(buffer, source, target):
    t_BA = buffer.lookup_transform(source, target, rospy.Time())
    return np.array(ros_numpy.numpify(t_BA.transform))

if __name__ == '__main__':

    rospy.init_node("Visual_marker_in_WO")
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            Pc_vm = buf.lookup_transform('WO', 'VMd_0', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
            
        H_WM = static_lookup(buf, 'W', 'VMw_0')

        H_MD = ros_numpy.numpify(Pc_vm.transform)
        H_N = np.matmul(np.linalg.inv(H_WM), H_MD)

        ts = TransformStamped(transform=ros_numpy.msgify(Transform, H_N))
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = "W"
        ts.child_frame_id = "WO"

        ts1 = TransformStamped(transform=Pc_vm.transform)
        ts1.header.stamp = rospy.Time.now()
        ts1.header.frame_id = "WO"
        ts1.child_frame_id = "VMwo"

        broadcaster.sendTransform(ts)
        broadcaster.sendTransform(ts1)
        rate.sleep()