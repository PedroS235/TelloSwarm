#!/usr/bin/python3

# - Math 
import numpy as np
from numpy.lib.financial import rate

# - ROS
import rospy
import ros_numpy as ros_np

# - TF
import tf2_ros

# - ROS messages
from geometry_msgs.msg import TransformStamped, Transform

def marker_detection(buffer, rate, broadcaster, static_broadcaster, tf_frames):
    try: 
        Pwo_vm = get_transform_in_np_array(buffer, tf_frames.get('WO'), tf_frames.get('VMd')) 
        Pw_vmw = get_transform_in_np_array(buffer, tf_frames.get('W'), tf_frames.get('VMw'))
        Pw_wo = np.dot(Pw_vmw, np.linalg.inv(Pwo_vm))

        transformStamped = TransformStamped(transform = ros_numpy.msgify(Transform, Pw_wo))
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = tf_frames.get('W')
        transformStamped.child_frame_id = tf_frames.get('WO')

        static_broadcaster.sendTransform(transformStamped)

        transformStamped = TransformStamped(transform = ros_numpy.msgify(Transform, Pwo_vm))
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = tf_frames.get('WO')
        transformStamped.child_frame_id = tf_frames.get('VMwo')

        broadcaster.sendTransform(transformStamped)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()


def robot_estimation(buffer, rate, broadcaster, tf_frames):
    try: 
        Pwo_cwo = get_transform_in_np_array(buffer, tf_frames.get('WO'), tf_frames.get('Cwo'))
        Pr_cw = get_transform_in_np_array(buffer, tf_frames.get('R'), tf_frames.get('Cw'))
        Pw_cw = get_transform_in_np_array(buffer, tf_frames.get('W'), tf_frames.get('WO'))
        Pw_r = np.dot(np.dot(Pw_cw, Pwo_cwo), np.linalg.inv(Pr_cw))

        transformStamped = TransformStamped(transform = ros_np.msgify(Transform, Pw_r))
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = tf_frames.get('W')
        transformStamped.child_frame_id = tf_frames.get('R')

        broadcaster.sendTransform(transformStamped)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()

def get_transform_in_np_array(buffer, parent, child):
    """
        @params:
            - buffer: tf2 buffer
            - parent: parent frame
            - child: child frame

        @return: transfrom between parent and child in a numpy array
    """
    return np.array(ros_np.numpify(buffer.lookup_transform(parent, child, rospy.Time())))

def main():
    tf_frames = {
        'W': 'W',
        'WO': 'WO',
        'R': 'R',
        'VMd': 'VMd_0',
        'VMwo': 'VMwo_0', 
        'VMw': 'VMw_0',
        'Cwo': 'Cwo',
        'Cw': 'Cw'
    }
    buffer = tf2_ros.Buffer()
    broadcaster = tf2_ros.TransformBroadcaster()
    listener = tf2_ros.TransformListener(buffer)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    rospy.init_node("Localisation_estimation")
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        marker_detection(buffer, rate, broadcaster, static_broadcaster, tf_frames)
        robot_estimation(buffer, rate, broadcaster, tf_frames)
        rate.sleep()


if __name__ == '__main__':
    main()
