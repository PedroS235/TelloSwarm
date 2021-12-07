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


"""
####################
# Name conventions #
####################

- Px_y -> transform from x to y (x = parent_frame_id | y = child_frame_id)

###########
#TF FRAMES#
###########

- W -> world
- WO -> world odometry
- R -> robot
- Cwo -> camera (child of world odometry)
- Cw -> camera (child of robot)
- VMd -> visual marker being detected
- VMwo -> visual marker detected in world odometry 
- VMw -> visual marker in world (static) 

[Note]: Cwo=Cw and VMd=VMwo=VMw (To be able to have a correct TF TREE!)
"""

def marker_detection(buffer, rate, broadcaster, static_broadcaster, tf_frames):
    """
        @params:
            - buffer: tf2_ros buffer
            - rate: rospy rate
            - brodcaster: broadcaster to send the transforms 
            - static_broadcaster: used to make the transfrom between the world and world odometry
            - tf_frames: a dictionary with all the names of the tf frames
    """

    try: 
        Pwo_vm = get_transform_in_np_array(buffer, tf_frames.get('WO'), tf_frames.get('VMd')) 
        Pw_vmw = get_transform_in_np_array(buffer, tf_frames.get('W'), tf_frames.get('VMw'))
        Pw_wo = np.dot(Pw_vmw, np.linalg.inv(Pwo_vm))

        transformStamped = TransformStamped(transform = ros_np.msgify(Transform, Pw_wo))
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = tf_frames.get('W')
        transformStamped.child_frame_id = tf_frames.get('WO')

        static_broadcaster.sendTransform(transformStamped)

        transformStamped = TransformStamped(transform = ros_np.msgify(Transform, Pwo_vm))
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = tf_frames.get('WO')
        transformStamped.child_frame_id = tf_frames.get('VMwo')

        broadcaster.sendTransform(transformStamped)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()


def robot_estimation(buffer, rate, broadcaster, tf_frames):
    """
        @params:
            - buffer: tf2_ros buffer
            - rate: rospy rate
            - brodcaster: broadcaster to send the transforms 
            - tf_frames: a dictionary with all the names of the tf frames
    """

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
    return np.array(ros_np.numpify(buffer.lookup_transform(parent, child, rospy.Time()).transform))

def get_tf_frames_from_params():
    W = rospy.get_param('world_frame_name', default='W')
    WO = rospy.get_param('world_odometry_frame_name', default='WO')
    Cw = rospy.get_param('camera_world_frame_name', default='Cw')
    Cwo = rospy.get_param('camera_world_odometry_frame_name', default='Cwo')
    VMd = rospy.get_param('marker_detected_frame_name', default='VMd_0')
    VMw = rospy.get_param('marker_in_world_frame_name', default='VMw_0')
    VMwo = rospy.get_param('marker_in_world_odometry_frame_name', default='VMwo_0')
    R = rospy.get_param('robot_frame_name', default='R')
    frames = {
        'W': W,
        'WO': WO,
        'R': R,
        'VMd': VMd,
        'VMwo': VMwo, 
        'VMw': VMw,
        'Cwo': Cwo,
        'Cw': Cw
    }
    return frames
    

def main():
    tf_frames = get_tf_frames_from_params()

    rospy.init_node("Robot_localisation_estimation")

    buffer = tf2_ros.Buffer()
    broadcaster = tf2_ros.TransformBroadcaster()
    listener = tf2_ros.TransformListener(buffer)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        marker_detection(buffer, rate, broadcaster, static_broadcaster, tf_frames)
        robot_estimation(buffer, rate, broadcaster, tf_frames)
        rate.sleep()


if __name__ == '__main__':
    main()
