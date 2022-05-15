#!/usr/bin/python3


import math
import threading

import numpy as np

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from tello_formation.msg import GotoCommand
from tello_ros.srv import TelloCommand

class TelloController:
    """
        This class is the one that controls the Tello. (It stays between the drone and the supervisor)
    """
    def __init__(self, id):
        """
            Constructor of the class TelloController
            parameter:
                - id = unique id of the drone
        """

        self.id = id
        self.name = f'tello_{id}'

        self.pos = None
        self._mvt_lock = threading.Lock()
        self._buf = None
        self.max_localisation_delay = None
        self.do = lambda _: None

    def _goto_callback(self, msg):
        """
            This method sends the commands to the tello so that he goes to the goal
        """

        with self._mvt_lock:
            rospy.loginfo(f'Received goto {msg.target_frame_id} (t={msg.start}) at {msg.speed} m/s')
            tf = None
            try:
                tf = self._buf.lookup_transform(self.name, msg.target_frame_id, msg.start, self.max_localisation_delay)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo(f'Ignoring goto: lookup failed ({e})')
            
            # Here we could add error checking (check if the transform makes sense)
            x, y = tf.transform.translation.x, tf.transform.translation.y
            speed = max(10, min(100, int(round(msg.speed * 100))))
            distance = max(20, min(500, int(round((x ** 2 + y ** 2) ** .5 * 100))))
            angle = int(round(math.degrees(math.atan2(y, x))))

            # note: the Tello did not complain for -18.0 cw...
            turn = 'ccw' if angle > 0 else 'cw'
            angle = abs(angle)
            self.do(f'speed {speed}')
            rospy.sleep(2)
            self.do(f'{turn} {angle}')
            rospy.sleep((math.radians(angle)*0.1/speed)+2) #compute the distance of the rotation
            self.do(f'forward {distance}')
            rospy.sleep(distance / speed + 2)  # +2 to be safe
            self.do(f'land')
            rospy.loginfo('Succeed to land on the goal')
            rospy.signal_shutdown('Fininalised the work')

    def _search_marker(self):
        """
            This method searchs for the Aruco markers to locate himself
        """

        rospy.loginfo('Searching for marker (has no effect unless search_marker_enabled parameter is set to True)')
        rospy.sleep(self.max_localisation_delay)  # give more time for start-up
        while not rospy.is_shutdown():
            try:
                t = self._buf.lookup_transform(rospy.get_param('world_frame_name', default='world'), self.name, rospy.Time(0), self.max_localisation_delay)
                rospy.loginfo(f'COORDINATIONS: {t}')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                with self._mvt_lock:
                    if rospy.get_param('search_marker_enabled'):
                        self.do('cw 45')
            else:
                rospy.loginfo('Found marker, localisation OK')
                rospy.sleep(5)

    def run(self):
        """
            This methods creates a node in ROS, sends the command 'command' to the drone and subscribes
        """
        rospy.init_node('controller')
        rospy.loginfo(f'Starting controller node for {self.name}')

        self._buf = tf2_ros.Buffer(rospy.Duration(5))
        self.max_localisation_delay = rospy.Duration(5)  # wait for this amount of time before deciding we don't see any marker

        rospy.wait_for_service('command')
        self.do = rospy.ServiceProxy('command', TelloCommand, persistent=True)
        tf_listener = tf2_ros.TransformListener(self._buf)
        gt_listener = rospy.Subscriber('goto', GotoCommand, self._goto_callback)
        threading.Thread(target=self._search_marker).start()

        rospy.spin()

# -- the rest of the code will be only runned when executed from the terminal
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', type=int, required=True, help='Tello ID (should be equal to or greater than 1)')
    args, unknown = parser.parse_known_args()

    TelloController(args.id).run()

