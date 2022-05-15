#!/usr/bin/python3

import numpy as np

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from tello_formation.msg import GotoCommand

from capt import CAPTSolver

class Supervisor(object):
    """
        This class is the supervisor and it controls all the controllers.
    """

    TELLO_RAD = 0.2  # in m, a bit bigger just to be safe
    TELLO_MAX_SPEED = 0.3  # in m/s, must be 0.1 <= MAX <= 1

    def __init__(self, n, m):
        """
            Constructor of the class Supervisor
            parameters:
                - n = number of start nodes (number of drones)
                - m = number of goal nodes
        """

        self.n = n
        self.m = m

        self.solver = CAPTSolver(Supervisor.TELLO_RAD, max_speed=Supervisor.TELLO_MAX_SPEED)

        self.lookup_rate = None
        self.max_localisation_delay = None
        self.buf = None


    def getpos(self, t):
        """
            Return translates the tf2 inforamtion into points coordinates
        """

        return t.translation.x, t.translation.y

    def set_marker_search(self, state):
        """
            This method sets the marker_search state (true/false)
            parameters:
                - state = if True -> search for makers and False -> no search
        """

        for i in range(self.n):
            rospy.set_param(f'tello_{i}/search_marker_enabled', state)

    def run(self):
        """
            This method creates a node in ROS and runs all the necessary instructions
        """
        rospy.init_node('supervisor')
        rospy.loginfo(f'Starting supervisor node (N={self.n}, M={self.m})')

        self.lookup_rate = rospy.Rate(5.0)
        self.max_localisation_delay = rospy.Duration(1)  # assume other nodes work fast enough
        self.buf = tf2_ros.Buffer()

        tf_listener = tf2_ros.TransformListener(self.buf)
        tf_broadcaster = tf2_ros.TransformBroadcaster()

        controllers = [rospy.Publisher(f'tello_{i}/goto', GotoCommand, queue_size=1) for i in range(self.n)]

        self.set_marker_search(True)
        while not rospy.is_shutdown():
            try:
                start = rospy.Time().now()
                self.solver.clear()
                starts = []  # we need to remember starting locations when m < n (i.e. don't move)
                for i in range(self.n):
                    t = self.buf.lookup_transform(rospy.get_param('world_frame_name', default='world'), f'tello_{i}', start, self.max_localisation_delay)
                    pos = self.getpos(t.transform)
                    self.solver.add_start(pos)
                    starts.append(t)
                goals = []
                for j in range(self.m):
                    t = self.buf.lookup_transform(rospy.get_param('world_frame_name', default='world'), f'goal_{j}', start, self.max_localisation_delay)
                    pos = self.getpos(t.transform)
                    self.solver.add_goal(pos)
                    goals.append(t)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, ValueError) as e:
                rospy.loginfo(f'Not enough or invalid start/end locations: {e}')
                self.lookup_rate.sleep()
            else:
                break
        else:
            exit()
        self.set_marker_search(False)
        rospy.loginfo(f'All Tello localised, computing trajectories for t_0={start.to_nsec()}')

        self.solver.compute_trajectories(start.to_sec())
        end = rospy.Time.from_sec(self.solver.end_time)

        for i in range(self.n):
            j = self.solver.assignment[i]
            tf = goals[j] if j != -1 else starts[i]
            tf.child_frame_id = f'tello_{i}_goal'
            tf_broadcaster.sendTransform(tf)
            speed = self.solver.speeds[i]
            controllers[i].publish(target_frame_id=tf.child_frame_id, start=start, speed=speed)

        rospy.loginfo('Supervisor node finished')

# -- the rest of the code will be only runned when execured from the terminal
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--tello-count', type=int, required=True,
        help='Number of Tello drones (ids need to be in 0,1,2,...,n-1)')
    parser.add_argument('-m', '--goal-count', type=int, required=True,
        help='Number of goals')
    args, unknown = parser.parse_known_args()

    Supervisor(args.tello_count, args.goal_count).run()

