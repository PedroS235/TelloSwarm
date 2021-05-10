#!/usr/bin/python3
import subprocess

import rospy

from tello_client import TelloClient
from tello_ros.srv import TelloCommand


class TelloClientROS(TelloClient):
    """
        This class is a sublclass of the TelloClient
    """

    def __init__(self, takeoff=False):
        setup = ['takeoff'] if takeoff else []
        super(TelloClientROS, self).__init__(client_ip='', tello_ip='192.168.10.1', logger=rospy.loginfo, setup=setup)
        self._service = None

    def start(self):
        """
            This method tries to automatically connect to the tello wifi
        """
        rospy.init_node('command_srv')
        self.logger('Connecting to Tello wifi...')
        while not rospy.is_shutdown():
            ret = subprocess.call(['nmcli', 'c', 'up', 'tello01']) #edit here the name of the Tello wifi name (tello01=name)
            if int(ret) == 0:
                self.logger('Connected to Tello wifi')
                break
            rospy.sleep(0.5)
        super(TelloClientROS, self).start()
        self._service = rospy.Service('command', TelloCommand, self._handle_command)

    def _handle_command(self, req):
        self.send_command(req.command)
        return "check the logs"

    def _recv_daemon(self):
        while not rospy.is_shutdown() and self.running:
            r = self._recv_response()

# -- the rest of the code will be only runned when execured from the terminal
if __name__ == '__main__':
    import argparse
    import atexit
    parser = argparse.ArgumentParser()
    parser.add_argument('--takeoff', action='store_true')
    args, unknown = parser.parse_known_args()
    client = TelloClientROS(takeoff=args.takeoff)
    atexit.register(client.stop)
    rospy.on_shutdown(lambda: rospy.loginfo('Command server shutting down'))
    rospy.on_shutdown(client.stop)
    client.start()
    rospy.spin()

