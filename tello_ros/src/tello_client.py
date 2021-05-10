#!/usr/bin/python3


import socket
import threading
import time

class TelloClient(object):
    """
    This class communicates with a drone via commands. It also receives the responses
    """

    def __init__(self, client_ip, tello_ip, logger=print, setup=[]):
        """
            constructor of the TelloClient
            client_ip = the IP address of the controller
            tello_ip = the IP address of the drone
            setup = list of commands to set
        """

        self.addr = (client_ip, 9000)
        self.tello_addr = (tello_ip, 8889)
        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._command_lock = threading.Lock()
        self.running = False
        self.logger = logger
        self.setup_commands = ["command", "battery?", "streamon"] + setup

    def start(self):
        """
            Start the tello client
        """
        self.logger("Starting tello client")
        self.running = True
        self.command_sock.bind(self.addr)
        ping_thread = threading.Thread(target=self._ping_daemon)
        recv_thread = threading.Thread(target=self._recv_daemon)
        ping_thread.daemon = True
        recv_thread.daemon = True
        recv_thread.start()
        for cmd in self.setup_commands:
            self.send_command(cmd)
            time.sleep(1)  # TODO: wait for Tello response
        ping_thread.start()

    def stop(self):
        """
            sends the command emergency so that the drone stop immediately
        """
        self.logger("Stopping tello client")
        self.send_command("emergency")
        self.running = False

    def send_command(self, command):
        """
            sends the command passed in the parameter to the drone
        """
        self.logger(f"Sending command <{command}> to tello")
        encoded = command.encode('utf8')
        with self._command_lock:
            self.command_sock.sendto(encoded, self.tello_addr)

    def _recv_daemon(self):
        """
            The purpose of this method is to run in the background to be able to send commands all the time
        """
        while self.running:
            self._recv_response()

    def _ping_daemon(self):
        """
            Ping the drone to check the battery. This is to prevent the drone to land by himself (the drone will land by himself if
            no command was sent to him in an interval of 10sec)
        """
        while self.running:
            self.send_command('battery?')
            time.sleep(9)

    def _recv_response(self):
        """
            Receives the response back from the drone
        """
        bufsize = 4096
        response, addr = self.command_sock.recvfrom(bufsize)
        try:
            decoded = response.decode('utf8')
        except UnicodeDecodeError:
            decoded = repr(response)
        self.logger(f"Received {decoded} from {addr}")
        return decoded

# -- the rest of the code will be only runned when execured from the terminal
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--client-ip', default='')
    parser.add_argument('-t', '--tello-ip', default='192.168.10.1')
    args = parser.parse_args()
    client = TelloClient(args.client_ip, args.tello_ip)
    client.start()
    try:
        while True:
            command = input() #what is the raw_input()?? it is not declared anywhere
            client.send_command(command)
    finally:
        client.send_command('emergency')


