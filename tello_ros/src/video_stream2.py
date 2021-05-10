#!/usr/bin/python3

import time
import threading
import sys
import traceback

import numpy as np
import ffmpeg
import rospy
import ros_numpy
import cv2

from sensor_msgs.msg import Image

class VideoStreamClient:
    """
        This class captures the video from the drone and publishes it
    """

    def __init__(self, url, width=960, height=720):
        """
            Constructor of the VideoStreamClient
            url = url to capture the video
            width = width of the video stream (default value = 960)
            height = height of the video stream (default value = 720)
        """
        self.url = url
        self.width = width
        self.height = height
        self._frame = None
        self._frame_lock = threading.Lock()
        self.running = False
        self.read_capture_thread = None

    @property
    def frame(self):
        with self._frame_lock:
            f = self._frame.copy() if self._frame is not None else None
        return f

    @frame.setter
    def frame(self, new):
        with self._frame_lock:
            self._frame = new

    def start(self):
        self.read_capture_thread = threading.Thread(target=self._run)
        self.read_capture_thread.daemon = True
        self.running = True
        self.read_capture_thread.start()

    def stop(self):
        self.running = False
        self.read_capture_thread.join()

    def _run(self):
        # Code adapted from example in documentation
        print(f'Decoding from {self.url}', file=sys.stderr)
        fps = 5  # increasing it causes delay to build up...
        delay_reduction = {
            'fflags': '+discardcorrupt+nobuffer+flush_packets',
            # 'vsync': 'drop',  # causes delay
            'flags': '+low_delay',
            # 'r': fps,  # do not enable this! this causes frames to fall behind and builds delay
            # 'framedrop': '',  # does not exist - maybe try frame_drop_threshold n
            'avioflags': 'direct',
        # } # the rest below is not needed, keep it just in case
        # faststart = {  # python 2 does not like f(**d1, **d2) so just use a single dict
            # 'probesize': 32,
            # 'analyzeduration': 0,
            # 'sync': 'ext',
        }
        decode = (ffmpeg
            # specify video decoder explicitely - default v4l2 did not work
            .input(self.url, vcodec='h264', **delay_reduction) #, **faststart)
            .output('pipe:', format='rawvideo', pix_fmt='rgb24', r=fps)
        )
        print('Executing:', ' '.join(decode.compile()), file=sys.stderr)
        starttime = time.time()
        readtime = None
        try:
            decoder_proc = decode.run_async(pipe_stdout=True)
            timings = []
            while self.running:
                t1 = time.time()
                in_bytes = decoder_proc.stdout.read(self.width * self.height * 3)
                readtime = readtime if readtime is not None else time.time()
                if not in_bytes:
                    print("Error: decoder process stdout empty", file=sys.stderr)
                    break
                pixels = np.frombuffer(in_bytes, dtype=np.uint8)
                self.frame = pixels.reshape(self.height, self.width, 3)
                timings.append(time.time() - t1)

            print(f"Duration from spawning process to first read: {(readtime - starttime)}", file=sys.stderr)
            print(f"Reading took {(sum(timings) / len(timings))} on avg", file=sys.stderr)
        except Exception as e:
            print(traceback.format_exc(e))
        finally:
            # TODO: check subprocess clean-up
            decoder_proc.stdout.close()
            decoder_proc.wait()


def publish(client, id, verbose=False, rate=30):
    """
        Publishes the frame captured from the drone
    """
    publisher = rospy.Publisher("camera/image_raw", Image, queue_size=2)
    rospy.init_node('tello_video')
    rospy.loginfo("Started tello_video node")
    rospy.wait_for_service('command')  # wait for streamon
    rate = rospy.Rate(RATE)
    client.start()
    while not rospy.is_shutdown():
        frame = None
        while frame is None:
            frame = client.frame
        img_msg = ros_numpy.msgify(Image, frame, encoding='rgb8')
        img_msg.header.stamp = rospy.Time.now()
        publisher.publish(img_msg)
        if VERBOSE:
            rospy.loginfo("Published video frame")
        rate.sleep()
    client.stop()

# -- the rest of the code will be only runned when execured from the terminal
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser("Publish video stream from Tello drone to /camera/image_raw")
    parser.add_argument('-u', '--url', default='udp://192.168.10.2:11111')
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('-r', '--rate', type=int, default=30)
    args, unknown = parser.parse_known_args()
    VERBOSE = args.verbose
    RATE = args.rate
    client = VideoStreamClient(args.url)
    publish(client, args.verbose, args.rate)
    client.running = True
    """
    input("Press <Enter> to start receiving video stream...")
    tello_ip = '192.168.10.1'

    # Creating stream capture object
    cap = cv2.VideoCapture('udp://'+tello_ip+':11111')
    try:
        while True:
            ret, frame = cap.read()
            cv2.imshow('DJI Tello', frame)
            if frame is not None:
                cv2.imshow('DJI Tello', frame)
            else:
                print("No frame could be read (did you send 'streamon'?)")
                input("Press <Enter> to retry...")
                continue

            # Video Stream is closed if escape key is pressed
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    """
    """
    client._run()
    t0 = time.time()
    while True:
        frame = client.frame()
        print("Got frame", frame, round(time.time() - t0, 2), " " * 20)
        try:
            cv2.imshow("Tello", frame)
        except Exception as e:
            print(e)"""
