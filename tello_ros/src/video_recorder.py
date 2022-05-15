#!/usr/bin/python3

import cv2
import sys

if __name__ == '__main__':
    url = 'udp://192.168.10.1:11111'

    video_capture = cv2.VideoCapture(url)
    w = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH)) 
    h = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    video_writer = cv2.VideoWriter(sys.argv[1], fourcc, video_capture.get(cv2.CAP_PROP_FPS), (w, h))

    while True:
        ret_val, frame = video_capture.read()
        video_writer.write(frame)
        cv2.imshow("Recording preview", frame) 

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
    video_writer.release()
   