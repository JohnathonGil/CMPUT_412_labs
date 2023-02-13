#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep


def gst_pipeline_string():
    # Parameters from the camera_node
    # Refer here : https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/camera_driver/config/jetson_nano_camera_node/duckiebot.yaml
    res_w, res_h, fps = 640, 480, 30
    fov = 'full'
    # find best mode
    camera_mode = 3  # 
    # compile gst pipeline
    gst_pipeline = """ \
            nvarguscamerasrc !\
            sensor-mode={}, exposuretimerange="100000 80000000" ! \
            video/x-raw(memory:NVMM), width=(int){}, height=(int){}, format=(string)NV12, 
                framerate=(fraction)30/1 ! \
            nvjpegenc ! \
            appsink \
        """.format(
        camera_mode,
        res_w,
        res_h,
        fps
    )

    # ---
    print("Using GST pipeline: `{}`".format(gst_pipeline))
    return gst_pipeline


cap = cv2.VideoCapture()
if not cap.open((e := gst_pipeline_string()), cv2.CAP_GSTREAMER):
    print("This is printing the following variable: " + e)
    exit(1)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Put here your code!
    # You can now treat output as a normal numpy array
    # Do your magic here

    sleep(1)

"""   lower_range_yellow= np.array([15,150,20])
    higher_range_yellow= np.array([25,255,255])

    curr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask  = cv2.inRange(curr_frame, lower_range_yellow, higher_range_yellow)

    result = cv2.bitwise_and(frame, frame)  """