from dt_apriltags import Detector
from networktables import NetworkTables
from networktables.util import ntproperty

import cv2
import json
import logging
import numpy
import os
import time

camera = cv2.VideoCapture(-1)

at_detector = Detector(families='tag16h5',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=.25,
                       debug=0)

server_ip = '10.49.11.2'
table_name = 'APRIL_TAGS'

NetworkTables.initialize(server_ip)

# Change to False to wait for NetworkTables connection
connected = True

# Mark this as connected
def connectionListener(connectedUpdate, info):
    print(info)
    connected = connectedUpdate

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

# Optionally wait for connection
while True:
    time.sleep(1)
    if not connected:
        print("waiting for connection")
        continue
    break

table = NetworkTables.getTable(table_name)

# Scan loop
while True:
    time.sleep(2)
    # Read image
    result, color_img = camera.read()
    # Convert to greyscale
    gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

# TODO(riley) Determine tag size value here
# TODO(riley) Determine camera_params and set them here to make transform pose available
# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
    tags = at_detector.detect(gray_img, estimate_tag_pose=False, camera_params=None, tag_size=None)

    # iterate through detected tags
    for tag in tags:
        # TODO(rbrewer) filter tags on hamming and/or decision margin
        data = {}
        data['id'] = tag.tag_id
        data['hamming'] = tag.hamming
        data['decision_margin'] = tag.decision_margin
        data['center'] = tag.center.tolist()
        data['corners'] = tag.corners.tolist()
        # data['pose_R'] = tag.poseR.tolist()
        # data['pose_t'] = tag.poset.tolist()
        print(json.dumps(data))
        # TODO(rbrewer) send tag_id + pose_t via network table
        # https://robotpy.readthedocs.io/projects/pynetworktables/en/stable/examples.html#ntproperty-example
        # Something like table.putNumberArray(tag.tag_id, tag.poset.tolist())
