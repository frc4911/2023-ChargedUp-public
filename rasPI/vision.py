from cscore import CameraServer
from dt_apriltags import Detector
from networktables import NetworkTables

import cv2
import json
import logging
import numpy
import os
import time

# Taken from https://calibdb.net/
c270_camera_params = [931.4759229459526, 934.3344071111114, 335.83219605049294, 233.4049079752566]

# FRC 2023 manual states 6-inch tags, which is this many meters:
frc_tag_size_meters = 0.1524

at_detector = Detector(families='tag16h5',
                       nthreads=4,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=.25,
                       debug=0)

# Default IP for RoboRIO
server_ip = '10.49.11.2'
table_name = 'APRIL_TAGS'

NetworkTables.initialize(server_ip)

# Change to False to wait for NetworkTables connection
connected = True

# Mark this as connected
def connectionListener(connectedUpdate, info):
    print(f'Connection success: {info}')
    connected = connectedUpdate

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

# Optionally wait for connection
while True:
    time.sleep(1)
    if not connected:
        print("waiting for connection")
        continue
    break

tag_table = NetworkTables.getTable(table_name)

# Configure video capture
CameraServer.enableLogging()
camera = CameraServer.startAutomaticCapture()
camera.setResolution(640, 480)
sink = CameraServer.getVideo()

# Allocating new images is very expensive, always try to preallocate
input_img = numpy.zeros(shape=(480, 640, 3), dtype=numpy.uint8)

# Scan loop
while True:
    time.sleep(0.020)
    print("Getting a new image")

    # Read a new image
    capture_time, input_img = sink.grabFrame(input_img)

    # Check for errors
    if capture_time == 0:
        print("Error getting image")
        continue

    # Convert to greyscale
    gray_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)

    tags = at_detector.detect(gray_img,
                              estimate_tag_pose=True,
                              camera_params=c270_camera_params,
                              tag_size=frc_tag_size_meters)

    detectedTags = []

    rejectedTagCount = 0
    
    # iterate through detected tags
    for tag in tags:
        # Filter out likely false positives
        if (tag.decision_margin < 10):
            rejectedTagCount += 1
            continue
        # Only tags 0-7 are used in the game
        if (tag.tag_id >= 7):
            rejectedTagCount += 1
            continue

        # Build JSON message
        # data = {}
        # data['id'] = tag.tag_id
        # data['hamming'] = tag.hamming
        # data['decision_margin'] = tag.decision_margin
        # data['center'] = tag.center.tolist()
        # data['corners'] = tag.corners.tolist()
        # data['pose_R'] = tag.pose_R.tolist()
        # data['pose_t'] = tag.pose_t.tolist()
        # is this useful?
        # data['pose_err'] = tag.pose_err.tolist()

        sub_table = tag_table.getSubTable(f'tag_{tag.tag_id}')

        sub_table.putBoolean('detected', True)
        print(tag.center)
        print(tag.corners)
        print(tag.pose_R)
        print(tag.pose_t)
        sub_table.putNumberArray('center', [])
        sub_table.putNumberArray('corners', [])
        sub_table.putNumberArray('pose_R', [])
        sub_table.putNumberArray('pose_t', [])

        # print(f'adding tag {tag.tag_id} to detected list')
        # detectedTags.append(data)

    print(f'rejected {rejectedTagCount} tags with low decision margin')

    # jsonMessage = json.dumps(detectedTags)
    # print(f'sending tag collection to robot: {jsonMessage}')
    # table.putString(entry_name, jsonMessage)
