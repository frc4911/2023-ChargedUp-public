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
c270_camera_params = [1419.6926739461624, 1419.204023362152, 624.4230937815905, 317.19888645121466]

# FRC 2023 manual states 6-inch tags, which is this many meters:
frc_tag_size_meters = 0.1524

at_detector = Detector(families='tag16h5',
                       nthreads=4,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=.25,
                       debug=0)

server_ip = '10.49.11.2'
table_name = 'APRIL_TAGS'
entry_name = 'DETECTED_TAGS'

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

table = NetworkTables.getTable(table_name)

# Configure video capture
CameraServer.enableLogging()
camera = CameraServer.startAutomaticCapture()
camera.setResolution(1280, 720)
sink = CameraServer.getVideo()

# Allocating new images is very expensive, always try to preallocate
input_img = numpy.zeros(shape=(720, 1280, 3), dtype=numpy.uint8)

# Scan loop
while True:
    time.sleep(1)
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

    # iterate through detected tags
    for tag in tags:
        # Filter out likely false positives
        if (tag.decision_margin < 10):
            print(f'rejecting tag {tag.tag_id} with low decision margin of: {tag.decision_margin}')
            continue
        # Only tags 0-7 are used in the game
        if (tag.tag_id >= 7):
            print(f'ignoring tag {tag.tag_id} with invalid id')
            continue

        # Build JSON message
        data = {}
        data['id'] = tag.tag_id
        data['hamming'] = tag.hamming
        data['decision_margin'] = tag.decision_margin
        data['center'] = tag.center.tolist()
        data['corners'] = tag.corners.tolist()
        data['pose_R'] = tag.pose_R.tolist()
        data['pose_t'] = tag.pose_t.tolist()
        # is this useful?
        # data['pose_err'] = tag.pose_err.tolist()

        print(f'adding tag {tag.tag_id} to detected list')
        detectedTags.append(data)

    jsonMessage = json.dumps(detectedTags)
    print(f'sending tag collection to robot: {jsonMessage}')
    table.putString(entry_name, jsonMessage)
