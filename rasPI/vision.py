from cscore import CameraServer
from dt_apriltags import Detector
from networktables import NetworkTables

import cv2
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

# [367.91895687 156.03958565]

# [[322.9781189  211.85998535]
#  [420.58554077 210.97744751]
#  [417.59829712  94.33355713]
#  [318.15740967 104.13204193]]

# [[ 0.86346625  0.03805549  0.502969  ]
#  [-0.00455012  0.99769697 -0.06767608]
#  [-0.50438609  0.05614744  0.86165082]]

# [[ 0.0444061 ]
#  [-0.10594213]
#  [ 1.27216578]]
# Scan loop
while True:
    # Adjust this to process faster/slower
    time.sleep(3)
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

    detected_tags = []

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

        # Put everything in one array since NetworkTables doesn't really do transactions
        tag_values_list = [1.0, tag.hamming, tag.decision_margin]
        tag_values_list.extend(tag.center)
        tag_values_list.extend(tag.corners.flatten())
        tag_values_list.extend(tag.pose_R.flatten())
        tag_values_list.extend(tag.pose_t.flatten())
        tag_table.getSubTable(f'tag_{tag.tag_id}').putNumberArray('values', tag_values_list)

        detected_tags.append(tag.tag_id)

    print(f'rejected {rejectedTagCount} tags with low decision margin')

    # Mark all the undetected tags as such
    for tag_id in range(8):
        if tag_id not in detected_tags:
            tag_table.getSubTable(f'tag_{tag_id}').putNumberArray('values', [0.0])

    # Pushes all pending changes immediately
    NetworkTables.flush()
    print(f'sending {len(detected_tags)} detected tags to robot')
