import sys
import time
import datetime
import cv2
import numpy as np
import math


# open camera
camera = cv2.VideoCapture(0)
camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,480)

if not camera.isOpened():
    print "failed to open camera, exiting!"
    sys.exit(0)
print "camera is open..."

while True:
    # get the image from the webcam
    success_flag, image = camera.read()

    # if successful overwrite our latest image
    if success_flag:
        latest_image = image
    print "saving..."
    filename = "Image: " + datetime.datetime.now().strftime("%m-%d %H:%M:%S") + ".jpg"
    cv2.imwrite(filename, latest_image)

    time.sleep(0.5)
	