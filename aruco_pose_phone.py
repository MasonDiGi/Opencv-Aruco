from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
from consts import ARUCO_DICT
import configparser
import numpy as np
from cameracal import cal

config = configparser.ConfigParser()
config.read("camera.ini")

camCal = cal()

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
                default="DICT_ARUCO_ORIGINAL",
                help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

# verify that the supplied ArUCo tag exists and is supported by OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))
    sys.exit(0)
# load the ArUCo dictionary and grab the ArUCo parameters
print(f"[INFO] detecting '{args['type']}' tags...")
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()
# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

# loop over the frames from the video stream
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 1000 pixels
    frame = vs.read()
    frame = imutils.resize(frame, width=800)
    # detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        frame, arucoDict, parameters=arucoParams)
# verify at least one ArUco marker was detected
    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, 2.25, camCal[0], camCal[1])
        print(tvecs)
        for i in ids:
            i = np.where(ids == i)
        cv2.aruco.drawAxis(frame, camCal[0],
                           camCal[1], rvecs[i], tvecs[i], 1.25)

        # show the output frame
    cv2.imshow("Frame", frame)
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
