from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
from consts import ARUCO_DICT
from consts import MARKER_LOCATIONS
import configparser
import numpy as np
import cameracal

def poseToMatrix(x, y, theta):
    # If this function doesn't work nothing else does
    cosTheta = np.cos(theta)
    sinTheta = np.sin(theta)
    arr = np.matrix([[cosFieldStart, -sinFieldStart, xFieldStart], 
        [sinFieldStart, cosFieldStart, yFieldStart],
        [0, 0, 1]])   
    return arr

def matrixToPose(arr):
    # I think I have to do some sign flipping with this arccos to prevent wrap around error
    # I think it was something like 
    # if sin(theta) < 0 then -arccos(x)
    # else arccos(x)
    # Don't remember tbh need testing
    return [arr[0][2], arr[1][2], numpy.arccos(arr[0][0])] 

def matrixTransform(arr1, arr2):
    # might be more useful to not make this a function and just do the matmul inline
    return np.matmul(arr1,np.linalg.inv(arr2))
    # matrixMarker * matrixCamera^-1
    

def getMarkerMatrix(id):
    # get marker pose from dictionary
    x = MARKER_LOCATIONS[id][0] 
    y = MARKER_LOCATIONS[id][1]
    theta = MARKER_LOCATIONS[id][2]
    # convert that pose to matrix and return
    return poseToMatrix(x,y,theta)

config = configparser.ConfigParser()
config.read("camera.ini")

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
                default="DICT_ARUCO_ORIGINAL",
                help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

mtx, dist, rvecs, tvecs, _ = cameracal.cal()

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
            corners, 0.079, mtx, dist, np.float32(rvecs), np.float32(tvecs))
        for i in ids:
            i = np.where(ids == i)
            cv2.aruco.drawAxis(frame, mtx,
                               dist, rvecs[i], tvecs[i], 0.035)
                               
            # robotPose = matrixToPose(matrixTransform(getMarkerMatrix(i),poseToMatrix(tvecs[i][0],tvecs[i][2],rvecs[i][0])))\
            rot, _ = cv2.Rodrigues(rvecs[0])
        print(str(rot[2][1]) + " " + str(rot[0][2]) + " " + str(rot[1][0]))


    
    # show the output frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
