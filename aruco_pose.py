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
import matrixfunctions as mf
import threading
from networktables import NetworkTables
import realsenseFuncs as rsf
import markerToField
import pickle

# Initialize NetworkTables Connection
# Wait for server to connect
cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
     print(info, '; Connected=%s' % connected)
     with cond:
         notified[0] = True
         cond.notify()

NetworkTables.initialize(server='10.0.41.2') # T41 Computer
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

print("Connected!")

table = NetworkTables.getTable('ArUco_Localization')


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
                default="DICT_5X5_250",
                help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

#Start realsense pipe
pipe = rsf.init()

with open("calsLogitechCharuco2/calibration.pkl", 'rb') as f:
    mtx, dist, rvecs, tvecs = pickle.load(f)

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

mapToOdomMatrix = np.matrix([[1,0,0], [0,1,0],[0,0,1]])

# loop over the frames from the video stream
while True:
    # PoseExists returns true if there is a pose, otherwise don't compute during this cycle
    poseExists, RSmatrix = rsf.getPose(pipe)
    odomX, odomY, odomTheta = mf.matrixToPose(RSmatrix)
    print(f"odomX: {odomX}; odomY: {odomY}, odomTheta: {odomTheta}")
    # If realsense isn't getting anything we're screwed basically
    if poseExists:
        # Construct matrix for the realsense in relation to the Odom DICT
        odomToRobotMatrix = mf.poseToMatrix(odomX, odomY,odomTheta)
        # Empty matrix to be filled in later with matrix for the odom origin in realtion to the map

        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 1000 pixels
    frame = vs.read()
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
            rot, _ = cv2.Rodrigues(rvecs[0])
            euler =  mf.rotationMatrixToEulerAngles(rot)
            # Construct matrix for the marker in relation to the map
            mapToRobotMatrix = markerToField.getMarkerPose(ids[i][0], tvecs[0], rvecs)
            # Homogenous Matrix Transformation to get map in Relation to Odom matrix
            mapToOdomMatrix = mf.matrixInverseMultipy(mapToRobotMatrix,odomToRobotMatrix)
            # print(mapToRobotMatrix)
    #robotPose = mf.matrixToPose(mapToRobotMatrix)
    robotPose = mf.matrixToPose(np.matmul(mapToOdomMatrix,odomToRobotMatrix))
    # Push to networktables
    table.putNumber("x", robotPose[0])
    table.putNumber("y", robotPose[1])
    table.putNumber("theta", robotPose[2])
    # print(str(robotPose))

    # # show the output frame
    # frame = cv2.flip(frame, 1)
    # cv2.imshow("Frame", frame)
    # key = cv2.waitKey(1) & 0xFF
    # # if the `q` key was pressed, break from the loop
    # if key == ord("q"):
    #     break
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
