import numpy
import cv2
from cv2 import aruco
# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 9
CHARUCOBOARD_COLCOUNT = 6
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_250)

# Create constants to be passed into OpenCV and Aruco methods
CHARUCO_BOARD = aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_COLCOUNT,
        squaresY=CHARUCOBOARD_ROWCOUNT,
        squareLength=0.04,
        markerLength=0.02,
        dictionary=ARUCO_DICT)

img = CHARUCO_BOARD.draw((2250,3000))

#Dump the calibration board to a file
cv2.imwrite('charuco2.png',img)
