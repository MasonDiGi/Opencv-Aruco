# R = Robot
# CM = Camera Mount
# CO = Camera optical frame
from math import sin, cos, radians
from consts import MARKER_LOCATIONS
import cv2
import numpy as np


def getMarkerPose(i, tvecs, rvecs):
    # Theta of marker
    marker = MARKER_LOCATIONS[i]
    th = marker[2]
    th = radians(th)
    x = marker[0]
    y = marker[1]
    z = 0.08
    robotZ = 0.047
    # Marker Rotation
    rot, _ = cv2.Rodrigues(rvecs)
    Tr2cm = np.matrix([[1, 0, 0,   0   ],
                       [0, 1, 0,   0   ], 
                       [0, 0, 1, robotZ], 
                       [0, 0, 0,  1 ]])
    Tcm2co = np.matrix([[0 , 0 , 1, 0],
                        [-1, 0 , 0, 0],
                        [0 , -1, 0, 0], 
                        [0 , 0 , 0, 1]])
    Tmarker = np.matrix([[-sin(th), 0, cos(th), x],
                         [cos(th) , 0, sin(th), y],
                         [0       , 1,    0   , z],
                         [0       , 0,    0   , 1]])
                         # tested working
    Tmeasure = np.matrix([[rot[0, 0], rot[0, 1], rot[0, 2], tvecs[0, 0]],
                          [rot[1, 0], rot[1, 1], rot[1, 2], tvecs[0, 1]],
                          [rot[2, 0], rot[2, 1], rot[2, 2], tvecs[0, 2]],
                          [    0    ,     0    ,     0    ,      1     ]])

    og = Tr2cm * Tcm2co * Tmeasure
    inv = np.linalg.inv(og)
    Tm2r = np.matmul(Tmarker, inv)
    return Tm2r
