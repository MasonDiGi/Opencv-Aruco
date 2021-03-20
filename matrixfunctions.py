import numpy as np
import math
from consts import MARKER_LOCATIONS


def poseToMatrix(x, y, theta):
    # If this function doesn't work nothing else does
    cosTheta = np.cos(theta)
    sinTheta = np.sin(theta)
    arr = np.matrix([[cosTheta, -sinTheta, x],
                     [sinTheta, cosTheta, y],
                     [0, 0, 1]])
    return arr


def matrixToPose(arr):
    # I think I have to do some sign flipping with this arccos to prevent wrap around error
    # I think it was something like
    # if sin(theta) < 0 then -arccos(x)
    # else arccos(x)
    # Don't remember tbh need testing
    angle = np.arccos(arr[0,0])

    if(arr[1,0] < 0):
        angle *= -1
    return [arr[0, 2], arr[1, 2], angle]


def matrixInverseMultipy(arr1, arr2):
    # might be more useful to not make this a function and just do the matmul inline
    return np.matmul(arr1, np.linalg.inv(arr2))
    # matrixMarker * matrixCamera^-1


def getMarkerMatrix(id):
    # get marker pose from dictionary
    x = MARKER_LOCATIONS[id][0]
    y = MARKER_LOCATIONS[id][1]
    theta = MARKER_LOCATIONS[id][2]
    # convert that pose to matrix and return
    return poseToMatrix(x, y, theta)

# Checks if a matrix is a valid rotation matrix.


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles


def rotationMatrixToEulerAngles(R):

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
