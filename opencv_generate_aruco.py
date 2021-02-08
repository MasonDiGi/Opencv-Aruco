import numpy as np
import argparse
import cv2
import sys
from consts import DIMENSION, ARUCO_DICT

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", required=True,
                help="Path to output image containing ArUCo tag")
ap.add_argument("-i", "--id", type=int, required=True,
                help="ID of ArUCo tag to generate")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL",
                help="Type of ArUCo tag to generate")
args = vars(ap.parse_args())

if ARUCO_DICT.get(args["type"], None) is None:
    print(f"[ERROR] ArUCo tag of {args['type']} is not supported.")
    sys.exit(0)

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
print(
    f"[INFO] Generating ArUCo tag type '{args['type']}' with ID '{args['id']}'."
)
tag = np.zeros((DIMENSION, DIMENSION, 1), dtype="uint8")
cv2.aruco.drawMarker(arucoDict, args['id'], DIMENSION, tag, 1)

cv2.imwrite(args["output"], tag)
cv2.imshow("ArUCo Tag", tag)
cv2.waitKey(0)
