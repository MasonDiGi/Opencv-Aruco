import numpy as np
import argparse
import cv2
import sys
from consts import DIMENSION, ARUCO_DICT

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", required=True, default="./",
                help="Path to output image containing ArUCo tag")
ap.add_argument("-t", "--type", type=str, default="DICT_5X5_250",
                help="Type of ArUCo tag to generate")
ap.add_argument("-n", "--number", type=int, required=True, default=50, 
                help="Number of tags to generate")
args = vars(ap.parse_args())

if ARUCO_DICT.get(args["type"], None) is None:
    print(f"[ERROR] ArUCo tag of {args['type']} is not supported.")
    sys.exit(0)

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
print(
    f"[INFO] Generating ArUCo tags of type '{args['type']}'."
)

for i in range(1, args["number"]+1):
    tag = np.zeros((DIMENSION, DIMENSION, 1), dtype="uint8")
    cv2.aruco.drawMarker(arucoDict, i, DIMENSION, tag, 1)
    cv2.imwrite(args["output"]+args["type"]+f"_id{i}.jpg", tag)
