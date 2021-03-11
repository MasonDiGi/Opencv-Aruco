import cv2

DIMENSION = 300
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


# (x, z, theta, field id)
MARKER_LOCATIONS = {
    1: (0.1219, 0, 0, "B1"),
    2: (0, 0, 0, "B2"),
    3: (0, 0, 0, "B3"),
    4: (0, 0, 0, "B4"),
    5: (0, 0, 0, "B5"),
    6: (0, 0, 0, "B6"),
    7: (0, 0, 0, "B7"),
    8: (0, 0, 0, "B8"),
    9: (0, 0, 0, "B9"),
    10: (0, 0, 0, "B10"),
    11: (0, 0, 0, "B11"),
    12: (0, 0, 0, "D1"),
    13: (0, 0, 0, "D2"),
    14: (0, 0, 0, "D3"),
    15: (0, 0, 0, "D4"),
    16: (0, 0, 0, "D5"),
    17: (0, 0, 0, "D6"),
    18: (0, 0, 0, "D7"),
    19: (0, 0, 0, "D8"),
    20: (0, 0, 0, "D9"),
    21: (0, 0, 0, "D10"),
    22: (0, 0, 0, "D11"),
    23: (0, 0, 0, "A1"),
    24: (0, 0, 0, "A3"),
    25: (0, 0, 0, "A4"),
    26: (0, 0, 0, "A6"),
    27: (0, 0, 0, "A7"),
    28: (0, 0, 0, "A8"),
    29: (0, 0, 0, "A9"),
    30: (0, 0, 0, "A10"),
    31: (0, 0, 0, "E1"),
    32: (0, 0, 0, "E3"),
    33: (0, 0, 0, "E4"),
    34: (0, 0, 0, "E6"),
    35: (0, 0, 0, "E7"),
    36: (0, 0, 0, "E8"),
    37: (0, 0, 0, "E9"),
    38: (0, 0, 0, "E10"),
    39: (0, 0, 0, "C3"),
    0: (0, 0, 0, "C9"),
}
