import cv2
import imutils
CORNERS = (6, 9)


def cal():
    import numpy as np
    import glob

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((CORNERS[0]*CORNERS[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CORNERS[0], 0:CORNERS[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    gray = None
    images = glob.glob('calsLogitech/*png')
    for fname in images:
        # objpoints = []  # 3d point in real world space
        # imgpoints = []  # 2d points in image plane.
        img = cv2.imread(fname)
        # img = imutils.resize(img, width=800)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(
            gray, CORNERS)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            # img = cv2.drawChessboardCorners(
            #     img, CORNERS, corners2, ret)
            # cv2.imshow('img',img)
            # cv2.waitKey(0)
    cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    return mtx, dist, rvecs, tvecs, img


if __name__ == '__main__':
    _, _, _, _, img = cal()
    cv2.imshow("pof", img)
    cv2.waitKey(0)
