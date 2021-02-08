def cal():
    import numpy as np
    import cv2
    import glob
    import imutils

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((5*5, 3), np.float32)
    objp[:, :2] = np.mgrid[0:5, 0:5].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    gray = None
    images = glob.glob('cals/*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (5, 5), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (5, 5), corners2, ret)
            img = imutils.resize(img, width=800)
            cv2.imshow("Frame", img)
            while True:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)
        return mtx, dist, rvecs, tvecs
    cv2.destroyAllWindows()


if __name__ == '__main__':
    cal()
