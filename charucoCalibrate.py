import time
import cv2
import numpy as np
from imutils.video import VideoStream


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
board = cv2.aruco.CharucoBoard_create(6,9,.025,.0125,dictionary)
# img = board.draw((200*3,200*3))
#
# #Dump the calibration board to a file
# cv2.imwrite('charuco.png',img)


#Start capturing images for calibration
print("[INFO] starting video stream...")
vs = VideoStream(src=1).start()
time.sleep(2.0)

while True:
    allCorners = []
    allIds = []
    decimator = 0
    for i in range(300):

        frame = vs.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res = cv2.aruco.detectMarkers(gray,dictionary)

        if len(res[0])>0:
            res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])
                
        #     cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])
        #
        # cv2.imshow('frame',frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        decimator+=1

    imsize = gray.shape

    #Calibration fails for lots of reasons. Release the video if we do
    try:
        cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
    except:
        cap.release()


cap.release()
cv2.destroyAllWindows()
