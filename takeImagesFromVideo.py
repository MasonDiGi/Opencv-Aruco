import imutils
from imutils.video import VideoStream
import time
import cv2

print("[INFO] starting video stream...")
vs = VideoStream(src=2).start()
time.sleep(2.0)
num = 2

while True:
    frame = vs.read()
    # show the output frame
    frame = cv2.flip(frame, 1)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        cv2.imwrite(f"calsLogitechCharuco2/Charcuo_cal_{num}.jpg", frame)
        num += 1
