import cv2
import numpy as np

# frame = cv2.imread("black_white_map.png")

# height = frame.shape[0]
# width = frame.shape[1]

# frame = frame[5:height-5,5:width-5]

# frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# (threshi, frame_bw) = cv2.threshold(frame_gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

# cv2.imshow('Output', frame_bw)
# cv2.waitKey(0)

# cv2.imwrite("bw.png", frame_bw)

frame = cv2.imread("map3.png")

height = frame.shape[0]
width = frame.shape[1]

frame = frame[5:height-5,5:width-5]

frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
(threshi, frame_bw) = cv2.threshold(frame_gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

cv2.imshow('Output', frame_bw)
cv2.waitKey(0)

cv2.imwrite("bw3.png", frame_bw)