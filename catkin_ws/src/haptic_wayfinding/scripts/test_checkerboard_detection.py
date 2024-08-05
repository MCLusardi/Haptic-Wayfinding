import cv2
import numpy as np

# Define the dimensions of the checkerboard
CHECKERBOARD = (6, 9)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Load the checkerboard image
img = cv2.imread('/mnt/data/Screen Shot 2024-06-27 at 3.36.31 PM.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Find the checkerboard corners
ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

if ret:
    print("Checkerboard detected.")
    img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
    cv2.imshow('Checkerboard', img)
    cv2.waitKey(0)
else:
    print("Checkerboard not detected.")

cv2.destroyAllWindows()
