#!/usr/bin/env python3

import cv2
import numpy as np
import os

# Define the dimensions of the checkerboard
CHECKERBOARD = (6, 9)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []

# Defining the world coordinates for 3D points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Start capturing images for calibration
cap = cv2.VideoCapture(2)

# Number of checkerboard images to capture
num_calibration_images = 30  # Set your desired number here

while len(objpoints) < num_calibration_images:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)

    cv2.imshow('Calibration', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Performing camera calibration
if len(objpoints) > 0:
    gray_shape = gray.shape[::-1]
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)

    # Save the calibration results
    np.savez('calibration_data.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

    print("Camera calibration complete.")
    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coeffs)
else:
    print("No 6 x 9 checkerboard images captured for calibration.")
