#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
from haptic_wayfinding.msg import HapticRumble
import os

class LandmarkDetector:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.haptic_pub = rospy.Publisher('/haptic_rumble', HapticRumble, queue_size=1)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Define the intrinsic parameters of Intel RealSense D435i
        fx = 616.537  # focal length in x direction (in pixels)
        fy = 616.979  # focal length in y direction (in pixels)
        cx = 325.952  # principal point x-coordinate (in pixels)
        cy = 253.313  # principal point y-coordinate (in pixels)

        self.camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float64)

        # Distortion coefficients for RealSense cameras are typically low
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)  # Assuming no distortion

        marker_size = 0.05
        self.marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                       [marker_size / 2, marker_size / 2, 0],
                                       [marker_size / 2, -marker_size / 2, 0],
                                       [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    def detect_marker(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame from camera.")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)

            
            if ids is not None:
                rospy.loginfo(f"Detected markers: {ids.flatten()}")
                for i in range(len(ids)):
                    # Estimate pose using solvePnP
                    rvec, tvec, _ = cv2.solvePnP(self.marker_points, corners[i], self.camera_matrix, self.dist_coeffs)

                    # Log distance and pose information
                    distance = np.linalg.norm(tvec)
                    rospy.loginfo(f"Marker ID: {ids[i]}, Distance: {distance}")

                    if 30 <= distance <= 240:
                        rospy.loginfo(f"Marker ID: {ids[i]} within range.")
                        self.publish_haptic_feedback()
            else:
                rospy.loginfo("No markers detected.")

            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("Quitting detection loop.")
                break

        self.cap.release()
        cv2.destroyAllWindows()
    
    def publish_haptic_feedback(self):
        rospy.loginfo("Before Publishing HapticRumble message.")
        msg = HapticRumble()
        msg.left = True
        msg.left_volume = 1.0
        msg.left_delay = 0.5
        msg.right = True
        msg.right_volume = 1.0
        msg.right_delay = 0.5
        msg.front = True
        msg.front_volume = 1.0
        msg.front_delay = 0.5
        rospy.loginfo("Publishing HapticRumble message.")
        self.haptic_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('aruco_detector')
    detector = LandmarkDetector()
    detector.detect_marker()
    rospy.spin()
