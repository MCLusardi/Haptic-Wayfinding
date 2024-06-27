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
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
        # Load camera calibration data
        package_path = os.path.dirname(__file__)
        calibration_file = os.path.join(package_path, '../scripts/calibration_data.npz')
        with np.load(calibration_file) as data:
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']

    def detect_marker(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame from camera.")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                rospy.loginfo(f"Detected markers: {ids.flatten()}")
                for i in range(len(ids)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.camera_matrix, self.dist_coeffs)
                    distance = np.linalg.norm(tvec)
                    rospy.loginfo(f"Marker ID: {ids[i]}, Distance: {distance}")

                    if 5 <= distance <= 10:
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
    rospy.init_node('landmark_detector')
    detector = LandmarkDetector()
    detector.detect_marker()
    rospy.spin()