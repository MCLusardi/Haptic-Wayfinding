#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import rospy
from haptic_wayfinding.msg import HapticRumble

class ArucoDetector:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.haptic_pub = rospy.Publisher('/haptic_rumble', HapticRumble, queue_size=1)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
        # Construct the path to the calibration data file
        package_path = os.path.dirname(__file__)  # Get the current directory of this script
        calibration_file = os.path.join(package_path, '../scripts/calibration_data.npz')

        # Load calibration data
        with np.load(calibration_file) as data:
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']

    def detect_marker(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                for i in range(len(ids)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.camera_matrix, self.dist_coeffs)
                    distance = np.linalg.norm(tvec)
                    if 5 <= distance <= 10:
                        self.publish_haptic_feedback()
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
    
    def publish_haptic_feedback(self):
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
        self.haptic_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('aruco_detector')
    detector = ArucoDetector()
    detector.detect_marker()
    rospy.spin()