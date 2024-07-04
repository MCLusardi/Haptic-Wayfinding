#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
from std_msgs.msg import Bool
import pyrealsense2 as rs

class LandmarkDetector:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device('153122077062')
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # Align depth frame to color frame
        self.align = rs.align(rs.stream.color)

        self.haptic_pub = rospy.Publisher('/landmark_in_range', Bool, queue_size=1)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
    
    def detect_marker(self):
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                rospy.logwarn("Failed to capture frames from camera.")
                continue

            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)

            if ids is not None:
                rospy.loginfo(f"Detected markers: {ids.flatten()}")
                marker_in_range = False
                for i in range(len(ids)):
                    # Get the depth value for the center of the marker
                    corners_reshaped = corners[i].reshape(-1, 2)
                    center_x = int(corners_reshaped[:, 0].mean())
                    center_y = int(corners_reshaped[:, 1].mean())
                    distance_z = depth_frame.get_distance(center_x, center_y)

                    rospy.loginfo(f"Marker ID: {ids[i]}, Distance (z-value): {distance_z}")

                    if 0.0 <= distance_z <= 1.0:
                        rospy.loginfo(f"Marker ID: {ids[i]} within range.")
                        marker_in_range = True

                    if 245 <= ids[i] <= 249:
                        self.publish_haptic_feedback(marker_in_range, int(ids[i]))
            else:
                rospy.loginfo("No markers detected.")

            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("Quitting detection loop.")
                break

        self.pipeline.stop()
        cv2.destroyAllWindows()

    def publish_haptic_feedback(self, in_range, marker_id):
        rospy.loginfo(f"Publishing to /landmark_in_range: {in_range} for marker ID: {marker_id}")
        msg = Bool()
        msg.data = in_range
        self.haptic_pub.publish(msg)
        rospy.set_param('detected_marker_id', marker_id)

if __name__ == '__main__':
    rospy.init_node('aruco_detector')
    detector = LandmarkDetector()
    detector.detect_marker()
    rospy.spin()