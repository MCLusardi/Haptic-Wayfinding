#!/usr/bin/env python

import rospy
import sys
import select
import tty
import termios
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
import json
import os
import signal

def tag_location_calculator():
    rospy.init_node('tag_location_calculator')
    listener = TransformListener()

    pose_file = "/path/to/pose_file.json"

    # Load existing poses from the file
    if os.path.exists(pose_file):
        with open(pose_file, 'r') as f:
            poses = json.load(f)
    else:
        poses = {}

    def publish_tag_location(event):
        try:
            # Get the robot's current pose in the map frame
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            tag_pose = PoseStamped()
            tag_pose.header.frame_id = "map"
            tag_pose.header.stamp = rospy.Time.now()
            tag_pose.pose.position.x = trans[0]
            tag_pose.pose.position.y = trans[1]
            tag_pose.pose.position.z = trans[2]
            tag_pose.pose.orientation.x = rot[0]
            tag_pose.pose.orientation.y = rot[1]
            tag_pose.pose.orientation.z = rot[2]
            tag_pose.pose.orientation.w = rot[3]

            # Publish the tag location
            tag_location_pub.publish(tag_pose)

        except Exception as e:
            rospy.logwarn("Failed to get robot pose: %s", str(e))

    def save_pose(label):
        try:
            if label in poses:
                rospy.loginfo("Label already exists. Please choose a different label.")
                return

            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            pose = {
                'position': {
                    'x': trans[0],
                    'y': trans[1],
                    'z': trans[2]
                },
                'orientation': {
                    'x': rot[0],
                    'y': rot[1],
                    'z': rot[2],
                    'w': rot[3]
                }
            }
            poses[label] = pose

            with open(pose_file, 'w') as f:
                json.dump(poses, f)

            rospy.loginfo("Pose saved with label: {}".format(label))
        except Exception as e:
            rospy.logwarn("Failed to get robot pose: {}".format(str(e)))

    def delete_pose(label):
        if label in poses:
            del poses[label]
            with open(pose_file, 'w') as f:
                json.dump(poses, f)
            rospy.loginfo("Pose with label '{}' deleted.".format(label))
        else:
            rospy.loginfo("Label '{}' not found.".format(label))

    def get_key():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def signal_handler(sig, frame):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.signal_shutdown("Ctrl+C pressed")
        sys.exit(0)
    
    #Set up signal handling for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    settings = termios.tcgetattr(sys.stdin)

    # Publisher for tag location
    tag_location_pub = rospy.Publisher('/tag_location', PoseStamped, queue_size=10)

    # Call publish_tag_location at 1 Hz
    rospy.Timer(rospy.Duration(1), publish_tag_location)

    while not rospy.is_shutdown():
        key = get_key()
        if key == '1':
            label = input("Enter label for the current pose: ")
            save_pose(label)
        elif key == '2':
            label = input("Enter label of the pose to delete: ")
            delete_pose(label)
        rospy.sleep(0.1)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    try:
        tag_location_calculator()
    except rospy.ROSInterruptException:
        pass
