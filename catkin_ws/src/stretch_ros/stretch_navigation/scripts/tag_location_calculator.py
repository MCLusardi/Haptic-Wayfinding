#!/usr/bin/env python

import rospy
import sys
import select
import tty
import termios
from tf import TransformListener
from geometry_msgs.msg import PoseStamped

def tag_location_calculator():
    rospy.init_node('tag_location_calculator')
    listener = TransformListener()

    pose_file = "/home/hcal-group/Haptic-Wayfinding/catkin_ws/src/stretch_ros/stretch_navigation/scripts/pose_file.json"

    # Load existing poses from the file
    if os.path.exists(pose_file):
        with open(pose_file, 'r') as f:
            poses = json.load(f)
    else:
        poses = {}

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

    settings = termios.tcgetattr(sys.stdin)

    # Call publish_tag_location at 1 Hz
    rospy.Timer(rospy.Duration(1), publish_tag_location)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    try:
        tag_location_calculator()
    except rospy.ROSInterruptException:
        pass
