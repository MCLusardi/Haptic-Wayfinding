#!/usr/bin/env python3

import rospy
import actionlib
import sys
import json
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf import transformations

class StretchNavigation:
    """
    A simple encapsulation of the navigation stack for a Stretch robot.
    """
    def __init__(self):
        """
        Create an instance of the simple navigation interface.
        :param self: The self reference.
        """
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

    def get_quaternion(self, theta):
        """
        A function to build Quaternions from Euler angles. Since the Stretch only
        rotates around z, we can zero out the other angles.
        :param theta: The angle (radians) the robot makes with the x-axis.
        """
        return Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, theta))

    def go_to(self, x, y, theta):
        """
        Drive the robot to a particular pose on the map. The Stretch only needs
        (x, y) coordinates and a heading.
        :param x: x coordinate in the map frame.
        :param y: y coordinate in the map frame.
        :param theta: heading (angle with the x-axis in the map frame)
        """
        rospy.loginfo("Heading for ({0}, {1}) at {2} radians".format(x, y, theta))

        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation = self.get_quaternion(theta)

        rospy.loginfo("Sending goal: {}".format(self.goal))
        self.client.send_goal(self.goal, done_cb=self.done_callback)
        rospy.loginfo("Goal sent, waiting for result...")
        self.client.wait_for_result()
        rospy.loginfo("Result received")

    def done_callback(self, status, result):
        """
        The done_callback function will be called when the joint action is complete.
        :param self: The self reference.
        :param status: status attribute from MoveBaseActionResult message.
        :param result: result attribute from MoveBaseActionResult message.
        """
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("SUCCEEDED in reaching the goal.")
        else:
            rospy.loginfo("FAILED in reaching the goal with status: {}".format(status))

    def convert_pose_to_2d_goal(self, x, y, qx, qy, qz, qw):
        """
        Convert the given pose to a 2D navigation goal and navigate.
        :param x: x coordinate in the map frame.
        :param y: y coordinate in the map frame.
        :param qx: x component of the quaternion.
        :param qy: y component of the quaternion.
        :param qz: z component of the quaternion.
        :param qw: w component of the quaternion.
        """
        # Extract quaternion and convert to Euler angles
        euler = transformations.euler_from_quaternion([qx, qy, qz, qw])
        theta = euler[2]  # Yaw angle

        # Navigate to the given pose
        self.go_to(x, y, theta)

if __name__ == '__main__':
    rospy.init_node('navigation', argv=sys.argv)
    nav = StretchNavigation()

    pose_file = "/path/to/pose_file.json"

    # Load the saved poses
    with open(pose_file, 'r') as f:
        poses = json.load(f)

    label = input("Enter the label of the landmark to navigate to: ")

    if label in poses:
        pose = poses[label]
        x = pose['position']['x']
        y = pose['position']['y']
        qx = pose['orientation']['x']
        qy = pose['orientation']['y']
        qz = pose['orientation']['z']
        qw = pose['orientation']['w']

        # Convert the landmark pose to 2D goal and navigate
        nav.convert_pose_to_2d_goal(x, y, qx, qy, qz, qw)
    else:
        rospy.loginfo("Label not found in saved poses.")
