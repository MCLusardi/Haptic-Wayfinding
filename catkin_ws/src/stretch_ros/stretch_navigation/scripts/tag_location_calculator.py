#!/usr/bin/env python

import rospy
from tf import TransformListener
from geometry_msgs.msg import PoseStamped

def tag_location_calculator():
    rospy.init_node('tag_location_calculator')

    listener = TransformListener()

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

    # Publisher for tag location
    tag_location_pub = rospy.Publisher('/tag_location', PoseStamped, queue_size=10)

    # Call publish_tag_location at 1 Hz
    rospy.Timer(rospy.Duration(1), publish_tag_location)

    rospy.spin()

if __name__ == '__main__':
    try:
        tag_location_calculator()
    except rospy.ROSInterruptException:
        pass
