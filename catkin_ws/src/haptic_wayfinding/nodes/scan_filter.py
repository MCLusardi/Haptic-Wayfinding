import rospy
from numpy import linspace, inf
from math import sin
from sensor_msgs.msg import LaserScan
from haptic_wayfinding.msg import FilteredScan
import numpy as np
from copy import copy

class ScanFilter:
    """
    A class that implements a LaserScan filter that removes all of the points
    that are not in front of the robot.
    """
    def __init__(self):
        self.width = 1.0
        self.extent = self.width / 2.0
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('/filtered_scan', FilteredScan, queue_size=1)
        rospy.loginfo("Publishing the filtered_scan topic. Use RViz to visualize.")

    def callback(self,msg):
        """
        Callback function to deal with incoming LaserScan messages.
        :param self: The self reference.
        :param msg: The subscribed LaserScan message.

        :publishes msg: updated LaserScan message.
        """
        
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        # front scan filter
        front_msg = copy(msg)
        points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(front_msg.ranges, angles)]
        # points = [r * sin(theta) if (theta < -np.pi/4 or theta > np.pi/6) else inf for r,theta in zip(front_msg.ranges, angles)]
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(front_msg.ranges, points)]
        front_msg.ranges = new_ranges
        
        # left scan filter
        left_msg = copy(msg)
        # points = [r * sin(theta) if (theta > -2.0 and theta < -1.0) else inf for r,theta in zip(left_msg.ranges, angles)]
        # print the angles with the top 30 closest points as a tuple of (angle, distance)
        # print([(angles[i], left_msg.ranges[i]) for i in np.argsort(left_msg.ranges)[:30]])
        # print("Smallest angle:", min([(angles[i], left_msg.ranges[i]) for i in np.argsort(left_msg.ranges)[:50]], key=lambda x: x[0]))
        # print("Largest angle:", max([(angles[i], left_msg.ranges[i]) for i in np.argsort(left_msg.ranges)[:43]], key=lambda x: x[0]))
        points = [r * sin(theta) if (theta > -2.0 and theta < -1.0 and not (-1.15 < theta < -0.88)) else inf for r,theta in zip(left_msg.ranges, angles)]
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(left_msg.ranges, points)]
        left_msg.ranges = new_ranges
        
        
        # right scan filter
        right_msg = copy(msg)
        points = [r * sin(theta) if (theta > 1.0 and theta < 2.0) else inf for r,theta in zip(right_msg.ranges, angles)]
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(right_msg.ranges, points)]
        right_msg.ranges = new_ranges

        pub_msg = FilteredScan()
        pub_msg.front = front_msg
        pub_msg.left = left_msg
        pub_msg.right = right_msg
        self.pub.publish(pub_msg)
        
if __name__ == '__main__':
    rospy.init_node('scan_filter')
    node = ScanFilter()
    rospy.spin()