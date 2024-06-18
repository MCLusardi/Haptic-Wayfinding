import rospy
from numpy import linspace, inf
from math import sin
from sensor_msgs.msg import LaserScan
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
        self.front_pub = rospy.Publisher('filtered_scan/front', LaserScan, queue_size=10)
        self.left_pub = rospy.Publisher('filtered_scan/left', LaserScan, queue_size=10)
        self.right_pub = rospy.Publisher('filtered_scan/right', LaserScan, queue_size=10)
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
        self.front_pub.publish(front_msg)
        
        # left scan filter
        left_msg = copy(msg)
        points = [r * sin(theta) if (theta > -2.0 and theta < -0.5) else inf for r,theta in zip(left_msg.ranges, angles)]
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(left_msg.ranges, points)]
        left_msg.ranges = new_ranges
        self.left_pub.publish(left_msg)
        
        
        # right scan filter
        right_msg = copy(msg)
        points = [r * sin(theta) if (theta > 0.5 and theta < 2.0) else inf for r,theta in zip(right_msg.ranges, angles)]
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(right_msg.ranges, points)]
        right_msg.ranges = new_ranges
        self.right_pub.publish(right_msg)
        

if __name__ == '__main__':
    rospy.init_node('scan_filter')
    ScanFilter()
    rospy.spin()