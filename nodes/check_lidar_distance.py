import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan

import numpy as np

# from pyjoycon import JoyCon, get_R_id, get_L_id

""" Instructions to run script
1) roslaunch stretch_core stretch_driver.launch
2) roslaunch stretch_core rplidar.launch
3) rosservice call /switch_to_navigation_mode # allows Twist messages to be sent
4) python3 check_lidar_distance.py

"""

class CheckLidarDistance:
    def __init__(self):
        # self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.front_lidar_sub = rospy.Subscriber('filtered_scan/front', LaserScan, self.front_scan_callback)
        self.left_lidar_sub = rospy.Subscriber('filtered_scan/left', LaserScan, self.left_scan_callback)
        self.right_lidar_sub = rospy.Subscriber('filtered_scan/right', LaserScan, self.right_scan_callback)
        print("Initialized Lidar Node")
        
        self.rumble_flag_pub = rospy.Publisher('/rumble_flag', Bool, queue_size=1)
        self.rumble_delay_pub = rospy.Publisher('/rumble_delay', Float32, queue_size=1)
        self.blinker_delay_pub = rospy.Publisher('/blinker_delay', Float32, queue_size=1)

        self.front_flag = True
        self.left_flag = True
        self.right_flag = True
        
        self.LIDAR_DISTANCE = 0.75

    def front_scan_callback(self, msg):
        # Given points, calculate distance from center of robot to point
        # If distance is less than 0.5m, rumble joycons

        ranges = np.array(msg.ranges)
        # remove inf values
        ranges = ranges[ranges != np.inf]
        # print(np.mean(ranges))
        if np.mean(ranges) < self.LIDAR_DISTANCE:
            print("RUMBLE", np.mean(ranges))
            self.front_flag = True
            self.rumble_flag_pub.publish(True)
            self.rumble_delay_pub.publish(np.mean(ranges)/self.LIDAR_DISTANCE)
        else:
            if self.front_flag:
                print("NO RUMBLE")
                self.front_flag = False
                # self.rumble_flag_pub.publish(False)
                self.rumble_delay_pub.publish(0)
            
    def left_scan_callback(self, msg):
        # Given points, calculate distance from center of robot to point
        # If distance is less than 0.5m, rumble joycons
        ranges = np.array(msg.ranges)
        ranges = ranges[ranges != np.inf] # remove inf values
        
        # add a edge case since the left side has the vertical manipulator belt 
        # TODO: instead of the length, change range of angles in scan_filter.py
        if len(ranges) >= 50 and np.mean(ranges) < self.LIDAR_DISTANCE:
            print("LEFT RUMBLE", np.mean(ranges))
            self.left_flag = True
            self.rumble_flag_pub.publish(False)
            self.rumble_delay_pub.publish(np.mean(ranges)/self.LIDAR_DISTANCE)
        else:
            if self.left_flag:
                print("NO LEFT RUMBLE")
                self.left_flag = False
                self.rumble_delay_pub.publish(0)
                
    def right_scan_callback(self, msg):
        # Given points, calculate distance from center of robot to point
        # If distance is less than 0.5m, rumble joycons

        ranges = np.array(msg.ranges)
        ranges = ranges[ranges != np.inf] # remove inf values
        if np.mean(ranges) < self.LIDAR_DISTANCE:
            print("RIGHT RUMBLE", np.mean(ranges))
            self.right_flag = True
            self.rumble_flag_pub.publish(False)
            self.rumble_delay_pub.publish(np.mean(ranges)/self.LIDAR_DISTANCE)
        else:
            if self.right_flag:
                print("NO RUMBLE")
                self.right_flag = False
                self.rumble_delay_pub.publish(0)
            

if __name__ == '__main__':
    rospy.init_node('check_lidar_distance')
    cld = CheckLidarDistance()
    rospy.spin()
