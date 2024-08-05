import rospy
# from haptic_wayfinding.msg import FilteredScan, HapticFeedback #Original import with HapticFeedback msg type
from haptic_wayfinding.msg import FilteredScan, HapticRumble
# from std_msgs.msg import Float32, Bool
# from sensor_msgs.msg import LaserScan

import numpy as np

# from pyjoycon import JoyCon, get_R_id, get_L_id

""" Instructions to run script
1) roslaunch stretch_core stretch_driver.launch
2) roslaunch stretch_core rplidar.launch
3) rosservice call /switch_to_navigation_mode # allows Twist messages to be sent
4) python3 check_lidar_distance.py
5) rosrun stretch_core keyboard_teleop

"""

class CheckLidarDistance:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/filtered_scan', FilteredScan, self.scan_callback)

        #Original with HapticFeedback msg type
        # self.rumble_pub = rospy.Publisher('/haptic_blinker', HapticFeedback, queue_size=1)

        #New with HapticRumble msg type
        self.rumble_pub = rospy.Publisher('/haptic_rumble', HapticRumble, queue_size=1)
        print("Initialized Lidar Node")

        self.LIDAR_DISTANCE = 0.75

    def scan_callback(self, msg):
        # Priority list: right + left, front

        #Original with HapticFeedback msg type
        # pub_msg = HapticFeedback()

        #New with HapticRumble msg type
        pub_msg = HapticRumble()

        left_ranges = np.array(msg.left.ranges)
        left_ranges = left_ranges[left_ranges != np.inf] # remove inf values
        right_ranges = np.array(msg.right.ranges)
        right_ranges = right_ranges[right_ranges != np.inf] # remove inf values

        filtered_left_ranges = left_ranges[left_ranges < self.LIDAR_DISTANCE]
        if len(filtered_left_ranges) > len(left_ranges) // 2:
            print("LEFT RUMBLE", np.mean(filtered_left_ranges))
            pub_msg.left = True
            pub_msg.left_delay = np.mean(filtered_left_ranges) / self.LIDAR_DISTANCE
            pub_msg.left_volume = 1.0
        else:
            pub_msg.left = False
        
        filtered_right_ranges = right_ranges[right_ranges < self.LIDAR_DISTANCE]
        if len(filtered_right_ranges) > len(right_ranges) // 2:
            print("RIGHT RUMBLE", np.mean(filtered_right_ranges))
            pub_msg.right = True
            pub_msg.right_delay = np.mean(filtered_right_ranges) / self.LIDAR_DISTANCE
            pub_msg.right_volume = 1.0
        else:
            pub_msg.right = False

        # self.rumble_pub.publish(pub_msg)
        if pub_msg.left or pub_msg.right:
            pub_msg.front = False
            self.rumble_pub.publish(pub_msg)

        front_ranges = np.array(msg.front.ranges)
        front_ranges = front_ranges[front_ranges != np.inf] # remove inf values

        filtered_front_ranges = front_ranges[front_ranges < self.LIDAR_DISTANCE]
        if len(filtered_front_ranges) > len(front_ranges) // 2:
            print("FRONT RUMBLE", np.mean(filtered_front_ranges))
            pub_msg.front = True
            pub_msg.front_delay = np.mean(filtered_front_ranges) / self.LIDAR_DISTANCE
            pub_msg.front_volume = 1.0
        else:
            # print(len(front_ranges))
            pub_msg.front = False

        self.rumble_pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('check_lidar_distance')
    cld = CheckLidarDistance()
    rospy.spin()
