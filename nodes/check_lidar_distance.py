import rospy
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
        # self.l_joycon = JoyCon(get_L_id())
        # self.l_joycon.set_rumble(0, 0)
        # self.l_joycon.set_leds([0, 0, 0, 0])
        # self.l_joycon.set_player_lights(1)
        # self.l_joycon.set_home_light(1)
        # self.l_joycon.set_rumble(0, 0)
        
        # self.r_joycon = JoyCon(get_R_id())
        # self.r_joycon.set_rumble(0, 0)
        # self.r_joycon.set_leds([0, 0, 0, 0])
        # self.r_joycon.set_player_lights(1)
        # self.r_joycon.set_home_light(1)
        # self.r_joycon.set_rumble(0, 0)
        self.front_flag = True
        self.left_flag = True
        self.right_flag = True

    def front_scan_callback(self, msg):
        # Given points, calculate distance from center of robot to point
        # If distance is less than 0.5m, rumble joycons

        ranges = np.array(msg.ranges)
        # remove inf values
        ranges = ranges[ranges != np.inf]
        # print(np.mean(ranges))
        if np.mean(ranges) < 0.75:
            print("RUMBLE", np.mean(ranges))
            self.front_flag = True
            # self.l_joycon.set_rumble(1, 1)
            # self.r_joycon.set_rumble(1, 1)
        else:
            if self.front_flag:
                print("NO RUMBLE")
                self.front_flag = False
            pass
            # self.l_joycon.set_rumble(0, 0)
            # self.r_joycon.set_rumble(0, 0)
            
    def left_scan_callback(self, msg):
        # Given points, calculate distance from center of robot to point
        # If distance is less than 0.5m, rumble joycons
        ranges = np.array(msg.ranges)
        # remove inf values
        ranges = ranges[ranges != np.inf]
        # add a edge case since the left side has the vertical manipulator belt 
        if len(ranges) >= 50 and np.mean(ranges) < 0.75:
            print("LEFT RUMBLE", np.mean(ranges))
            self.left_flag = True
            # self.l_joycon.set_rumble(1, 1)
            # self.r_joycon.set_rumble(1, 1)
        else:
            if self.left_flag:
                print("NO LEFT RUMBLE")
                self.left_flag = False
            pass
        
    def right_scan_callback(self, msg):
        # Given points, calculate distance from center of robot to point
        # If distance is less than 0.5m, rumble joycons

        ranges = np.array(msg.ranges)
        # remove inf values
        ranges = ranges[ranges != np.inf]
        # print(np.mean(ranges))
        if np.mean(ranges) < 0.75:
            print("RIGHT RUMBLE", np.mean(ranges))
            self.right_flag = True
            # self.l_joycon.set_rumble(1, 1)
            # self.r_joycon.set_rumble(1, 1)
        else:
            if self.right_flag:
                print("NO RUMBLE")
                self.right_flag = False
            pass




if __name__ == '__main__':
    rospy.init_node('check_lidar_distance')
    cld = CheckLidarDistance()
    rospy.spin()
