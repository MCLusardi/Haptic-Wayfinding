import rospy
from sensor_msgs.msg import LaserScan

# from pyjoycon import JoyCon, get_R_id, get_L_id

class CheckLidarDistance:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
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

    def lidar_callback(self, msg):
        # TODO: check intensities and ranges (discard values not between range_min and range_max)
        print(msg, type(msg))
        min_distance = min(msg.ranges)
        if min_distance < 0.5:
            if min_distance < 0.2:
                # self.l_joycon.set_rumble(0.5, 0.5)
                # self.r_joycon.set_rumble(0.5, 0.5)
                print("rumble both joycons")

            else:
                if msg.ranges.index(min_distance) < len(msg.ranges) / 2:
                    # self.l_joycon.set_rumble(0.5, 0)
                    print("rumble left joycon")
                else:
                    # self.r_joycon.set_rumble(0.5, 0)
                    print("rumble right joycon")

        else:
            # self.l_joycon.set_rumble(0, 0)
            # self.r_joycon.set_rumble(0, 0)
            print("no rumble")



if __name__ == '__main__':
    rospy.init_node('check_lidar_distance')
    cld = CheckLidarDistance()
    rospy.spin()
