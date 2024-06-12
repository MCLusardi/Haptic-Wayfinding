import rospy
from nav_msgs.msg import Odometry
from pyjoycon import JoyCon, get_R_id, get_L_id

class VibrationNode:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Get JoyCon IDs and create instances
        joycon_id_right = get_R_id()
        joycon_id_left = get_L_id()

        print(joycon_id_right)
        print(joycon_id_left)

        self.joyconR = RumbleJoyCon(*joycon_id_right)
        self.joyconL = RumbleJoyCon(*joycon_id_left)

    def odom_callback(self, msg):
        # Process odometry data to detect turns
        if msg.twist.twist.angular.z > 0.5:  # adjust this value to detect turns
            self.trigger_rumble('R')  # send rumble command for right turn
        elif msg.twist.twist.angular.z < -0.5:  # adjust this value to detect turns
            self.trigger_rumble('L')  # send rumble command for left turn

    def trigger_rumble(self, direction):
        freq = 320  # Set frequency value
        amp = 0.5  # Set amplitude value

        data = RumbleData(freq / 2, freq, amp)
        b = data.GetData()

        if direction == 'R':
            self.joyconR._send_rumble(b)
        elif direction == 'L':
            self.joyconL._send_rumble(b)

        rospy.sleep(1.5)  # Rumble duration

if __name__ == '__main__':
    rospy.init_node('vibration_node')
    vibration_node = VibrationNode()
    rospy.spin()
