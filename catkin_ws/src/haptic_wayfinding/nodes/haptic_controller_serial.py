import rospy
from std_msgs.msg import Float32, Bool
import serial
import time

from haptic_wayfinding.msg import HapticFeedback

# Node that will play sounds and updates delay from subscriber

class HapticController():
    def __init__(self, serial_connection):
        # Subscribes to the rumble flag. If true, play both channels 
        self.blinker_sub = rospy.Subscriber('/haptic_blinker', HapticFeedback, self.blinker_callback, queue_size=1)
        self.rumble_sub = rospy.Subscriber('/haptic_rumble', HapticFeedback, self.rumble_callback, queue_size=1)
        
        # Initialize serial connection
        self.ser = serial_connection
        
    def send_command(self, command):
        if command[-1] != '\n':
            command += '\n'
        self.ser.write(command.encode())
        time.sleep(0.1) # small time to send command
        response = self.ser.read(self.ser.inWaiting())
        return response
    
    def blinker_rumble(self, channel, duration, frequency, duty_cycle, num_cycles):
        pass
    
    def rumble_callback(self, msg):
        if msg.left:
            pass
        if msg.right:
            pass
        if msg.front:
            pass
        
if __name__ == '__main__':
    port = "" # TODO: define port on robot
    baud_rate = 115200
    ser = serial.Serial(port, baud_rate, timeout=1)
    rospy.init_node('haptic_controller')
    cld = HapticController(ser)
    rospy.spin()

