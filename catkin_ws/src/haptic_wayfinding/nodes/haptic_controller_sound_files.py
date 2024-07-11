import rospy
from std_msgs.msg import Float32, Bool
import pygame
import time

from haptic_wayfinding.msg import HapticFeedback

# Node that will play sounds and updates delay from subscriber

class HapticController():
    def __init__(self):
        # Subscribes to the rumble flag. If true, play both channels 
        # self.rumble_flag_sub = rospy.Subscriber('/rumble_flag', Bool, self.rumble_flag_callback, queue_size=1)
        # # Subscribes to the delay of the rumble, between -1 and 1
        # self.rumble_sub = rospy.Subscriber('/rumble_delay', Float32, self.rumble_callback, queue_size=1)
        # # Subscribes to the delay of the blinker, between -1 and 1
        self.blinker_sub = rospy.Subscriber('/haptic_blinker', HapticFeedback, self.blinker_callback, queue_size=1)
        self.rumble_sub = rospy.Subscriber('/haptic_rumble', HapticFeedback, self.rumble_callback, queue_size=1)
        # Initialize pygame mixer
        pygame.mixer.init()
        self.sleep_factor = 0.75
    
    def rumble_callback(self, msg):
        # print("RUMBLE", msg)
        if msg.left:
            # play sound for left rumble
            pygame.mixer.music.load('../data/rumble_left.wav')
            pygame.mixer.music.play()
            pygame.mixer.music.set_volume(msg.left_volume)
            time.sleep(msg.left_delay * self.sleep_factor)
            pygame.mixer.music.stop()
            time.sleep(msg.left_delay * self.sleep_factor)

        if msg.right:
            # play sound for right rumble
            pygame.mixer.music.load('../data/rumble_right.wav')
            pygame.mixer.music.play()
            pygame.mixer.music.set_volume(msg.right_volume)
            time.sleep(msg.right_delay * self.sleep_factor)
            pygame.mixer.music.stop()
            time.sleep(msg.right_delay * self.sleep_factor)
            
        if msg.front:
            # play sound for front rumble
            pygame.mixer.music.load('../data/rumble_both.wav')
            pygame.mixer.music.play()
            pygame.mixer.music.set_volume(msg.front_volume)
            time.sleep(msg.front_delay * self.sleep_factor)
            pygame.mixer.music.stop()
            time.sleep(msg.front_delay * self.sleep_factor)

        # stop sound
        pygame.mixer.music.stop()

    def blinker_callback(self, msg):
        if msg.left:
            # play sound for left blinker
            pygame.mixer.music.load('../data/blinker_left.wav')
            pygame.mixer.music.play()
            pygame.mixer.music.set_volume(msg.left_volume)
            time.sleep(msg.left_delay * self.sleep_factor)
            pygame.mixer.music.stop()
            time.sleep(msg.left_delay * self.sleep_factor)
        elif msg.right:
            # play sound for right blinker
            pygame.mixer.music.load('../data/blinker_right.wav')
            pygame.mixer.music.play()
            pygame.mixer.music.set_volume(msg.right_volume)
            time.sleep(msg.right_delay * self.sleep_factor)
            pygame.mixer.music.stop()
            time.sleep(msg.right_delay * self.sleep_factor)
            
        # stop sound
        pygame.mixer.music.stop()

if __name__ == '__main__':
    rospy.init_node('haptic_controller')
    cld = HapticController()
    rospy.spin()

