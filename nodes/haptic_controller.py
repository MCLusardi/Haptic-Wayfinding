import rospy
from std_msgs.msg import Float32, Bool
import pygame
import time

# Node that will play sounds and updates delay from subscriber

class HapticController():
    def __init__(self):
        # Subscribes to the rumble flag. If true, play both channels 
        self.rumble_flag_sub = rospy.Subscriber('/rumble_flag', Bool, self.rumble_flag_callback, queue_size=1)
        # Subscribes to the delay of the rumble, between -1 and 1
        self.rumble_sub = rospy.Subscriber('/rumble_delay', Float32, self.rumble_callback, queue_size=1)
        # Subscribes to the delay of the blinker, between -1 and 1
        self.blinker_sub = rospy.Subscriber('/blinker_delay', Float32, self.blinker_callback, queue_size=1)
        
        # Initialize pygame mixer
        pygame.mixer.init()
        
        self.rumble_flag = False
        
    def rumble_flag_callback(self, msg):
        self.rumble_flag = msg.data
        print("FLIPPED RUMBLE FLAG", self.rumble_flag)
        
    def rumble_callback(self, msg):
        print("RUMBLE", msg.data)
        if self.rumble_flag:
            # play sound from front
            pygame.mixer.music.load('data/rumble_both.wav')
            pygame.mixer.music.play()
            time.sleep(abs(msg.data))
        else:
            # play sound from left or right side based on sign
            if msg.data < 0:
                pygame.mixer.music.load('data/rumble_left.wav')
                pygame.mixer.music.play()
                time.sleep(abs(msg.data))
            elif msg.data > 0:
                pygame.mixer.music.load('data/rumble_right.wav')
                pygame.mixer.music.play()
                time.sleep(abs(msg.data))
            else:
                # stop sound
                pygame.mixer.music.stop()
    
    def blinker_callback(self, msg):
        print("BLINKER", msg.data)
        if msg.data < 0:
            # play sound for left blinker
            pygame.mixer.music.load('data/left_blinker_sound.wav')
            pygame.mixer.music.play()
            time.sleep(abs(msg.data))
        elif msg.data > 0:
            # play sound for right blinker
            pygame.mixer.music.load('data/right_blinker_sound.wav')
            pygame.mixer.music.play()
            time.sleep(abs(msg.data))
        else:
            # stop sound
            pygame.mixer.music.stop()