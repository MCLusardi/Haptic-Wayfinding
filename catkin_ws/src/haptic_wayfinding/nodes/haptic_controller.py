import rospy
from std_msgs.msg import Float32, Bool
import pygame
import time
from haptic_wayfinding.msg import HapticRumble

from haptic_wayfinding.msg import HapticFeedback

# Node that will play sounds and updates delay from subscriber

class HapticController():
    def __init__(self):
        # Subscribes to the rumble flag. If true, play both channels 
        # self.rumble_flag_sub = rospy.Subscriber('/rumble_flag', Bool, self.rumble_flag_callback, queue_size=1)
        # # Subscribes to the delay of the rumble, between -1 and 1
        # self.rumble_sub = rospy.Subscriber('/rumble_delay', Float32, self.rumble_callback, queue_size=1)
        # # Subscribes to the delay of the blinker, between -1 and 1

        #Original code
        # self.blinker_sub = rospy.Subscriber('/haptic_blinker', HapticFeedback, self.blinker_callback, queue_size=1)
        # self.rumble_sub = rospy.Subscriber('/haptic_rumble', HapticFeedback, self.rumble_callback, queue_size=1)
        # self.blinker_sub = rospy.Subscriber('/blinker_delay', Float32, self.blinker_callback, queue_size=1)

        #new code
        self.rumble_sub = rospy.Subscriber('/haptic_rumble', HapticRumble, self.rumble_callback, queue_size=1)
        self.landmark_sub = rospy.Subscriber('/landmark_in_range', Bool, self.landmark_callback, queue_size=1)

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

    #Old blinker callback
    # def blinker_callback(self, msg):
    #     if msg.left:
    #         # play sound for left blinker
    #         pygame.mixer.music.load('../data/blinker_left.wav')
    #         pygame.mixer.music.play()
    #         pygame.mixer.music.set_volume(msg.left_volume)
    #         time.sleep(msg.left_delay * self.sleep_factor)
    #         pygame.mixer.music.stop()
    #         time.sleep(msg.left_delay * self.sleep_factor)
    #     elif msg.right:
    #         # play sound for right blinker
    #         pygame.mixer.music.load('../data/blinker_right.wav')
    #         pygame.mixer.music.play()
    #         pygame.mixer.music.set_volume(msg.right_volume)
    #         time.sleep(msg.right_delay * self.sleep_factor)
    #         pygame.mixer.music.stop()
    #         time.sleep(msg.right_delay * self.sleep_factor)
            
    #     # stop sound
    #     pygame.mixer.music.stop()

    #New code
    def blinker_callback(self, msg):
        print("BLINKER", msg.data)
        if msg.data < 0:
            # play sound for left blinker
            pygame.mixer.music.load('../data/left_blinker_sound.wav')
            pygame.mixer.music.play()
            time.sleep(abs(msg.data))
        elif msg.data > 0:
            # play sound for right blinker
            pygame.mixer.music.load('../data/right_blinker_sound.wav')
            pygame.mixer.music.play()
            time.sleep(abs(msg.data))
        else:
            # stop sound
            pygame.mixer.music.stop()
    
    def landmark_callback(self, msg):
        if msg.data:
            marker_id = rospy.get_param('detected_marker_id', -1)
            self.play_rumble(marker_id)
    
    def play_rumble(self, marker_id):
        if 245 <= marker_id <= 249:
            if marker_id == 245:
                sound_file = '../data/rumble_both.wav'
            elif marker_id == 246:
                sound_file = '../data/rumble_left.wav'
            elif marker_id == 247:
                sound_file = '../data/rumble_right.wav'
            elif marker_id == 248:
                sound_file = '../data/rumble_both.wav'
            else:
                sound_file = '../data/rumble_left.wav'  

            pygame.mixer.music.load(sound_file)
            pygame.mixer.music.play()
            pygame.mixer.music.set_volume(1.0)
            time.sleep(0.5 * self.sleep_factor)
            pygame.mixer.music.stop()
            time.sleep(0.5 * self.sleep_factor)


if __name__ == '__main__':
    rospy.init_node('haptic_controller')
    cld = HapticController()
    rospy.spin()

