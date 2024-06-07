import rospy
from std_msgs.msg import String

class RumbleNode:
    def __init__(self):
        self.subscriber = rospy.Subscriber('instruction_topic', String, self.callback)
        self.publisher = rospy.Publisher('vibration_topic', String, queue_size=10)

    def callback(self, msg):
        instruction = msg.data
        if instruction == 'R':
            # Run rumble script for right controller
            self.publisher.publish('R')
        elif instruction == 'L':
            # Run rumble script for left controller
            self.publisher.publish('L')

if __name__ == '__main__':
    rospy.init_node('rumble_node')
    rumble_node = RumbleNode()
    rospy.spin()