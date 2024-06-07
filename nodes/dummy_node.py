import rospy
from std_msgs.msg import String

class DummyNode:
    def __init__(self):
        self.publisher = rospy.Publisher('direction_topic', String, queue_size=10)
        self.timer = rospy.Timer(1.0, self.publish_direction)

    def publish_direction(self, event):
        direction = 'R' if rospy.get_time() % 2 == 0 else 'L'
        self.publisher.publish(direction)

if __name__ == '__main__':
    rospy.init_node('dummy_node')
    dummy_node = DummyNode()
    rospy.spin()