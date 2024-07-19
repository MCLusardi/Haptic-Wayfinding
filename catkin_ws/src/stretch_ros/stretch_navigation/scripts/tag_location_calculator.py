#!/usr/bin/env python

import rospy
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
import sys
from select import select
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

def tag_location_calculator():
    rospy.init_node('tag_location_calculator')

    listener = TransformListener()

    def publish_tag_location(event):
        try:
            # Get the robot's current pose in the map frame
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            tag_pose = PoseStamped()
            tag_pose.header.frame_id = "map"
            tag_pose.header.stamp = rospy.Time.now()
            tag_pose.pose.position.x = trans[0]
            tag_pose.pose.position.y = trans[1]
            tag_pose.pose.position.z = trans[2]
            tag_pose.pose.orientation.x = rot[0]
            tag_pose.pose.orientation.y = rot[1]
            tag_pose.pose.orientation.z = rot[2]
            tag_pose.pose.orientation.w = rot[3]

            # Publish the tag location
            tag_location_pub.publish(tag_pose)

        except Exception as e:
            rospy.logwarn("Failed to get robot pose: %s", str(e))

    # Publisher for tag location
    tag_location_pub = rospy.Publisher('/tag_location', PoseStamped, queue_size=10)

    # Call publish_tag_location at 1 Hz
    rospy.Timer(rospy.Duration(1), publish_tag_location)

    #Save pose when key '1' is pressed
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    settings = saveTerminalSettings()
    key = getKey(settings, key_timeout)
    print(key)
    # if key == '1':
    #     filename = os.path.join(self.pose_save_dir, str(self.img_counter) + '.pickle')
    #     trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
    #     x_, y_ = trans.transform.translation.x, trans.transform.translation.y
    #     x_r, y_r, z_r, w_r = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
    #     # print(x_r, y_r,z_r,w_r)
    #     with open(filename, 'wb') as f:
    #         pickle.dump([x_, y_, x_r, y_r, z_r, w_r], f, protocol=pickle.HIGHEST_PROTOCOL)
    #     self.img_counter = self.img_counter + 1


    rospy.spin()

if __name__ == '__main__':
    try:
        tag_location_calculator()
    except rospy.ROSInterruptException:
        pass
