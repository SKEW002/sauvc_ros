#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Int16

def get_depth_input():
    depth_input = rospy.Publisher("/cmd_out/depth_input_topic", Int16, queue_size=10)
    rospy.init_node("depth_input_node", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        depth = int(raw_input("Input depth within range 1200-1800: "))
        if depth > 1800:
            depth=1800
        elif depth < 1200:
            depth=1200
        rospy.loginfo(depth)
        depth_input.publish(depth)
        rate.sleep()

if __name__ == '__main__':
    try:
        get_depth_input()
    except:
        pass
    