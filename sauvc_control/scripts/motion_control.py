#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Int16

def motion():
    motion = rospy.Publisher("/cmd_out/motion_topic", Int16, queue_size=10)
    rospy.init_node("motion_node", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        motion_value = int(raw_input("Input motion_value within range 1200-1800: "))
        if motion_value > 1800:
            motion_value=1800
        elif motion_value < 1200:
            motion_value=1200
        rospy.loginfo(motion_value)
        motion.publish(motion_value)
        rate.sleep()

if __name__ == '__main__':
    try:
        motion()
    except:
        pass