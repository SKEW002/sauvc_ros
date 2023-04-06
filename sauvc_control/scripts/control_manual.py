#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Int16


import math
import time

class Control:
    def __init__(self):
        rospy.Subscriber("/cmd_out/depth_input_topic", Int16, self.depthCallback)
        rospy.Subscriber("/cmd_out/motion_topic", Int16, self.motionCallback)

        self.pwm_pub = rospy.Publisher("/cmd_out/pwm", Int16MultiArray, queue_size=10)
        self.pwm_msg = Int16MultiArray()

        self.pwm = [1500 for i in range(8)]  # thruster 1-8

    def depthCallback(self, msg):
        self.depth_pwm = int(msg.data)

    def motionCallback(self, msg):
        self.motion_pwm = int(msg.data)

    def publish_pwm(self):
        # thrusters 5-8 is to push the bot down
        for i in range(4,8):
            self.pwm[i] = self.depth_pwm
        # thrusters 1-4 is to control the bot motion
        for i in range(4):
            self.pwm[i] = self.motion_pwm
        self.pwm_msg.data = self.pwm

        self.pwm_pub.publish(self.pwm_msg)

    

