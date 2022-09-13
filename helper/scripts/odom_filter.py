#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


class Odom_Filter:
    def __init__(self, sample):
        rospy.Subscriber('/zedm/zed_node/odom',Odometry , self.odomCallback)
        self.start_odom = False

        self.odom_pub = rospy.Publisher('/cmd_out/odom_data', Float32MultiArray , queue_size=1)


        self.odom_data = [[0 for i in range(sample)] for j in range(6)]

        self.odom_msg = Float32MultiArray()
        self.odom_msg.data = [0 for i in range(6)]


    def odomCallback(self, msg):
        self.raw_odom = [1.0, 1.0, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        self.start_odom = True

    def filter(self):
        '''
        Exponential Moving Average (EMA)
        '''
        ema = 0
        alpha = 0.125
        denominator = 0

        for j in range(6):
            for i in range(len(self.odom_data[j])):
                denominator += (1-alpha)**i

            
            for i in range(len(self.odom_data[j])):
                ema += ((self.odom_data[j][len(self.odom_data[j]) - i - 1]) *(1-alpha)**i) / denominator

            self.odom_msg.data[j] = ema
            
        self.odom_msg.data = self.raw_odom



if __name__ == '__main__':
    odom_index = 0
    sample = 10

    rospy.init_node('odom_filter')
    odom_filter = Odom_Filter(sample)

    rate = rospy.Rate(225)
    while not rospy.is_shutdown(): 
        if odom_filter.start_odom:

            if odom_index == sample:
                odom_filter.filter()
                odom_filter.odom_pub.publish(odom_filter.odom_msg)
                odom_index = 0
            else:
                for j in range(6):
                    odom_filter.odom_data[j][odom_index] = odom_filter.raw_odom[j]
                odom_index += 1

            rate.sleep()