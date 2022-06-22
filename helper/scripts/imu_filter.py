#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from math import pi
import time

class IMU_Filter:
    def __init__(self, sample):
        rospy.Subscriber('/zedm/zed_node/imu/data',Imu , self.imuCallback)
        self.start_imu = False

        self.ang_vel_pub = rospy.Publisher('/cmd_out/ang_vel', Float32 , queue_size=1)

        self.gx_pub = rospy.Publisher('/cmd_out/gx', Float32 , queue_size=1)
        self.gy_pub = rospy.Publisher('/cmd_out/gy', Float32 , queue_size=1)
        self.oz_pub = rospy.Publisher('/cmd_out/oz', Float32 , queue_size=1)
        self.ow_pub = rospy.Publisher('/cmd_out/ow', Float32 , queue_size=1)

        
        self.ang_vel_msg = Float32()
        self.ang_vel_array = [0 for i in range(sample)]
        self.gx_msg = Float32()
        self.gy_msg = Float32()
        self.oz_msg = Float32()
        self.ow_msg = Float32()


    def imuCallback(self, msg):
        self.ang_vel = int(msg.angular_velocity.z * 180 / pi)
        self.gx_msg.data = msg.linear_acceleration.x
        self.gy_msg.data = msg.linear_acceleration.y

        self.oz_msg.data = msg.orientation.z
        self.ow_msg.data = msg.orientation.w

        self.start_imu = True

    def filter(self):
        '''
        Exponential Moving Average (EMA)
        '''
        ema = 0
        alpha = 0.125
        denominator = 0

        for i in range(len(self.ang_vel_array)):
            denominator += (1-alpha)**i

        
        for i in range(len(self.ang_vel_array)):
            ema += ((self.ang_vel_array[len(self.ang_vel_array) - i - 1]) *(1-alpha)**i) / denominator

        return ema


if __name__ == '__main__':
    imu_index = 0
    sample = 10

    rospy.init_node('imu_filter')
    imu_filter = IMU_Filter(sample)

    rate = rospy.Rate(225)
    while not rospy.is_shutdown(): 
        if imu_filter.start_imu:

            # if imu_index == sample:
            #     imu_filter.ang_vel_msg.data = imu_filter.filter()
            #     imu_filter.ang_vel_pub.publish(imu_filter.ang_vel_msg)
            #     imu_index = 0
            # else:
            #     imu_filter.ang_vel_array[imu_index] = imu_filter.ang_vel
            #     imu_index += 1

            imu_filter.gx_pub.publish(imu_filter.gx_msg)
            imu_filter.gy_pub.publish(imu_filter.gy_msg)
            imu_filter.oz_pub.publish(imu_filter.oz_msg)
            imu_filter.ow_pub.publish(imu_filter.ow_msg)

            rate.sleep()