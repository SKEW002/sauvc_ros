#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

class IMU_Filter:
    def __init__(self, sample):
        rospy.Subscriber('/zedm/zed_node/imu/data',Imu , self.imuCallback)
        self.start_imu = False

        self.imu_pub = rospy.Publisher('/cmd_out/imu_data', Float32MultiArray , queue_size=1)


        self.imu_data = [[0 for i in range(sample)] for j in range(6)]

        self.imu_msg = Float32MultiArray()
        self.imu_msg.data = [0 for i in range(6)]


    def imuCallback(self, msg):
        self.raw_imu = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        self.start_imu = True

    def filter(self):
        '''
        Exponential Moving Average (EMA)
        '''
        ema = 0
        alpha = 0.125
        denominator = 0

        for j in range(6):
            for i in range(len(self.imu_data[j])):
                denominator += (1-alpha)**i

            
            for i in range(len(self.imu_data[j])):
                ema += ((self.imu_data[j][len(self.imu_data[j]) - i - 1]) *(1-alpha)**i) / denominator

            self.imu_msg.data[j] = ema
            
        self.imu_msg.data = self.raw_imu



if __name__ == '__main__':
    imu_index = 0
    sample = 10

    rospy.init_node('imu_filter')
    imu_filter = IMU_Filter(sample)

    rate = rospy.Rate(225)
    while not rospy.is_shutdown(): 
        if imu_filter.start_imu:

            if imu_index == sample:
                imu_filter.filter()
                imu_filter.imu_pub.publish(imu_filter.imu_msg)
                imu_index = 0
            else:
                for j in range(6):
                    imu_filter.imu_data[j][imu_index] = imu_filter.raw_imu[j]
                imu_index += 1

            rate.sleep()