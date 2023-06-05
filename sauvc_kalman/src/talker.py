#!/usr/bin/env python
# license removed for brevity
import rospy
from sauvc.msg import Sonar
from sensor_msgs.msg import Imu

sonar_msg = Sonar()
imu_msg = Imu()
   
def talker():
    rospy.init_node('talker', anonymous=True)
    sonar_pub = rospy.Publisher('sonar', Sonar, queue_size = 1)
    imu_pub = rospy.Publisher('imu', Imu, queue_size = 1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        data1 = 1
        data2 = 2
        data3 = 3
        data4 = 4
        sonar_msg.distance = data1
        sonar_msg.distance2 = data2
        imu_msg.linear_acceleration.x = data3
        imu_msg.linear_acceleration.y = data4
        sonar_pub.publish(sonar_msg)
        imu_pub.publish(imu_msg)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass