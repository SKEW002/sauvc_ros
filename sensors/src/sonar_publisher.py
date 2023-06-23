#!/usr/bin/python3

from brping import Ping1D
import time

import rospy
from std_msgs.msg import Float32MultiArray

port1 = "/dev/ttyUSB0"
port2 = "/dev/ttyUSB1"
baudrate = 115200
speed_of_sound_in_air = 340000	#mm/s
speed_of_sound_in_water = 1500000 #mm/s


class pinger_publisher:
	def __init__(self, port1, port2, baudrate, speed_of_sound):
		if port1!=None:
			self.pinger1 = Ping1D()
			self.connect_pinger(self.pinger1, port1, "Pinger 1")
			self.pinger1.set_speed_of_sound(speed_of_sound)
		else:
			self.pinger1 = None

		if port2!=None:
			self.pinger2 = Ping1D()
			self.connect_pinger(self.pinger2, port2, "Pinger 2")
			self.pinger2.set_speed_of_sound(speed_of_sound)
		else:
			self.pinger2 = None

		self.measurements = []
		rospy.init_node('sonar_node', anonymous=True)
		self.pub = rospy.Publisher('/sensors/sonar_topic', Float32MultiArray, queue_size=10)
		self.rate = rospy.Rate(1/0.15)

	def connect_pinger(self, pinger, port, name):
		while True:
			try:
				pinger.connect_serial(port, baudrate)
			except:
				print(name + " is still connecting.")
				time.sleep(1)
			else:
				print(name + " is successfully connected.")
				return
		print(name + " is not successfully connected.")

	def get_distance_reading(self, pinger):
		try:
			data = pinger.get_distance()
		except:
			return
		try:
			return data["distance"]
		except:
			return
	
	def publish_data(self):
		while not rospy.is_shutdown():
			distance1 = self.get_distance_reading(self.pinger1)		# this is error-proof regardless of whether pinger is present thanks to try-except-else block in get_distance_reading()
			distance2 = self.get_distance_reading(self.pinger2)
				
			self.measurements.extend((distance1, distance2))

			print(self.measurements)
			ros_msg = Float32MultiArray()
			ros_msg.data = self.measurements
			self.pub.publish(ros_msg)
			self.rate.sleep()

			self.measurements = []

if __name__ == '__main__':
	try:
		pinger = pinger_publisher(port1=port1, port2=port2, baudrate=baudrate, speed_of_sound=speed_of_sound_in_air)
		pinger.publish_data()
	except rospy.ROSInterruptException:
		pass

