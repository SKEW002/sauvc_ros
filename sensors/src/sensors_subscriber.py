#!/usr/bin/python3

import rospy
import csv
import signal
import sys
import numpy
from std_msgs.msg import Float32MultiArray, Float32

csv_path = "/home/mecatron/sonar_exp/src/sensors/data.csv"

class sensors_subscriber:
	def __init__(self, csv_path):
		self.one_measurement = []
		self.data_list = []
		self.csv_path = csv_path
		signal.signal(signal.SIGINT, self.signal_handler)		# this is to handle the saving of csv file after pressing Ctrl+C

		rospy.init_node('sensors_node', anonymous=True)	
		rospy.Subscriber('/sensors/imu_topic', Float32MultiArray, self.imu_callback)	
		rospy.Subscriber('/sensors/sonar_topic', Float32MultiArray, self.sonar_callback_last)
		
		rospy.spin()
    
	def imu_callback(self, data):
		self.one_measurement = []
		rospy.loginfo("Received message: %s", data.data)
		#if type(data.data)==float:
		#	data.data = (data.data,)		#this is so that all the data received is of iterable type
		if not data.data:
			data.data = ("Nan","Nan","Nan","Nan","Nan","Nan")
		self.one_measurement.extend(data.data)
		#print(self.one_measurement)

	def sonar_callback_last(self, data):
		rospy.loginfo("Received message: %s", data.data)
		#if type(data.data)==float:
		#	data.data = (data.data,)		#this is so that all the data received is of iterable type
		if not data.data:
			data.data = ("Nan","Nan")
		self.one_measurement.extend(data.data)
		#print(self.one_measurement)
		self.data_list.append(self.one_measurement)
		self.one_measurement = []
  
	def save_to_csv(self):
		#print(self.data_list)
		with open(self.csv_path, mode='w') as file:
			writer = csv.writer(file)
			for data in self.data_list:
				writer.writerow(data)

	def signal_handler(self, sig, frame):		# clean-up operation after pressing Ctrl+C
		self.save_to_csv()
		print("\nData is saved to csv file")
		sys.exit(0)
            

if __name__ == '__main__':
	try:
		subscriber = sensors_subscriber(csv_path)
	except rospy.ROSInterruptException:
		subscriber.save_to_csv()

