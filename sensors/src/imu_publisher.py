#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32MultiArray

import time
import serial

port, baudrate = "/dev/ttyACM0", 9600

class imu_publisher(object):
	def __init__(self, arduinoPort, arduinoBaudrate):	
		self.connect_arduino(arduinoPort, arduinoBaudrate)
		rospy.init_node('imu_node', anonymous=True)
		self.pub = rospy.Publisher('/sensors/imu_topic', Float32MultiArray, queue_size=10)
		self.rate = rospy.Rate(1/0.15)
		self.msg = ""
		self.overall_data = []

	def connect_arduino(self, port, baudrate):
		for _ in range(3):
			try:
				self.arduinoData = serial.Serial(port, baudrate)
			except:
				print("IMU is still connecting to " + port)	
				time.sleep(1)
			else:
				print("IMU is successfully connected.")
				return
		print("IMU is not successfully connected to " + port)
		
	def get_and_publish_data(self):
		while not rospy.is_shutdown():
			while self.arduinoData.inWaiting()==0:
				pass
			data = self.arduinoData.readline()

    		#------ start of processing data ------#
			try:
				data = int(data)
			except:
				continue
    
			char = chr(data)
			if char!=";" and char!="|":
				self.msg = self.msg + char
			elif char==";":           # when we have received all quaternion info
				numbers = self.msg.split(sep=",")
				if len(numbers) == 3:
					try:
						numbers = list(int(i)/100 for i in numbers)
						#w,x,y,z = numbers
						#roll, pitch, yaw = numbers
						self.overall_data.extend(numbers)
						#print(w,x,y,z)
					except:
						pass
				self.msg = ""

			elif char=="|":         # when we have received all acceleration info
				numbers = self.msg.split(sep=",")
				if len(numbers) == 3:
					try:
						numbers = list(int(i)/100 for i in numbers)
						#x_acc, y_acc, z_acc = numbers
						self.overall_data.extend(numbers)
						#print(x_acc, y_acc, z_acc)
                
					except:
						pass

				if len(self.overall_data)==6:
					print(self.overall_data)
					ros_msg = Float32MultiArray()
					ros_msg.data = self.overall_data
					self.pub.publish(ros_msg)
					self.rate.sleep()

        		# reset the cycle    
				self.overall_data = []
				self.msg=""
				
if __name__ == '__main__':
	try:
		publisher = imu_publisher(arduinoPort=port, arduinoBaudrate=baudrate)       
		publisher.get_and_publish_data()
	except rospy.ROSInterruptException:
		pass
