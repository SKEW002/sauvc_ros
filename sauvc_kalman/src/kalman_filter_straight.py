#!/usr/bin/env python
from filterpy.kalman import KalmanFilter
import time
import rospy
from sauvc.msg import Sonar
from sensor_msgs.msg import Imu
import numpy as np
from std_msgs.msg import Float32, Float64


class KF():
    def __init__(self):
        rospy.init_node('kalman_filter')

        
        rospy.Subscriber(
            '/imu', Imu, self.cb_imu, queue_size=1)                                      

        self.Kalman_filter_x = rospy.Publisher(
            '/kf_output_x', Float64, queue_size=1)   
        self.Kalman_filter_y = rospy.Publisher(
            '/kf_output_y', Float32, queue_size=1)                       

        self.Imu_Msg = Imu() 
        self.Sonar_Msg = Sonar() 
        self.msg_x = Float64()       
        self.msg_y = Float32()
        self.previous_distance = None
        self.previous_state_time = None
        self.previous_accel = None                              #change to np.array([0],[0])    2 dimensional now
        self.previous_sonar_time = None
        self.previous_sonar_distance=None
        self.imu_bias_list = []
        self.imu_acc = []
        #self.imu_bias = np.zeros((1,2))                                   ##2D?
        self.imu_bias = np.array([[0,0]])
        self.wall = []
        self.old_v_x = np.array([[0,0]])
        self.v_x = np.array([[0,0]])
        self.sonar_filtered = []
        self.acc_sonar = []
        self.tof = []
        self.velocity = []
        self.position_x = []
        self.ori_vel = []
        #time = []
        self.start = False
        self.first_distance = [0,0]
        



        self.kf = KalmanFilter(dim_x=2, dim_z=2,dim_u=1)
        self.kf.P[0, 0] = 0.001
        self.kf.P[1, 1] = 0.001
        self.kf.x = np.array([[0,0],[0]])
        self.kf.Q = np.array([[1,0],
                             [0 ,1]])
        self.kf.H = np.array([[1., 0.],
                             [0., 1.]])

        self.prev_time = rospy.Time(0)
        self.dt = 0
        

       
        rospy.spin()


    def cb_imu(self, Imu_Msg):
        #if previous_accel == None:
            #previous_state_time = df["time"][i]                            
            #previous_accel = df["accel x"][i]
        if self.start == False:
            self.first_distance[0] = self.Sonar_Msg.distance          #fixed starting x
            self.first_distance[1] = self.Sonar_Msg.distance2         #fixed starting y
            self.start = True

        else:                                                            #predict using imu
            #self.previous_accel = np.array([(Imu_Msg.linear_acceleration.x*np.sin(np.pi/4) - Imu_Msg.linear_acceleration.y*np.cos(np.pi/4)) , -(Imu_Msg.linear_acceleration.x*np.sin(np.pi/4) + Imu_Msg.linear_acceleration.y*np.cos(np.pi/4))])
            self.previous_accel = np.array([[(Imu_Msg.linear_acceleration.x) , (Imu_Msg.linear_acceleration.y)]])
            #else:
                
                #dt = df["time"][i] - previous_state_time
            
            self.dt = rospy.Time.now() - self.prev_time           
            self.kf.F = np.array([[1., self.dt],
                            [0., 1.]])
            self.kf.B = np.array([[0.5*self.dt.secs*self.dt.secs],
                            [       self.dt]])
            #u = np.array([previous_accel-imu_bias])              #add another 1 of prev acc_acc - imu_bias? or change imu_bias to 2 dim
            u = self.previous_accel - self.imu_bias                          #u = [[acc_x,acc_y]], previously u = [[acc_x]]

            #for acc in u:  #kf cannot predict 2D array
            for i in range(0,2):
                self.kf.predict(np.array(u[i]))
                
            self.imu_acc.append(self.previous_accel-self.imu_bias)                 #2 dim?
            self.imu_bias_list.append(self.imu_bias)
                #previous_accel =-(Imu_Msg.linear_acceleration.x[0]*np.sin(np.pi/4)+Imu_Msg.linear_acceleration.y[0]*np.cos(np.pi/4)) 
                #2 dim?
                #previous_state_time = df["time"][i]                  





                                                                        #update using sonar
            #self.first_distance = Sonar_Msg.distance[0]
                #first_tof_distance = df["tof_distance"][i]
            self.previous_sonar_distance=np.zeros(1,2)
                #previous_sonar_distance_x = 0
                #previous_sonar_distance_y = 0
                #previous_sonar_time = df["time"][i]
                #start = True
            #sonar_lfilter(0)
            print("here")                                                       #z is current measurement
            z = np.array([-self.Sonar_Msg.distance[0]-self.first_distance[0], self.Sonar_Msg.distance2[0]-self.first_distance[1]])
            
            old_v_x = self.v_x
            v_x = (z-self.previous_sonar_distance)/(self.dt.secs)
            #y = kf.residual_of(np.array([z,v_x]))               #y = z - Hx                
            #y = measurement - predicted_measurement
            #y = residual/difference between measured and predicted
            y = z - np.dot(self.kf.H, self.kf.x[0])                     #H = np.array([[1., 0.],        x[0] = [X,Y]    so Hx = 2 by 1 matrix 
            if np.all(np.abs(v_x)) < 0.1: # to filter out outlier                       [0.,1]])
                try:
                    if np.all(np.abs(y[0])) < 1 and self.Sonar_Msg.confidence == 100:               #if  difference of measured(by sonar) & prediction is small, proceed to use sonar measurement                  
                        self.kf.R = np.array([[0.01,0   ],
                                        [0   ,0.01]])
                        self.kf.update(np.array([z,v_x]))
                    else: #only update velocity here
                        self.kf.R = np.array([[10,0   ],
                                        [0   ,10]])
                        self.kf.update(np.array([self.kf.x[0],v_x]))                  ##still using predicted measurement (measured by IMU)
                except ValueError:
                    print("error")
 
            #Estimate accelerometer bias
            if self.previous_accel != None:
                if np.all(np.abs(v_x - old_v_x)) < 0.02:
                    self.imu_bias = self.previous_accel
            self.acc_sonar.append(v_x - old_v_x)
            self.previous_sonar_distance=z
            #previous_sonar_distance_x = x
            #previous_sonar_distance_y = y_yep
            #previous_sonar_time = df["time"][i]
            #tof.append(tof_distance)
            self.wall.append(self.Sonar_Msg.distance[0])
            self.velocity.append(self.kf.x[1])                               #x = states = [[position],[velocity]]
            self.ori_vel.append(self.v_x)
            #print(kf.x[0])
            self.position_x.append(self.kf.x[0])
            #time.append(df["time"][i])
            self.prev_time = rospy.Time.now()

            
            self.msg_x = self.kf.x[0][0]
            self.msg_y = self.kf.x[0][1]
            self.Kalman_filter_x.publish(self.msg_x)
            self.Kalman_filter_y.publish(self.msg_y)
 
  #  def cb_sonar(self, Sonar_Msg):
if __name__ == '__main__':
    try:
        KF()
    except rospy.ROSInterruptException:
        pass







