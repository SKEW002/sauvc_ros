#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Int16MultiArray, String, UInt16
from sensor_msgs.msg import Imu


import math
import time

class Control:
    def __init__(self):

        # ROS subscribe
        rospy.Subscriber('/cmd_out/imu_data',Float32MultiArray , self.imuCallback)
        rospy.Subscriber('/cmd_out/depth',UInt16 , self.depthCallback)
        rospy.Subscriber('/cmd_out/target_angle',Float32 , self.targetAngleCallback)
        rospy.Subscriber('/cmd_out/motion',String , self.motionCallback)

        # ROS publish
        self.pwm_pub = rospy.Publisher("/cmd_out/pwm", Int16MultiArray, queue_size=10)
        self.angle_pub = rospy.Publisher("/cmd_out/current_angle", Float32, queue_size=10)

        # ROS msgs
        self.angle_msg = Float32()
        self.pwm_msg = Int16MultiArray()

        # Angles
        self.target_alpha = 0
        self.target_beta = 0
        self.target_yaw = 0
        self.prev_roll_angle_error = 0
        self.prev_pitch_angle_error = 0
        self.current_time = 10000000
        self.last_recorded_time = 0
        self.motion = []
        #self.pwm_array = [1,2,3,4,5,6,7,8]   # test

        self.max_pwm = 1610
        self.min_pwm = 1390

        self.max_depth = 0.4
        self.min_depth = 0.3
        self.target_depth = 55 #cm

        self.kp = 2
        self.kd = 10
        self._P = 20

        self.pwm = [1500 for i in range(8)]  # thruster 1-8
        self.max_depth_pwm = 60
        self.max_balance_pwm = 50


        self.stable = False

        self.start_imu = False
        self.start_angle = True
        self.saved_angle = False
        self.start_depth = False # set true for testing


        self.depth = 5 #cm for testing


    def targetAngleCallback(self, msg):
        self.target_yaw = msg.data
        self.start_angle = True

    def imuCallback(self, msg):
        self.gx = msg.data[0]
        self.gy = msg.data[1]

        # self.oz = msg.orientation.z
        # self.ow = msg.orientation.w

        self.actual_alpha, self.actual_beta, self.actual_yaw = self.quaternion_to_euler(msg.data[2], msg.data[3], msg.data[4], msg.data[5])  # x,y,z,w

        print(self.actual_alpha, self.actual_beta)

        self.angle_msg.data = self.actual_yaw
        self.angle_pub.publish(self.angle_msg)
        self.start_imu = True

        if self.start_imu == True and self.saved_angle == False: # remove this for normal control
            self.saved_angle =True
            self.target_yaw = self.actual_yaw



    def depthCallback(self, msg):
        self.depth = msg.data
        self.start_depth = True

    def motionCallback(self, msg):
        self.motion = msg.data.split()


    def balance(self):
        error_tolerance = 10  # degree
        roll_angle_error  = self.actual_alpha - self.target_alpha
        pitch_angle_error = self.actual_beta - self.target_beta
        roll_angle_error_difference = roll_angle_error - self.prev_roll_angle_error
        pitch_angle_error_difference = pitch_angle_error - self.prev_pitch_angle_error
        self.prev_roll_angle_error = roll_angle_error
        self.prev_pitch_angle_error = pitch_angle_error
        self.current_time = rospy.get_rostime()
        time_difference = self.current_time.nsecs - self.last_recorded_time
        self.last_recorded_time = self.current_time.nsecs
        time_difference = 1
        self.pwm[4] = 1500
        self.pwm[5] = 1500
        self.pwm[6] = 1500
        self.pwm[7] = 1500
    
        if abs(roll_angle_error) > error_tolerance or abs(pitch_angle_error) > error_tolerance:

            if abs(roll_angle_error) > error_tolerance:
                roll_difference = int(roll_angle_error * self.kp - roll_angle_error_difference * self.kd / time_difference)
                if abs(roll_difference) >= self.max_balance_pwm and roll_difference >= 0:
                    roll_difference = self.max_balance_pwm

                elif abs(roll_difference) >= self.max_balance_pwm and roll_difference < 0:
                    roll_difference = -(self.max_balance_pwm)

                self.pwm[4] -= roll_difference
                self.pwm[5] -= roll_difference
                self.pwm[6] += roll_difference
                self.pwm[7] += roll_difference

            if abs(pitch_angle_error) > error_tolerance:
                pitch_difference = int(pitch_angle_error * self.kp - pitch_angle_error_difference * self.kd/ time_difference)
                if abs(pitch_difference) >= self.max_balance_pwm and pitch_difference >= 0:
                    pitch_difference = self.max_balance_pwm

                elif abs(roll_difference) >= self.max_balance_pwm and pitch_difference < 0:
                    pitch_difference = -(self.max_balance_pwm)

                self.pwm[4] -= pitch_difference
                self.pwm[5] += pitch_difference
                self.pwm[6] -= pitch_difference
                self.pwm[7] += pitch_difference
                

    def depth_control(self):
        self.depth_tolerance = 5 # cm
        #print(self.depth - self.target_depth)
        if abs(self.depth - self.target_depth) > self.depth_tolerance:
            depth_difference += int((self.depth - self.target_depth) * self.kp)

        if depth_difference >= self.max_depth_pwm:
            depth_difference = self.max_depth_pwm

        elif depth_difference <= -(self.max_depth_pwm):
            depth_difference = -(self.max_depth_pwm)

        for i in range(4):
            self.pwm[i+4] += depth_difference


            #for i in range(4):
             #   self.pwm[i+4] += int((self.target_depth - self.max_depth) * self.kp)

        ''' P controller
        if abs(self.actual_beta) > self.target_beta or abs(self.actual_alpha) > self.target_alpha:
            self.stable = False
            if abs(self.actual_beta) > self.target_beta:
                self.pwm[4] -= int((self.actual_beta - self.target_beta) * self.kp)
                self.pwm[5] -= int((self.actual_beta - self.target_beta) * self.kp)
                self.pwm[6] += int((self.actual_beta - self.target_beta) * self.kp)
                self.pwm[7] += int((self.actual_beta - self.target_beta) * self.kp)

            # elif self.gx < -0.1:
            #     self.pwm_5 += 10
            #     self.pwm_6 += 10
            #     self.pwm_7 -= 10
            #     self.pwm_8 -= 10


            if abs(self.actual_alpha) > self.target_alpha:
                self.pwm[4] -= int((self.actual_alpha - self.target_alpha) * self.kp)
                self.pwm[5] += int((self.actual_alpha - self.target_alpha) * self.kp)
                self.pwm[6] -= int((self.actual_alpha - self.target_alpha) * self.kp)
                self.pwm[7] += int((self.actual_alpha - self.target_alpha) * self.kp)

            # elif self.gy < -0.1:
            #     self.pwm_5 += 10
            #     self.pwm_6 -= 10
            #     self.pwm_7 += 10
            #     self.pwm_8 -= 10
        else:
            self.stable = True
            if self.depth < self.min_depth:
                for i in range(4):
                    self.pwm[i+4] += int((self.min_depth - self.depth) * self.kp)

            elif self.depth > self.max_depth:
                for i in range(4):
                    self.pwm[i+4] -= int((self.depth - self.max_depth) * self.kp)
        '''  


    def check_limit(self):

        for i, pwm in enumerate(self.pwm):

            if pwm >= self.max_pwm:
                self.pwm[i] = self.max_pwm

            elif pwm <= self.min_pwm:
                self.pwm[i] = self.min_pwm

        self.pwm_msg.data = self.pwm



    def move(self, motion="FORWARD"):
        if motion == "FORWARD":
            direction_to_compensate, error = self.compute_forward_movement_error()
            self.pwm[0] = self.compute_stabilised_speed(1, error, direction_to_compensate)
            self.pwm[1] = self.compute_stabilised_speed(2, error, direction_to_compensate)
            self.pwm[2] = 1600
            self.pwm[3] = 1600


    def compute_forward_movement_error(self):
        """Compute the forward movement error's magnitude and direction."""
        if (self.actual_yaw >= 0 and self.target_yaw >= 0) or (self.actual_yaw < 0 and self.target_yaw < 0):
            error = math.fabs(self.target_yaw - self.actual_yaw)

            if self.target_yaw > self.actual_yaw:
                direction_to_compensate = "CCW"

            else:
                direction_to_compensate = "CW"

        else:
            if math.fabs(self.actual_yaw) > 90 and math.fabs(self.target_yaw) > 90:
                error = math.fabs(180 - math.fabs(self.target_yaw)) + math.fabs(180 - math.fabs(self.actual_yaw))

                if self.target_yaw < self.actual_yaw:
                    direction_to_compensate = "CCW"

                else:
                    direction_to_compensate = "CW"

            else:
                error = math.fabs(self.target_yaw) + math.fabs(self.actual_yaw)

                if self.target_yaw > self.actual_yaw:
                    direction_to_compensate = "CCW"

                else:
                    direction_to_compensate = "CW"

        return direction_to_compensate, error




    def compute_stabilised_speed(self, thruster_id, error, direction_to_compensate):
        """Compute the stabilised speed from the controller."""
        if (thruster_id == 1 and direction_to_compensate == "CW") or (thruster_id == 2 and direction_to_compensate == "CCW"):
            error = -1*error
        return int(self.pwm[thruster_id - 1] + self._P * error)



    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1) * 180 / math.pi
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2) * 180 / math.pi
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4) * 180 / math.pi
        return roll, pitch, yaw



    def publish_pwm(self):
        # try:
        #     self.update_pwm()
        # except TypeError:
        #     pass
        # else:
        #     try:
        #         self.pwm_pub.publish(pwm_msg)
        self.check_limit()
        self.pwm_pub.publish(self.pwm_msg)




if __name__ == '__main__':
    rospy.init_node("controller", anonymous=True)
    rate = rospy.Rate(10)
    controller = Control()
    while not rospy.is_shutdown():
        if controller.start_imu and controller.start_depth:
            controller.balance()
            controller.depth_control()
            controller.move()
            controller.publish_pwm()
        rate.sleep()
