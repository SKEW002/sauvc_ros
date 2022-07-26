#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu


import math
import time

class Control:
    def __init__(self):
        rospy.Subscriber('/cmd_out/imu_data',Float32MultiArray , self.imuCallback)
        rospy.Subscriber('/depth',Int16 , self.depthCallback)

        self.pwm_pub = rospy.Publisher("/cmd_out/pwm", Int16MultiArray, queue_size=10)
        self.angle_pub = rospy.Publisher("/cmd_out/current_angle", Int16MultiArray, queue_size=10)

        self.pwm_msg = Int16MultiArray()

        #self.pwm_array = [1,2,3,4,5,6,7,8]   # test

        self.max_pwm = 1700
        self.min_pwm = 1300

        self.max_depth = 0.3
        self.min_depth = 0.2

        self.kp = 25
        self._P = 25

        self.pwm = [1500 for i in range(8)]  # thruster 1-8

        self.stable = False

        self.start_imu = False


        self.start_depth = True # set true for testing
        self.depth = 0.25 # for testing


    def imuCallback(self, msg):
        self.gx = msg.data[0]
        self.gy = msg.data[1]

        # self.oz = msg.orientation.z
        # self.ow = msg.orientation.w

        self.alpha, self.beta, self.gamma = self.quaternion_to_euler(msg.data[2], msg.data[3], msg.data[4], msg.data[5])  # x,y,z,w
        self.start_imu = True


    def depthCallback(self, msg):
        self.depth = float(msg.data / 100)
        self.start_depth = True


    def balance(self):
        tol = 0.4
        if abs(self.gx) > tol or abs(self.gy) > tol:
            self.stable = False
            if abs(self.gx) > tol:
                self.pwm[4] -= int(self.gx * self.kp)
                self.pwm[5] -= int(self.gx * self.kp)
                self.pwm[6] += int(self.gx * self.kp)
                self.pwm[7] += int(self.gx * self.kp)

            # elif self.gx < -0.1:
            #     self.pwm_5 += 10
            #     self.pwm_6 += 10
            #     self.pwm_7 -= 10
            #     self.pwm_8 -= 10


            if abs(self.gy) > tol:
                self.pwm[4] -= int(self.gy * self.kp)
                self.pwm[5] += int(self.gy * self.kp)
                self.pwm[6] -= int(self.gy * self.kp)
                self.pwm[7] += int(self.gy * self.kp)

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



    def check_limit(self):

        for i, pwm in enumerate(self.pwm):

            if pwm >= self.max_pwm:
                self.pwm[i] = self.max_pwm

            elif pwm <= self.min_pwm:
                self.pwm[i] = self.min_pwm

        self.pwm_msg.data = self.pwm



    def move(self, motion, angle):
        if self.motion == "forward":
            direction_to_compensate, error = self._compute_forward_movement_error()
            self.pwm[0] = self._compute_stabilised_speed("1", error, direction_to_compensate)
            self.pwm[1] = self._compute_stabilised_speed("2", error, direction_to_compensate)


    def _compute_forward_movement_error(self):
        """Compute the forward movement error's magnitude and direction."""
        if (self._actual_euler["gamma"] >= 0 and self._target_euler["gamma"] >= 0) or (self._actual_euler["gamma"] < 0 and self._target_euler["gamma"] < 0):
            error = math.fabs(self._target_euler["gamma"] - self._actual_euler["gamma"])

            if self._target_euler["gamma"] > self._actual_euler["gamma"]:
                direction_to_compensate = "CCW"

            else:
                direction_to_compensate = "CW"

        else:
            if math.fabs(self._actual_euler["gamma"]) > 90 and math.fabs(self._target_euler["gamma"]) > 90:
                error = math.fabs(180 - math.fabs(self._target_euler["gamma"])) + math.fabs(180 - math.fabs(self._actual_euler["gamma"]))

                if self._target_euler["gamma"] < self._actual_euler["gamma"]:
                    direction_to_compensate = "CCW"

                else:
                    direction_to_compensate = "CW"

            else:
                error = math.fabs(self._target_euler["gamma"]) + math.fabs(self._actual_euler["gamma"])

                if self._target_euler["gamma"] > self._actual_euler["gamma"]:
                    direction_to_compensate = "CCW"

                else:
                    direction_to_compensate = "CW"

        return direction_to_compensate, error




    def _compute_stabilised_speed(self, thruster_id, error, direction_to_compensate):
        """Compute the stabilised speed from the controller."""
        if (thruster_id == "1" and direction_to_compensate == "CW") or (thruster_id == "2" and direction_to_compensate == "CCW"):
            error = -1*error
        return int(self._thrusters_actual_speed[thruster_id] + self._P * error)


    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        alpha = math.atan2(t0, t1) * 180 / math.pi
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        beta = math.asin(t2) * 180 / math.pi
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        gamma = math.atan2(t3, t4) * 180 / math.pi
        return alpha, beta, gamma


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


    def filter(self, imu_data):
        '''
        Exponential Moving Average (EMA)
        '''

        ema = 0
        alpha = 0.125
        denominator = 0

        for i in range(len(imu_data)):
            denominator += (1-alpha)**i

        
        for i in range(len(imu_data)):
            ema += ((imu_data[len(imu_data) - i - 1]) *(1-alpha)**i) / denominator

        return ema


if __name__ == '__main__':
    rospy.init_node("controller", anonymous=True)
    rate = rospy.Rate(10)
    controller = Control()
    while not rospy.is_shutdown():
        if controller.start_imu and controller.start_depth:
            controller.balance()
            controller.publish_pwm()
        rate.sleep()