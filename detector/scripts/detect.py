#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32, Int16MultiArray
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError
from detector.msg import BoundingBox, BoundingBoxes
import time

# obj detection
import cv2
import torch
import numpy as np
from math import isnan, isinf, atan, pi
print("Using CUDA: ",torch.cuda.is_available())

import time

# Inference
class Detection:
    def __init__(self):
        self.weights_path = rospy.get_param('~weights_path')

        self.model = torch.hub.load("ultralytics/yolov5","custom", path=self.weights_path)
        self.model.classes=[0]
        self.br = CvBridge()
        self.target_angle_msg = Float32()

        self.start_image = False
        self.start_depth = False
        self.start_angle = False

        rospy.Subscriber('/zedm/zed_node/left/image_rect_color', Image, self.image_callback)
        rospy.Subscriber('/zedm/zed_node/depth/depth_registered', Image, self.depth_callback)

        rospy.Subscriber('/cmd_out/current_angle', Float32, self.current_angle_callback)

        self.pub_the_msg = True 

        self.pub_target_angle = rospy.Publisher('/cmd_out/target_angle', Float32, queue_size=10)

        self.start_service = rospy.Service('detection_service', SetBool, self.trigger_publish)

        self.x0 = 0
        self.y0 = 0
        self.x1 = 0
        self.y1 = 0
        self.obj = 30
        self.frame_num = 0
    
    def current_angle_callback(self, msg):
        self.current_angle = msg.data
        self.start_angle = True

    def image_callback(self, data):
        self.bgr_image = self.br.imgmsg_to_cv2(data,"bgr8")
        self.start_image = True


    def depth_callback(self, data):
        self.depth_image = self.br.imgmsg_to_cv2(data, desired_encoding="32FC1")
        self.start_depth = True


    def inference(self):
            
        results = self.model(self.bgr_image, size=640)  # includes NMS
        outputs = results.xyxy[0].cpu()
        if len(outputs) > 0:
            for i,detection in enumerate(outputs):
                self.x0 = int(outputs[i][0]) #xmin
                self.y0 = int(outputs[i][1]) #ymin
                self.x1 = int(outputs[i][2]) #xmax
                self.y1 = int(outputs[i][3]) #ymax
                self.obj = int(outputs[i][5]) #object number
                self.center = ((self.x0+self.x1)//2,(self.y0+self.y1)//2)

                self.bgr_image = cv2.circle(self.bgr_image, [self.center[0], self.center[1]], 2,(0,0,255),2)

                if self.pub_the_msg == True:
                    # self.get_distance()
                    cv2.rectangle(self.bgr_image,(self.x0, self.y0),(self.x1,self.y1),(0,255,0),3)

            cv2.imshow("frame",self.bgr_image)  
        
            try:
                target_angle = atan((672.0/2 - self.center[0]) / self.center[1]) * 180.0/3.14159; # 672x376 ################################################## check
                target_angle += self.current_angle
                self.target_angle_msg.data = target_angle
                self.pub_target_angle.publish(self.target_angle_msg)

            except ZeroDivisionError:
                print(self.center)

            if cv2.waitKey(1) == ord('q'):  # q to quit
                cv2.destroyAllWindows()
                raise StopIteration  

            


    def trigger_publish(self, request):
        if request.data:
            self.pub_the_msg = True
            return SetBoolResponse(True, 'Publishing data...')
        else:
            self.pub_the_msg = False
            return SetBoolResponse(False, 'Keep Quiet...')


    def get_distance(self):
        self.depth_array = np.array(self.depth_image, dtype=np.float32)

        u = (self.x0 + self.x1)//2 
        v = (self.y0 + self.y1)//2

        # print(self.depth_array[v,u])
        if isnan(self.depth_array[v,u]):
            return 0.7

        elif isinf(self.depth_array[v,u]):
            return 2.0
        else:
            return self.depth_array[v,u]

    def reset(self):
        self.center = (0,0)


# Results
if __name__ == "__main__":
    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(15)
    detection = Detection()
    try:
        
        while not rospy.is_shutdown():
            if detection.start_image and detection.start_depth and detection.start_angle:
                
                detection.inference()
                #detection.pub_the_msg = True
            rate.sleep()

    except (KeyboardInterrupt, StopIteration):
        pass
