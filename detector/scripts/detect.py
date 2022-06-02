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
from math import isnan, isinf
print("Using CUDA: ",torch.cuda.is_available())

import time

# Inference
class Detection:
    def __init__(self):
        self.weights_path = rospy.get_param('~weights_path')

        self.model = torch.hub.load("ultralytics/yolov5","custom", path=self.weights_path)
        self.model.classes=[0]
        self.br = CvBridge()

        self.pub_the_msg = True 
        self.pub_box = rospy.Publisher('/boundingbox', Int16MultiArray, queue_size=10)
        self.start_image = False
        self.start_depth = False
        rospy.Subscriber('/zedm/zed_node/left/image_rect_color', Image, self.image_callback)
        rospy.Subscriber('/zedm/zed_node/depth/depth_registered', Image, self.depth_callback)

        self.start_service = rospy.Service('detection_service', SetBool, self.trigger_publish)

        self.x0 = 0
        self.y0 = 0
        self.x1 = 0
        self.y1 = 0
        self.obj = 30
        self.center = ((self.x0+self.x1)//2,(self.y0+self.y1)//2)
        self.frame_num=0
    

    def image_callback(self, data):
        self.bgr_image = self.br.imgmsg_to_cv2(data,"bgr8")
        self.start_image = True


    def depth_callback(self, data):
        self.depth_image = self.br.imgmsg_to_cv2(data, desired_encoding="32FC1")
        self.start_depth = True


    def inference(self):
        bbox = Int16MultiArray()
        bbox.data = []
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.start_image and self.start_depth:
                start_time = time.time()
                               
                # Convert ROS Image message to OpenCV image
          
                results = self.model(self.bgr_image, size=640)  # includes NMS

                outputs = results.xyxy[0].cpu()
                if len(outputs) > 0:
                    for i,detection in enumerate(outputs):
                        self.x0 = int(outputs[i][0]) #xmin
                        self.y0 = int(outputs[i][1]) #ymin
                        self.x1 = int(outputs[i][2]) #xmax
                        self.y1 = int(outputs[i][3]) #ymax
                        #conf = detection[4].tolist()
                        self.obj = int(outputs[i][5]) #object number
                        #allInfo = outputs[i].tolist()
                        bbox.data.insert(i,[self.x0, self.y0, self.x1, self.y1, self.obj])
                        #allInfo.insert(4,0)

                        if self.pub_the_msg == True:
                            self.get_distance()
                            cv2.rectangle(self.bgr_image,(self.x0, self.y0),(self.x1,self.y1),(0,255,0),3)
                            #self.frame_num+=1

                            #cv2.putText(frame, "Frames: {}".format(self.frame_num), (5, 35), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (0, 255, 0), 2)
                            #cv2.putText(frame, "FPS: {:2f}".format(1.0 / (time.time() - start_time)), (5, 80), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (0, 255, 0), 2)

                    cv2.imshow("frame",self.bgr_image)  
                    rate.sleep()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
                    if cv2.waitKey(1) == ord('q'):  # q to quit
                        cv2.destroyAllWindows()
                        raise StopIteration  
                    #rospy.loginfo(1.0 / (time.time() - start_time)," fps")
                    if bbox:
                        self.pub_box.publish(bbox)     # TO BE FIX

            rate.sleep()
            


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

        print(self.depth_array[v,u])
        if isnan(self.depth_array[v,u]):
            return 0.7

        elif isinf(self.depth_array[v,u]):
            return 2.0
        else:
            return self.depth_array[v,u]



# Results
if __name__ == "__main__":
    try:
        rospy.init_node('detect', anonymous=True)
        with torch.no_grad():
            detection = Detection()
            detection.inference()
            #detection.pub_the_msg = True

    except (KeyboardInterrupt, StopIteration):
        pass
