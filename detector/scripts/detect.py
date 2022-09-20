#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError

# obj detection
import cv2
import torch
import numpy as np
from math import isnan, isinf, atan, pi
from detection_msgs.msg import BoundingBox, BoundingBoxes
print("Using CUDA: ",torch.cuda.is_available())

# Inference
class Detection:
    def __init__(self):

        '''Perception init'''
        self.weights_path = rospy.get_param('~weights_path')
        #self.model = torch.hub.load("ultralytics/yolov5","custom", path=self.weights_path)
        #self.model.cuda()
        # self.model.classes=[0]
        #self.model.model.half()
        self.br = CvBridge()

        '''Start init'''
        self.start_image = False
        self.start_depth = True
        self.start_angle = True

        '''Subscriber'''
        rospy.Subscriber('/zedm/zed_node/left/image_rect_color', Image, self.image_callback)
        #rospy.Subscriber('/zedm/zed_node/depth/depth_registered', Image, self.depth_callback)
        rospy.Subscriber('/cmd_out/current_angle', Float32, self.current_angle_callback)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.boundingbox_callback)

        '''Publisher'''
        self.pub_target_angle = rospy.Publisher('/cmd_out/target_angle', Float32, queue_size=10)
        self.pub_motion = rospy.Publisher('/cmd_out/motion', String, queue_size=10)
        self.motion_msg = String()
        self.target_angle_msg = Float32()

        '''Start/stop service'''
        self.start_service = rospy.Service('detection_service', SetBool, self.trigger_publish)
        self.pub_the_msg = True


        '''Object detection variable'''
        self.x0 = 0
        self.y0 = 0
        self.x1 = 0
        self.y1 = 0
        self.obj = 'qual_gate'
        self.distance = 10


        '''Angle'''
        self.current_angle = 0.0
        self.loaded_current_angle = 0.0


        ''' Color detection'''
        self.color_center = [0,0]
        self.boundaries = { # hsv color boundaries
            'red' : np.array([[0,120,5], [5,255,255], [161, 125, 5], [179, 255, 255]]),  
            'blue' : np.array([[98, 109, 2], [116, 255, 255]]),  
        }
        self.bgr_colors = {'red':(0,0,255), 'blue':(255,0,0), 'orange':(0,140,255)}
        self.found_red = False


        self.time_init = rospy.get_time()

        self.mission = ["main_gate", "yellow_flare", "surface"]

    def boundingbox_callback(self, msg):
        if msg.bounding_boxes:
            boundingbox = msg.bounding_boxes[0]
            self.x0 = boundingbox.xmin
            self.y0 = boundingbox.ymin
            self.x1 = boundingbox.xmax
            self.y1 = boundingbox.ymax
            self.obj = boundingbox.Class

    def current_angle_callback(self, msg):
        self.current_angle = msg.data
        self.start_angle = True

    def image_callback(self, data):
        self.bgr_image = self.br.imgmsg_to_cv2(data,"bgr8")
        self.image_height, self.image_width, self.image_chanel = self.bgr_image.shape # (480,640,3)
        self.start_image = True


    def depth_callback(self, data):
        self.depth_image = self.br.imgmsg_to_cv2(data, desired_encoding="32FC1")
        self.depth_array = np.array(self.depth_image, dtype=np.float32)

        u = self.image_width//2
        v = self.image_height//2

        if isnan(self.depth_array[v,u]):
            self.distance = 0.7

        elif isinf(self.depth_array[v,u]):
            self.distance = 2.0

        else:
            self.distance = self.depth_array[v,u]

        self.start_depth = True


    def inference(self):
        self.motion_msg.data = " FORWARD"
        # results = self.model(self.bgr_image, size=320)  # includes NMS
        # outputs = results.xyxy[0].cpu()

        self.center = ((self.x0+self.x1)//2,(self.y0+self.y1)//2)

        print(self.x0,self.y0,self.x1,self.y1, self.center)

        if self.obj == 'qual_gate': # qualification gate
            self.motion_msg.data = " FORWARD"

        #self.bgr_image = cv2.circle(self.bgr_image, [self.center[0], self.center[1]], 2,(0,0,255),2)

        if self.pub_the_msg == True:
            cv2.rectangle(self.bgr_image,(self.x0, self.y0),(self.x1,self.y1),(0,255,0),3)  
    
        try:
            target_angle = atan((self.image_width/2 - self.center[0]) / self.center[1]) * 180.0/pi # 672x376 ################################################## check
            
            if target_angle < 10:
                target_angle = -10
            elif target_angle > 10:
                target_angle = 10
            else:
                target_angle = 0

            target_angle += self.current_angle
            self.target_angle_msg.data = target_angle

        except ZeroDivisionError:
            print(self.center)


        if self.distance < 1.0:
            self.motion_msg.data = " STOP"

        #self.color_detect()

        ################################################ TODO ############################
        if self.found_red:
            #print(self.color_center)
            self.load_current_angle()
            if self.color_center[0] >= self.image_width/2:
                self.target_angle_msg.data = self.loaded_current_angle + 10 ###this is wrong

            else:
                self.target_angle_msg.data = self.loaded_current_angle - 10
        else:
            try:
                self.target_angle_msg.data = target_angle
            except UnboundLocalError:
                pass

        
        self.pub_target_angle.publish(self.target_angle_msg)
        self.pub_motion.publish(self.motion_msg)

        #cv2.imshow("frame",self.bgr_image)

        if cv2.waitKey(1) == ord('q'):  # q to quit
            cv2.destroyAllWindows()
            raise StopIteration  

            

    def load_current_angle(self):
        self.loaded_current_angle = self.current_angle


    def trigger_publish(self, request):
        if request.data:
            self.pub_the_msg = True
            return SetBoolResponse(True, "Publishing data...")
        else:
            self.pub_the_msg = False
            return SetBoolResponse(False, "Keep Quiet...")


    def get_distance(self):
        pass
        # u = (self.x0 + self.x1)//2 
        # v = (self.y0 + self.y1)//2

        # print(self.depth_array[v,u])

    def time_check(self):
        duration = rospy.get_time() - self.time_init
        if duration > 80:
            self.mission_num += 1

        elif duration > 140:
            self.mission_num += 1

    def color_detect(self):
        largest_radius = 0

        blur = cv2.GaussianBlur(self.bgr_image, (11, 11), 0) # reduce noises, smoothing image
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) # convert BGR image to HSV image
        center = [0,0]
        for color, code in self.boundaries.items():
            if color == 'red':
                low1, high1, low2, high2 = code
                mask1 = cv2.inRange(hsv, low1, high1)
                mask2 = cv2.inRange(hsv, low2, high2)
                mask = mask1 + mask2
            else:
                low, high = code
                mask = cv2.inRange(hsv, low, high)

            kernel = np.ones((10,10),np.uint8) # Unsigned(no negative value) int 8bits(0-255)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # remove false positives. remove pixels(noises) from image (outside detected shape)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # remove false negatives. inside detected shape

            cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

            if color == 'red':
                for index, con in enumerate(cnts):
                    (x,y),radius = cv2.minEnclosingCircle(con)
                    if radius < 2: # ignore noises 
                        pass
                    else:
                        if radius > largest_radius:
                            self.found_red = True
                            largest_radius = radius
                            center = [int(x),int(y)]
                            #cv2.drawContours(self.bgr_image, cnts, -1, self.bgr_colors[color], 5)
                            cv2.putText(self.bgr_image,color, center, cv2.FONT_HERSHEY_SIMPLEX, 0.6,self.bgr_colors[color],2)

            if largest_radius == 0:
                self.found_red = False

        self.color_center = center


    def reset(self):
        self.center = (0,0)


# Results
if __name__ == "__main__":
    rospy.init_node('computer_vision', anonymous=True)
    rate = rospy.Rate(10)
    detection = Detection()
    try:
        while not rospy.is_shutdown():
            if detection.start_image and detection.start_depth and detection.start_angle:
                detection.inference()
                #detection.pub_the_msg = True
            rate.sleep()

    except (KeyboardInterrupt, StopIteration):
        pass
