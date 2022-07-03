# sauvc_ros


## Hardware

- NVIDIA Jetson Nano
- Arduino Mega / Pixhawk 4
- ZED mini Camera

## Dependencies

- opencv>=4.0
- torch==1.8.0
- torchvision==0.9.0
- python==3.6
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- ZED SDK==3.7

## Run
```shell
# detection
roslaunch zed_wrapper zedm.launch
roslaunch detector detect.launch  # object detection with YOLOv5
rosrun detector client.py  # service to enable or disable the publisher
rosrun detector pub_image.py  # test with video without using zed camera

# control
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600  # initiate communication with arduino
rosrun helper imu_filter.py  # subscribe to IMU topic from zed camera for data filtering
rosrun sauvc_control control.py  # control node that send pwm value to arduino
```  
