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

## Run detection
```shell
# main
roslaunch detector detect.launch 

# other nodes
rosrun detector client.py  # service to enable or disable the publisher
rosrun detector pub_image.py  # test with video without using zed camera
```  
