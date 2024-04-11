# YOLOv8_ROS1
YOLOv8 with ROS1
Referece: https://github.com/mgonzs13/yolov8_ros
=============
Install
=============
Ubuntu 20.04 + ROS noetic
```
$ git clone https://github.com/HJH0303/YOLOv8_ROS1.git
$ pip3 install -r YOLOv8_ROS1/requirements.txt
$ cd ~/YOLOv8_ROS1/catkin_yolov8
$ catkin build
```
Usage
=============
 ```
roslaunch yolov8_ros yolov8_3d.launch 
 ```
Topic
=============
Detection image topic name: /dbg_image
