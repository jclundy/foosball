#! /bin/bash  
source /opt/ros/kinetic/setup.bash

source /home/joe/catkin_ws/devel/setup.bash
sudo chmod a+rw /dev/input/js0
roslaunch foos.launch

