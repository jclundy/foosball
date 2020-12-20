source ~/catkin_ws/devel/setup.bash

roscore

#for subscribing to msgs from arduino
rosrun rosserial_python serial_node.py /dev/ttyACM0

rostopic echo steps

#for publishing speed command
rostopic pub -1 speed std_msgs/Int16 -- <speed>


