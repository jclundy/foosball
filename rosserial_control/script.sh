source ~/catkin_ws/source devel/setup.bash

roscore

#for subscribing to msgs from arduino
rosrun rosserial_python serial_node.py /dev/ttyACM0

rostopic echo steps

#for publishing speed command
rostopic pub speed std_msgs/Int16  -- <speed>


