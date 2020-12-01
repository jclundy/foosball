roscore

#for subscribing to msgs from arduino
rosrun rosserial_python serial_node.py /dev/ttyACM0

rostopic echo chatter steps

#for publishing speed command
rostopic pub speed std_msgs/UInt16  <angle>


