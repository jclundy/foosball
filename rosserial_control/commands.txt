roscore

#for subscribing to msgs from arduino
source ~/catkin_ws/devel/setup.bash
rosrun rosserial_python serial_node.py /dev/ttyACM0

rostopic echo linear_steps

rostopic echo wrist_steps

#for publishing wrist speed command
rostopic pub -1 wrist_speed std_msgs/Int16 -- <wrist_speed>

#for publishing linear speed command
rostopic pub -1 linear_speed std_msgs/Int16 -- <linear_speed>
