#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/Joy.h"

int16_t linearSpeedRequested;
int16_t wristSpeedRequested;

#define LINEAR_MAX_SPEED 1000
#define RH_JOY_VERTICAL_AXIS_INDEX 1
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  linearSpeedRequested = LINEAR_MAX_SPEED*joy->axes[RH_JOY_VERTICAL_AXIS_INDEX];
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "openLoop");
  ros::NodeHandle n;

  ros::Publisher linearSpeedPub = n.advertise<std_msgs::Int16>("linear_speed", 10);
  ros::Publisher wristSpeedPub = n.advertise<std_msgs::Int16>("wrist_speed", 10);
  ros::Publisher speedModePub = n.advertise<std_msgs::UInt8>("motor_speed_mode_cmd", 10);

	ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

  ros::Rate loop_rate(10);
 
  std_msgs::UInt8 speedModeCmd;
  speedModeCmd.data = 1; //LINEAR_MOTOR
  speedModePub.publish(speedModeCmd);
  
  std_msgs::Int16 linearSpeedCmd;
  std_msgs::Int16 wristSpeedCmd;
  
  while (ros::ok())
  {
  
    linearSpeedCmd.data = linearSpeedRequested;
    wristSpeedCmd.data = 0;
    linearSpeedPub.publish(linearSpeedCmd);
    wristSpeedPub.publish(wristSpeedCmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;

}
