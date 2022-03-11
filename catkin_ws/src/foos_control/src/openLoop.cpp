#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/Joy.h"
#include "foos_control/GetLinearCalibration.h"

#include <math.h>


#define LINEAR_MAX_SPEED 7500
#define WRIST_MAX_SPEED 5000
#define KICK_SPEED 15000
#define LH_JOY_VERTICAL_AXIS_INDEX 1
#define RH_JOY_HORIZONTAL_AXIS_INDEX 3

#define REAR_RIGHT_JOY_INDEX 5
static struct {
	int16_t linearSpeed;
	int16_t wristSpeed;
} setpoints;

static struct {
  int16_t maxLinearPos;
  int16_t minLinearPos;
  int16_t currentPos;
  float alpha = 2000.0;
  
} controlSettings;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  setpoints.linearSpeed = LINEAR_MAX_SPEED*joy->axes[LH_JOY_VERTICAL_AXIS_INDEX];
  
  int16_t wristControlSpeed = -WRIST_MAX_SPEED*joy->axes[RH_JOY_HORIZONTAL_AXIS_INDEX];
  
  float rearRightJoystick = joy->axes[REAR_RIGHT_JOY_INDEX]; //nominal 1.0, when triggered -1.0
  int16_t wristKickSpeed = -KICK_SPEED *(rearRightJoystick - 1)/2;
  setpoints.wristSpeed = wristKickSpeed + wristControlSpeed;
}

void linearStepsCallBack(const std_msgs::Int16& msg) 
{
   controlSettings.currentPos = msg.data;
}

void setAlphaCallBack(const std_msgs::Int16& msg) {
	controlSettings.alpha = (float) msg.data;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "openLoop");
  ros::NodeHandle n;

  ros::Publisher linearSpeedPub = n.advertise<std_msgs::Int16>("linear_speed", 10);
  ros::Publisher wristSpeedPub = n.advertise<std_msgs::Int16>("wrist_speed", 10);
  ros::Publisher speedModePub = n.advertise<std_msgs::UInt8>("motor_speed_mode_cmd", 10);

  ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
  ros::Subscriber stepsSub = n.subscribe("linear_steps", 10, linearStepsCallBack);
  ros::Subscriber alphaSub = n.subscribe("set_alpha", 10, setAlphaCallBack);
  
  ros::ServiceClient client = n.serviceClient<foos_control::GetLinearCalibration>("linear_calibration_info");

  foos_control::GetLinearCalibration srv;
  srv.request.id = 0;
  
  if(client.call(srv)) {
    ROS_INFO("Called linear_calibration_info service");
    
    controlSettings.maxLinearPos = srv.response.max;
    controlSettings.minLinearPos = srv.response.min;
    
    ROS_INFO("Max : %i, Min : %i", controlSettings.maxLinearPos, controlSettings.minLinearPos);
    
  } else {
    ROS_INFO("Failed to call service linear_calibration_info");
  }

  ros::Rate loop_rate(10);
 
  std_msgs::UInt8 speedModeCmd;
  speedModeCmd.data = 1; //LINEAR_MOTOR
  speedModePub.publish(speedModeCmd);
  
  std_msgs::Int16 linearSpeedCmd;
  std_msgs::Int16 wristSpeedCmd;
  
  
  while (ros::ok())
  {
  
    int16_t speedCommand = setpoints.linearSpeed;
    
    int16_t distance = (speedCommand > 0)? (controlSettings.maxLinearPos - controlSettings.currentPos) : (controlSettings.currentPos - controlSettings.minLinearPos);  
    float scale = tanh(fabs(distance/(controlSettings.alpha)));
    
    ROS_INFO("Scale: %f, Distance %i", scale, distance);
     
    linearSpeedCmd.data = speedCommand * scale;
    wristSpeedCmd.data = setpoints.wristSpeed;
    linearSpeedPub.publish(linearSpeedCmd);
    wristSpeedPub.publish(wristSpeedCmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;

}
