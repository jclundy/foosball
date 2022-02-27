#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/Joy.h"
#include "foos_control/GetLinearCalibration.h"

int16_t linearSpeedRequested;
int16_t wristSpeedRequested;

#define LINEAR_MAX_SPEED 7500
#define WRIST_MAX_SPEED 5000
#define LH_JOY_VERTICAL_AXIS_INDEX 1
#define RH_JOY_HORIZONTAL_AXIS_INDEX 3

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  linearSpeedRequested = LINEAR_MAX_SPEED*joy->axes[LH_JOY_VERTICAL_AXIS_INDEX];
  wristSpeedRequested = -WRIST_MAX_SPEED*joy->axes[RH_JOY_HORIZONTAL_AXIS_INDEX];
}

typedef struct {
  int16_t maxLinearPos;
  int16_t minLinearPos;
  
} controlSettings;

static controlSettings openLoopControlSettings;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "openLoop");
  ros::NodeHandle n;

  ros::Publisher linearSpeedPub = n.advertise<std_msgs::Int16>("linear_speed", 10);
  ros::Publisher wristSpeedPub = n.advertise<std_msgs::Int16>("wrist_speed", 10);
  ros::Publisher speedModePub = n.advertise<std_msgs::UInt8>("motor_speed_mode_cmd", 10);

  ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
  
  ros::ServiceClient client = n.serviceClient<foos_control::GetLinearCalibration>("linear_calibration_info");

  foos_control::GetLinearCalibration srv;
  srv.request.id = 0;
  
  if(client.call(srv)) {
    ROS_INFO("Called linear_calibration_info service");
    
    openLoopControlSettings.maxLinearPos = srv.response.max;
    openLoopControlSettings.minLinearPos = srv.response.min;
    
    ROS_INFO("Max : %i, Min : %i", openLoopControlSettings.maxLinearPos, openLoopControlSettings.minLinearPos);
    
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
  
    linearSpeedCmd.data = linearSpeedRequested;
    wristSpeedCmd.data = wristSpeedRequested;
    linearSpeedPub.publish(linearSpeedCmd);
    wristSpeedPub.publish(wristSpeedCmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;

}
