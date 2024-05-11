#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/Joy.h"
#include "foos_control/GetLinearCalibration.h"

#include "joystick_definitions.h"
#include "control_definitions.h"

#include <math.h>

static struct {
  int16_t maxLinearPos;
  int16_t minLinearPos;
  int16_t currentPos; 
} controlSettings;

ros::Publisher positionPub;

void linearStepsCallBack(const std_msgs::Int16& msg) 
{
   controlSettings.currentPos = msg.data;
}

static struct {
  int16_t length;
  int16_t width;
} tableDimensions;

tableDimensions.length = 288; //MM
tableDimensions.width = 398; //MM

typedef foosManZone {
  int16_t start;
  int16_t end;
}

void ballPositionCallback(const geometry_msgs::Pose2D& msg) {
  // for now rescale position down by factor of 2

  int16_t ball_y = msg.data.y / 2.0;

}


void linearStepsCallBack(const std_msgs::Int16& msg) 
{
   controlSettings.currentPos = msg.data;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ballFollower");
  ros::NodeHandle n;

  positionPub = n.advertise<std_msgs::Int16>("linear_position_cmd", 10);
  ros::Subscriber stepsSub = n.subscribe("linear_steps", 10, linearStepsCallBack);
  ros::Subscriber ballPositionSub = n.subscribe("foosball/ball_position", 10, ballPositionCallback);

  ("foosball/ball_position", 10);
  
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
  
  while (ros::ok())
  {
  
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;

}