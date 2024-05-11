#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "foos_control/GetLinearCalibration.h"
#include "geometry_msgs/Pose2D.h"

#include "control_definitions.h"
#include "FoosRod.h"
#include <math.h>

static struct {
  int16_t maxLinearPos;
  int16_t minLinearPos;
  int16_t currentPos; 
} controlSettings;


static struct {
  float length;
  float width;
} tableDimensions;

ros::Publisher positionPub;

FoosRod *foosRod;

void ballPositionCallback(const geometry_msgs::Pose2D& msg) {
  // for now rescale position down by factor of 2

  int16_t ball_y = msg.y / 2.0;

  int zone = foosRod->getZoneNumber(ball_y);
  if(zone >= 0 && zone < foosRod->getNumFoosMen()) {
    float targetCarriagePosition = ball_y - foosRod->getFoosManOffset(zone);
    
    std_msgs::Int16 positionCmd;
  	positionCmd.data = (int16_t) roundf(targetCarriagePosition);
  	positionPub.publish(positionCmd);
  }

}

void linearStepsCallBack(const std_msgs::Int16& msg) 
{
   controlSettings.currentPos = msg.data;
}

int main(int argc, char **argv)
{

  const float foosManOffsets[] = {17, 105, 186};
  const float motionRange = 88; // MM
  const float footWidth = 13; //MM
  tableDimensions.width = 288; //MM
  tableDimensions.length = 398; //MM

  foosRod = new FoosRod(3, footWidth, foosManOffsets, motionRange, tableDimensions.width);

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