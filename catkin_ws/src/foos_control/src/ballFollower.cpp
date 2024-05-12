#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "foos_control/GetLinearCalibration.h"
#include "geometry_msgs/Pose2D.h"

#include "control_definitions.h"
#include "FoosRod.h"
#include <math.h>

#define MAX_SPEED 15000

static struct {
  int maxLinearPos;
  int minLinearPos;
  int currentPos; 
  int setpointSteps;
} controlSettings;


static struct {
  float length;
  float width;
} tableDimensions;

FoosRod *foosRod;

float mapWorldPositionToCarriageSteps(float worldY) {
  float dy = controlSettings.maxLinearPos - controlSettings.minLinearPos;
  float dx = tableDimensions.width;
  float m = dy / dx;
  return worldY * m + controlSettings.minLinearPos;
}

void ballPositionCallback(const geometry_msgs::Pose2D& msg) {
  // for now rescale position down by factor of 2

  int ball_y = msg.y / 2.0;

  int zone = foosRod->getZoneNumber(ball_y);

  if(zone >= 0 && zone < foosRod->getNumFoosMen()) {
    float targetCarriageWorldPosition = ball_y - foosRod->getFoosManOffset(zone);
    
    float targetCarriageSteps = mapWorldPositionToCarriageSteps(targetCarriageWorldPosition);

  	controlSettings.setpointSteps = (int) roundf(targetCarriageSteps);
  }
}

void linearStepsCallBack(const std_msgs::Int16& msg) 
{
   controlSettings.currentPos = msg.data;
}

int main(int argc, char **argv)
{

  ROS_INFO("Debug ball follower start");

  const float foosManOffsets[] = {17, 105, 186};
  const float motionRange = 88; // MM
  const float footWidth = 13; //MM
  tableDimensions.width = 288; //MM
  tableDimensions.length = 398; //MM

  ROS_INFO("before calling new FoosRod");
  foosRod = new FoosRod(3, footWidth, foosManOffsets, motionRange, tableDimensions.width);
  ROS_INFO("after calling new FoosRod");

  ros::init(argc, argv, "ballFollower");
  ros::NodeHandle n;

  ROS_INFO("after calling ros init");
  ros::Publisher linearSpeedPub = n.advertise<std_msgs::Int16>("linear_speed", 10);

  ros::Subscriber stepsSub = n.subscribe("linear_steps", 10, linearStepsCallBack);
  ros::Subscriber ballPositionSub = n.subscribe("foosball/ball_position", 10, ballPositionCallback); 
  ros::ServiceClient client = n.serviceClient<foos_control::GetLinearCalibration>("linear_calibration_info");
  foos_control::GetLinearCalibration srv;

  ROS_INFO("initialize publishers and subscribers");
  srv.request.id = 0;
  
  if(client.call(srv)) {
    ROS_INFO("Called linear_calibration_info service");
    
    controlSettings.maxLinearPos = srv.response.max;
    controlSettings.minLinearPos = srv.response.min;
    
    ROS_INFO("Max : %i, Min : %i", controlSettings.maxLinearPos, controlSettings.minLinearPos);
    
  } else {
    ROS_INFO("Failed to call service linear_calibration_info");
  }

  ROS_INFO("called service");

  ros::Rate loop_rate(30);
  
  ROS_INFO("loop");

  while (ros::ok())
  {

    int stepRange = controlSettings.maxLinearPos - controlSettings.minLinearPos;
    int error = controlSettings.setpointSteps - controlSettings.currentPos;
    const float P_gain = fabs(MAX_SPEED / stepRange);

    float speedRequest = P_gain * error;

    std_msgs::Int16 linearSpeedCmd;
    linearSpeedCmd.data = roundf(speedRequest);
    linearSpeedPub.publish(linearSpeedCmd);

    ROS_INFO("Setpoint = %i, CurretPos = %i, Error=%i, SpeedRequest=%i", controlSettings.setpointSteps, controlSettings.currentPos,  error, linearSpeedCmd.data);


    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;

}