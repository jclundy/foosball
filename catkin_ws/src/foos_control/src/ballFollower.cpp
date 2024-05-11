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
  float length;
  float width;
} tableDimensions;

tableDimensions.width = 288; //MM
tableDimensions.length = 398; //MM

typedef struct foosManZone {
  float start;
  float end;
}


class FoosRod {
  public:
    FoosRod(uint8_t numFoosMen, float footWidth, float relativeOffsets[], float motionRange);
    uint8_t getZoneNumber(float ballPositionY);

  private:
    uint8_t numFoosMen;
    float footWidth;
    float relativeOffsets[];
    float motionRange;

    foosManZone zones[];
};

/*
* Todo:
- make numbers all ints, in terms of MM
- if ball in zone, check neighbour for overlap
- fix scale of ball coordinates output by colorTracker
*/

FoosRod::FoosRod(float numFoosMen, float footWidth, float relativeOffsets[], float motionRange) {
  numFoosMen = numFoosMen;
  footWidth = footWidth;
  relativeOffsets = relativeOffsets;
  motionRange = motionRange;
  zones = malloc(numFoosMen * sizeof(foosManZone));
  
  // first zone will go from 0 to 
  zones[0].start = 0;
  zones[0].end = relativeOffsets[0] + motionRange + footWidth / 2;

  for (int i = 0; i < numFoosMen; ) {
     zones[i].start = relativeOffsets[0] - footWidth / 2;
     zones[i].end = relativeOffsets[0] + motionRange + footWidth / 2;
  }
  zones[0].start = 0;
  if(numFoosMen > 1) {
    zones[numFoosMen - 1].end = tableDimensions.width;
  }
}

int8_t FoosRod::getZoneNumber(float ballPositionY, float rodPosition) {
  if(ballPosition < 0) {
    return 0;
  }
  if(ballPosition > tableDimensions.width) {
    return numFoosMen - 1;
  }
  for(int i = 0; i < numFoosMen; i++) {
    if(ballPositionY > zones[i].start && ballPositionY < zones[i].end) {
      // TODO - compare if overlap with zone of neighbour
      return i;
    }
  }

  // technically shouldn't reach here
  return -1;
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