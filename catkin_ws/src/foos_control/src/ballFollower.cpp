#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "foos_control/GetLinearCalibration.h"
#include "geometry_msgs/Pose2D.h"

#include "control_definitions.h"
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


typedef struct {
  float start;
  float end;
} foosManZone;


class FoosRod {
  public:
    FoosRod(uint8_t numFoosMen, float footWidth, const float relativeOffsets[], float motionRange, float tableWidth);
    int8_t getZoneNumber(float ballPositionY);
    uint8_t getNumFoosMen() {return numFoosMen;};
    float getFoosManOffset(uint8_t foosManNumber);

  private:
    uint8_t numFoosMen;
    float footWidth;
    float motionRange;
    float tableWidth;

    foosManZone *zones;
    float relativeOffsets[];

};

FoosRod::FoosRod(uint8_t numFoosMen, float footWidth, const float relativeOffsets[], float motionRange, float tableWidth) {
  numFoosMen = numFoosMen;
  footWidth = footWidth;
  relativeOffsets = relativeOffsets;
  motionRange = motionRange;
  tableWidth = tableWidth;
  zones = (foosManZone *) malloc(numFoosMen * sizeof(foosManZone));
  
  // first zone will go from 0 to 
  zones[0].start = 0;
  zones[0].end = relativeOffsets[0] + motionRange + footWidth / 2;

  for (int i = 0; i < numFoosMen; ) {
     zones[i].start = relativeOffsets[0] - footWidth / 2;
     zones[i].end = relativeOffsets[0] + motionRange + footWidth / 2;
  }
  zones[0].start = 0;
  if(numFoosMen > 1) {
    zones[numFoosMen - 1].end = tableWidth;
  }
}

int8_t FoosRod::getZoneNumber(float ballPositionY) {
  if(ballPositionY < 0) {
    return 0;
  }
  if(ballPositionY > tableWidth) {
    return numFoosMen - 1;
  }
  for(int i = 0; i < numFoosMen; i++) {
    if(ballPositionY > zones[i].start && ballPositionY < zones[i].end) {
      return i;
    }
  }

  // technically shouldn't reach here
  return -1;
}

float FoosRod::getFoosManOffset(uint8_t foosManNumber) {
    if(foosManNumber >= 0 && foosManNumber < numFoosMen) {
        return relativeOffsets[foosManNumber];
    }
    return 0;
}

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