#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"

typedef struct {
  bool waitingForUserSwitchPress;
  bool forwardDepressed;
  bool rearDepressed;
} calibration_progress_t;

calibration_progress_t calibrationProgress;

const int16_t DEFAULT_SPEED = 200;

int16_t steps = 0;
int16_t maxSteps = 0;
int16_t minSteps = 0;

typedef enum {
  REVERSE = -1,
  NEUTRAL = 0,
  FORWARD = 1,
} direction_t;


typedef enum {
  WRIST_MOTOR,
  LINEAR_MOTOR,
} motor_type_t;

ros::Publisher speedPub;
ros::Publisher positionPub;

void publishSpeedCmd(int16_t speed) {
	std_msgs::Int16 linearSpeedCmd;
	linearSpeedCmd.data = speed;
  	speedPub.publish(linearSpeedCmd);
}

void limitReachedCallBack(const std_msgs::Int8& msg)
{
  int16_t speedCmd = 0;
  if(calibrationProgress.waitingForUserSwitchPress) {
  	if(msg.data == FORWARD) {
  		// user pressed forward switch, drive forward
  		speedCmd = DEFAULT_SPEED;
  	} else if(msg.data == REVERSE) {
  		// user pressed rear switch, driver reverse
  		speedCmd = -1 * DEFAULT_SPEED;
  	}
  	calibrationProgress.waitingForUserSwitchPress = false;
 	
  	publishSpeedCmd(speedCmd);

  	ROS_INFO("calibration started");
  } else if (msg.data == FORWARD && !calibrationProgress.forwardDepressed) {
  	// was driving forward and hit limit
  	calibrationProgress.forwardDepressed = true;
  	// drive reverse
  	speedCmd = -1 * DEFAULT_SPEED;
  	
  	maxSteps = steps;
  	
  	publishSpeedCmd(speedCmd);
  } else if (msg.data == REVERSE && !calibrationProgress.rearDepressed) {
  	// was driving reverse and hit limit
	calibrationProgress.rearDepressed = true;
	// drive forward
	speedCmd = DEFAULT_SPEED;
	minSteps = steps;
	
	publishSpeedCmd(speedCmd);
  }
  

    
  if(calibrationProgress.rearDepressed && calibrationProgress.forwardDepressed) {
  	ROS_INFO("done calibrating");
  	// send command to reset position
  	int midPoint = (maxSteps + minSteps)/2;
  	
  	std_msgs::Int16 positionCmd;
  	positionCmd.data = midPoint;
  	positionPub.publish(positionCmd);
  	
  	speedCmd = copysign(DEFAULT_SPEED, (midPoint - steps));
    	
  	publishSpeedCmd(speedCmd);
  }

  ROS_INFO("steps %i", steps);  
  ROS_INFO("limit hit %i", msg.data);    
  ROS_INFO("speed cmd %i", speedCmd);  
  ROS_INFO("forward pressed %i", calibrationProgress.forwardDepressed);
  ROS_INFO("rear pressed %i", calibrationProgress.rearDepressed);
}

void linearStepsCallBack(const std_msgs::Int16& msg) 
{
   steps = msg.data;
}

int main(int argc, char **argv)
{

  calibrationProgress.waitingForUserSwitchPress = true;
  calibrationProgress.forwardDepressed = false;
  calibrationProgress.rearDepressed = false;

  ros::init(argc, argv, "startup");
  ros::NodeHandle n;

  ros::Subscriber limitSub = n.subscribe("limit_reached", 10, limitReachedCallBack);
  ros::Subscriber stepsSub = n.subscribe("linear_steps", 10, linearStepsCallBack);
  speedPub = n.advertise<std_msgs::Int16>("linear_speed", 10);
  positionPub = n.advertise<std_msgs::Int16>("linear_position_cmd", 10);

  ros::Publisher speedModePub = n.advertise<std_msgs::UInt8>("motor_speed_mode_cmd", 10);

  ros::Rate loop_rate(10);

  ROS_INFO("waiting for user input");
  
  std_msgs::UInt8 speedModeCmd;
  speedModeCmd.data = LINEAR_MOTOR;
  speedModePub.publish(speedModeCmd);
  
  
  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;

}
