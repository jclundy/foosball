#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"

typedef struct {
  bool waitingForUserSwitchPress;
  bool forwardDepressed;
  bool rearDepressed;
} calibration_progress_t;

calibration_progress_t calibrationProgress;

const int16_t DEFAULT_SPEED = 200;
int16_t speedCmd = 0;

typedef enum {
  REVERSE = -1,
  NEUTRAL = 0,
  FORWARD = 1,
} direction_t;

void limitReachedCallBack(const std_msgs::Int8& msg)
{
  if(calibrationProgress.waitingForUserSwitchPress) {
  	if(msg.data == FORWARD) {
  		// user pressed forward switch, drive forward
  		speedCmd = DEFAULT_SPEED;
  	} else if(msg.data == REVERSE) {
  		// user pressed rear switch, driver reverse
  		speedCmd = -1 * DEFAULT_SPEED;
  	}
  	calibrationProgress.waitingForUserSwitchPress = false;
  	ROS_INFO("calibration started");
  } else if (msg.data == FORWARD && !calibrationProgress.forwardDepressed) {
  	// was driving forward and hit limit
  	calibrationProgress.forwardDepressed = true;
  	// drive reverse
  	speedCmd = -1 * DEFAULT_SPEED;
  } else if (msg.data == REVERSE && !calibrationProgress.rearDepressed) {
  	// was driving reverse and hit limit
	calibrationProgress.rearDepressed = true;
	// drive forward
	speedCmd = DEFAULT_SPEED;
  }
    
  if(calibrationProgress.rearDepressed && calibrationProgress.forwardDepressed) {
  	speedCmd = 0;
  	ROS_INFO("done calibrating");
  }
  
  ROS_INFO("limit hit %i", msg.data);    
  ROS_INFO("speed cmd %i", speedCmd);  
  ROS_INFO("forward pressed %i", calibrationProgress.forwardDepressed);
  ROS_INFO("rear pressed %i", calibrationProgress.rearDepressed);
}


int main(int argc, char **argv)
{

  calibrationProgress.waitingForUserSwitchPress = true;
  calibrationProgress.forwardDepressed = false;
  calibrationProgress.rearDepressed = false;

  ros::init(argc, argv, "startup");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("limit_reached", 10, limitReachedCallBack);
  ros::Publisher speedPub = n.advertise<std_msgs::Int16>("linear_speed", 10);

  ros::Rate loop_rate(10);

  ROS_INFO("waiting for user input");
  
  while (ros::ok())
  {
    std_msgs::Int16 linearSpeedCmd;
    linearSpeedCmd.data = speedCmd;
    speedPub.publish(linearSpeedCmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;

}
