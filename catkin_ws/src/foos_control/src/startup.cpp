#include "ros/ros.h"
#include "std_msgs/String.h"

typedef struct {
  bool waitingForUserSwitchPress;
  bool forwardDepressed;
  bool rearDepressed;
} calibration_progress_t;

calibration_progress_t calibrationProgress;

const int16_t DEFAULT_SPEED = 5000;
int16_t speedCmd = 0;

typedef enum {
  REVERSE = -1,
  NEUTRAL = 0,
  FORWARD = 1,
} direction_t;

void limitReachedCallBack(const std_msgs::Int8::ConstPtr& msg)
{

  ROS_INFO("limit hit %f", msg.data);
  if(calibrationProgress.waitingForUserSwitchPress) {
  	if(msg.data == FORWARD) {
  		// user pressed forward switch, drive forward
  		speedCmd = DEFAULT_SPEED;
  	} else {
  		// user pressed rear switch, driver reverse
  		speedCmd = -1 * DEFAULT_SPEED;
  	}
  	calibrationProgress.waitingForUserSwitchPress = false;
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
  }  

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "startup");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("limit_reached", 10, limitReachedCallBack);
  ros::Publisher speedPub = n.advertise<std_msgs::Int16>("linear_speed", 10);

  calibrationProgress.waitingForUserSwitchPress = true;
  calibrationProgress.forwardDepressed = false;
  calibrationProgress.rearDepressed = false;

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
