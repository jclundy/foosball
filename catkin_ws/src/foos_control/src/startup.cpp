#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "foos_control/RailCalibration.h"

#include "control_definitions.h"

static struct {
  bool waitingForUserSwitchPress;
  bool forwardDepressed;
  bool rearDepressed;
}  calibrationProgress;

int16_t steps = 0;
int16_t maxSteps = 0;
int16_t minSteps = 0;
int16_t midPoint = 0;

const int16_t DEFAULT_SPEED = 500;

ros::Publisher speedModePub;
ros::Publisher positionPub;
ros::Publisher calibrationPub;
ros::Publisher speedPub;
int16_t speedCmd = 0;


void publishSpeedMode() {
  std_msgs::UInt8 speedModeCmd;
  speedModeCmd.data = LINEAR_MOTOR;
  speedModePub.publish(speedModeCmd);
}

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

	publishSpeedMode();

  	ROS_INFO("calibration started");
  } else if (msg.data == FORWARD && !calibrationProgress.forwardDepressed) {
  	// was driving forward and hit limit
  	calibrationProgress.forwardDepressed = true;
  	// drive reverse
  	speedCmd = -1 * DEFAULT_SPEED;
  	
  	maxSteps = steps;
  	
	publishSpeedMode();  	
  	
  } else if (msg.data == REVERSE && !calibrationProgress.rearDepressed) {
  	// was driving reverse and hit limit
		calibrationProgress.rearDepressed = true;
		// drive forward
		speedCmd = DEFAULT_SPEED;
		minSteps = steps;

		publishSpeedMode();
  }
  

    
  if(calibrationProgress.rearDepressed && calibrationProgress.forwardDepressed) {
  	ROS_INFO("done calibrating");
  	// send command to reset position
  	midPoint = (maxSteps + minSteps)/2;
  	
  	std_msgs::Int16 positionCmd;
  	positionCmd.data = midPoint;
  	positionPub.publish(positionCmd);
  	
  	speedCmd = copysign(DEFAULT_SPEED, (midPoint - steps));
  	
  	ROS_INFO("setpoint %i", midPoint);
  	
	foos_control::RailCalibration calibrationData;
  	calibrationData.min = minSteps;
  	calibrationData.max = maxSteps;
  	calibrationPub.publish(calibrationData);
  }

  ROS_INFO("================");
  ROS_INFO("steps %i", steps);  
  ROS_INFO("limit hit %i", msg.data);    
  ROS_INFO("speed cmd %i", speedCmd);  
  ROS_INFO("forward pressed %i", calibrationProgress.forwardDepressed);
  ROS_INFO("rear pressed %i", calibrationProgress.rearDepressed);
}

void linearStepsCallBack(const std_msgs::Int16& msg) 
{
   steps = msg.data;
   
   if(calibrationProgress.rearDepressed && calibrationProgress.forwardDepressed) {
      if(abs(steps - midPoint < 5)){
        ROS_INFO("finished homing");
        
        // reset operation mode to speed control
        std_msgs::UInt8 speedModeCmd;
        speedModeCmd.data = LINEAR_MOTOR;
        speedModePub.publish(speedModeCmd);

        std_msgs::Int16 linearSpeedCmd;
        linearSpeedCmd.data = 0;
        speedPub.publish(linearSpeedCmd);
        
        system("rosrun foos_control openLoop");
        ros::shutdown();
      }
   }
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
  speedModePub = n.advertise<std_msgs::UInt8>("motor_speed_mode_cmd", 10);
  calibrationPub = n.advertise<foos_control::RailCalibration>("linear_calibration", 10);

  ros::Rate loop_rate(10);

  ROS_INFO("waiting for user input");
  
  std_msgs::UInt8 speedModeCmd;
  speedModeCmd.data = LINEAR_MOTOR;
  speedModePub.publish(speedModeCmd);
  
  std_msgs::Int16 linearSpeedCmd;
  
  while (ros::ok())
  {
  
    linearSpeedCmd.data = speedCmd;
    speedPub.publish(linearSpeedCmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;

}
