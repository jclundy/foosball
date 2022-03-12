#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/Joy.h"

#include "foos_control/RailCalibration.h"
#include "foos_control/GetLinearCalibration.h"

#include "joystick_definitions.h"
#include "control_definitions.h"

class LinearRail {
	int maxLimit, minLimit;
	public:
		LinearRail();
		int getMaxLimit();
		int getMinLimit();
		void setLimits(int min, int max);
};

LinearRail::LinearRail() {
	maxLimit = 0;
	minLimit = 0;
}

void LinearRail::setLimits(int min, int max) {
	maxLimit = max;
	minLimit = min;
}

int LinearRail::getMaxLimit() {
	return maxLimit;
}

int LinearRail::getMinLimit() {
	return minLimit;
}

class Foosbot {
	public:
		LinearRail rail;
		Foosbot();
};

Foosbot::Foosbot() {
	rail = LinearRail();
}

Foosbot robot;

class Button {
    public:
        bool prevState;
        bool currentState;
        float filteredValue;
        float emaWeight;
        Button(float w);
        void update(bool value);
};

Button::Button(float w) {
    prevState = false;
    currentState = false;
    filteredValue = 0;
    emaWeight = w;
}

void Button::update(bool value) {
    prevState = currentState;
    
    filteredValue = filteredValue * (1-emaWeight) + emaWeight * value;
    currentState = (filteredValue >= 0.5);
}

Button calibrateWristButton(0.1);

ros::Publisher resetPositionPub;

void linearCalibrationCallBack(const foos_control::RailCalibration& msg) {
	robot.rail.setLimits(msg.min, msg.max);
	ROS_INFO("saving limits min=%i; max=%i", msg.min, msg.max); 
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  bool buttonValue = joy->axes[A_BUTTON_INDEX];
  calibrateWristButton.update(buttonValue);
  
  if(calibrateWristButton.currentState && !calibrateWristButton.prevState) {
      std_msgs::UInt8 resetPositionMsg;
      resetPositionMsg.data = WRIST_MOTOR;
      resetPositionPub.publish(resetPositionMsg);
  }
  
}

bool linearCalibrationRequest(foos_control::GetLinearCalibration::Request  &req, 
				foos_control::GetLinearCalibration::Response &res) {
	res.min = robot.rail.getMinLimit();
	res.max = robot.rail.getMaxLimit();
	
	return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "supervisor");
  ros::NodeHandle n;

  resetPositionPub = n.advertise<std_msgs::UInt8>("reset_steps_cmd", 10);

  ros::Subscriber limitSub = n.subscribe("linear_calibration", 10, linearCalibrationCallBack);
  ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
  
  ros::ServiceServer service = n.advertiseService("linear_calibration_info", linearCalibrationRequest);
  
  ROS_INFO("Advertising linear calibration info.");
  
	ros::spin();


  return 0;

}
