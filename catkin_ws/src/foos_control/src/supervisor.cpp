#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "foos_control/RailCalibration.h"
#include "foos_control/GetLinearCalibration.h"

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

void linearCalibrationCallBack(const foos_control::RailCalibration& msg) {
	robot.rail.setLimits(msg.min, msg.max);
	ROS_INFO("saving limits min=%i; max=%i", msg.min, msg.max); 
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

  ros::Subscriber limitSub = n.subscribe("linear_calibration", 10, linearCalibrationCallBack);
  
  ros::ServiceServer service = n.advertiseService("linear_calibration_info", linearCalibrationRequest);
  
  ROS_INFO("Advertising linear calibration info.");
  
	ros::spin();


  return 0;

}
