#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int16MultiArray.h"

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

void linearCalibrationCallBack(const std_msgs::Int16MultiArray::ConstPtr& array) {
	int min = array->data[0];
	int max = array->data[1];
	robot.rail.setLimits(min, max);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "startup");
  ros::NodeHandle n;

  ros::Subscriber limitSub = n.subscribe("linear_calibration", 10, linearCalibrationCallBack);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {


    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;

}
