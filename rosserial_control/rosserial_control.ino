#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

void servo_cb( const std_msgs::UInt16& cmd_msg){
  stepper.setSpeed(cmd_msg.data);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> sub("speed", servo_cb);

std_msgs::UInt16 stepsMsg;
ros::Publisher stepsPub("steps", &stepsMsg);

void setup() {
  stepper.setMaxSpeed(10000);
  stepper.setSpeed(0);

  nh.initNode();
  nh.advertise(stepsPub);
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  stepper.runSpeed();

  stepsMsg.data = stepper.currentPosition();
  stepsPub.publish(&stepsMsg);
  delay(1);
}
