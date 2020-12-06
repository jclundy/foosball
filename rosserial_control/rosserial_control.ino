#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <arduino-timer.h>

AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

void servo_cb( const std_msgs::Int16& cmd_msg){
  stepper.setSpeed(cmd_msg.data);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> sub("speed", servo_cb);

std_msgs::Int16 stepsMsg;
ros::Publisher stepsPub("steps", &stepsMsg);

std_msgs::Int16 speedMsg;
ros::Publisher speedPub("speedAck", &speedMsg);

Timer<1> timer;
bool timerCallback(void *argument) {
  nh.spinOnce();

  stepsMsg.data = stepper.currentPosition();
  stepsPub.publish(&stepsMsg);

  speedMsg.data = stepper.speed();
  speedPub.publish(&speedMsg);
  
  return true;
}

void setup() {
  stepper.setMaxSpeed(20000);
  stepper.setSpeed(0);

  nh.initNode();
  nh.advertise(stepsPub);
  nh.advertise(speedPub);
  nh.subscribe(sub);

  timer.every(100, timerCallback);
}

void loop() {
  // put your main code here, to run repeatedly:

  timer.tick();
  stepper.runSpeed();
}
