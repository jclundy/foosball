#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int16.h>

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

void setup() {
  stepper.setMaxSpeed(10000);
  stepper.setSpeed(0);

  nh.initNode();
  nh.advertise(stepsPub);
  nh.advertise(speedPub);
  nh.subscribe(sub);
}
int count = 0;
void loop() {
  // put your main code here, to run repeatedly:

  count++;
  if (count > 100) {
    nh.spinOnce();  
    stepsMsg.data = stepper.currentPosition();
    stepsPub.publish(&stepsMsg);
  
    speedMsg.data = stepper.speed();
    speedPub.publish(&speedMsg);

    count = 0;
  }
  stepper.runSpeed();
}
