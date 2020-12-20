#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <arduino-timer.h>

AccelStepper stepper(AccelStepper::DRIVER, 9, 8);


typedef enum {
  REVERSE = -1,
  NEUTRAL = 0,
  FORWARD = 1,
} direction_t;

int stallDirection = NEUTRAL;

bool drivingIntoStall(int stallDirection, int speedCommand) {
  return (stallDirection == FORWARD && speedCommand > 0) || (stallDirection == REVERSE && speedCommand < 0);
}

void servo_cb( const std_msgs::Int16& cmd_msg){

  int speedCommand = cmd_msg.data;
  stepper.setSpeed(speedCommand);
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

const int rearLimitPin = 2;
const int forwardLimitPin = 3;

void setup() {
  stepper.setMaxSpeed(20000);
  stepper.setSpeed(0);

  nh.initNode();
  nh.advertise(stepsPub);
  nh.advertise(speedPub);
  nh.subscribe(sub);

  timer.every(100, timerCallback);

  pinMode(rearLimitPin, INPUT);
  pinMode(forwardLimitPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

int checkLimitSwitches(const int forwardSwitchPin, const int rearSwitchPin) {
  int forwardDepressed = digitalRead(forwardSwitchPin) == LOW;
  int rearDepressed = digitalRead(rearSwitchPin) == LOW;

  int ledOutput = (forwardDepressed || rearDepressed);
  digitalWrite(LED_BUILTIN, ledOutput);

  if(forwardDepressed) {
    return FORWARD;
  } else if(rearDepressed) {
    return REVERSE;
  } else {
    return NEUTRAL;
  }
}

void checkStalled() {
  stallDirection = checkLimitSwitches(forwardLimitPin, rearLimitPin);

  if(stallDirection != NEUTRAL) {
    int speedCommand = stepper.speed();
    if(drivingIntoStall(stallDirection, speedCommand)){
      stepper.setSpeed(0);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  timer.tick();
  checkStalled();
  stepper.runSpeed();
}
