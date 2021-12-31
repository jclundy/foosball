#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <arduino-timer.h>

AccelStepper wristMotor(AccelStepper::DRIVER, 7, 6);
AccelStepper linearMotor(AccelStepper::DRIVER, 9, 8);

typedef enum {
  REVERSE = -1,
  NEUTRAL = 0,
  FORWARD = 1,
} direction_t;

typedef enum {
  SPEED_CONTROL,
  POSITION_CONTROL,
} stepper_mode_t;

typedef enum {
  WRIST_MOTOR,
  LINEAR_MOTOR,
} motor_type_t;

int stallDirection = NEUTRAL;
stepper_mode_t wristMode = SPEED_CONTROL;
stepper_mode_t linearMode = SPEED_CONTROL;

bool drivingIntoStall(int stallDirection, int speedCommand) {
  return (stallDirection == FORWARD && speedCommand > 0) || (stallDirection == REVERSE && speedCommand < 0);
}

void wrist_cb( const std_msgs::Int16& cmd_msg){
  int speedCommand = cmd_msg.data;
  wristMotor.setSpeed(speedCommand);
}

void linear_cb( const std_msgs::Int16& cmd_msg){
  
  int speedCommand = cmd_msg.data;
  linearMotor.setSpeed(speedCommand);
}

void setWristPositionCB( const std_msgs::Int16& cmd_msg) {
  wristMode = POSITION_CONTROL;
  int setpoint = cmd_msg.data;
  wristMotor.moveTo(setpoint);  
}

void setLinearPositionCB( const std_msgs::Int16& cmd_msg) {
  linearMode = POSITION_CONTROL;
  int setpoint = cmd_msg.data;
  linearMotor.moveTo(setpoint);  
}

void setControlModeSpeed( const std_msgs::UInt8& cmd_msg) {
  if(cmd_msg.data == WRIST_MOTOR) {
    wristMode = SPEED_CONTROL;
  } else if (cmd_msg.data == LINEAR_MOTOR) {
    linearMode = SPEED_CONTROL;
  }
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> wrist_sub("wrist_speed", wrist_cb);
ros::Subscriber<std_msgs::Int16> linear_sub("linear_speed", linear_cb);

ros::Subscriber<std_msgs::Int16> wrist_position_sub("wrist_position_cmd", setWristPositionCB);
ros::Subscriber<std_msgs::Int16> linear_position_sub("linear_position_cmd", setLinearPositionCB);

ros::Subscriber<std_msgs::UInt8> speed_mode_sub("motor_speed_mode_cmd", setControlModeSpeed);

std_msgs::Int16 linearStepsMsg;
ros::Publisher linearStepsPub("linear_steps", &linearStepsMsg);

std_msgs::Int16 wristStepsMsg;
ros::Publisher wristStepsPub("wrist_steps", &wristStepsMsg);

std_msgs::Int8 limitMsg;
ros::Publisher limitPub("limit_reached", &limitMsg);

Timer<1> timer;
bool timerCallback(void *argument) {
  nh.spinOnce();

  linearStepsMsg.data = linearMotor.currentPosition();
  linearStepsPub.publish(&linearStepsMsg);
  
  wristStepsMsg.data = wristMotor.currentPosition();
  wristStepsPub.publish(&wristStepsMsg);
  
  return true;
}

const int rearLimitPin = 2;
const int forwardLimitPin = 3;

void setup() {
  wristMotor.setMaxSpeed(20000);
  wristMotor.setSpeed(0);

  linearMotor.setMaxSpeed(20000);
  linearMotor.setSpeed(0);

  nh.initNode();
  nh.advertise(linearStepsPub);
  nh.advertise(wristStepsPub);
  nh.advertise(limitPub);
  nh.subscribe(wrist_sub);
  nh.subscribe(linear_sub);
  nh.subscribe(wrist_position_sub);
  nh.subscribe(linear_position_sub);
  nh.subscribe(speed_mode_sub);

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
  int newStallDirection = checkLimitSwitches(forwardLimitPin, rearLimitPin);

  if(newStallDirection != stallDirection){
    stallDirection = newStallDirection;
    limitMsg.data = stallDirection;
    limitPub.publish(&limitMsg);
  }

  if(stallDirection != NEUTRAL) {
    int speedCommand = linearMotor.speed();
    if(drivingIntoStall(stallDirection, speedCommand)){
      linearMotor.setSpeed(0);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  timer.tick();
  checkStalled();

  if(wristMode == SPEED_CONTROL) {
    wristMotor.runSpeed();
  } else {
    wristMotor.runSpeedToPosition();
  }

  if(linearMode == SPEED_CONTROL) {
    linearMotor.runSpeed(); 
  } else {
    linearMotor.runSpeedToPosition();    
  }
}
