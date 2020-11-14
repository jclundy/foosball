// Example7 for Brian Schmalz's Easy Driver Example page
// http://www.schmalzhaus.com/EasyDriver/EasyDriverExamples.html
// We control the direction and speed of a stepper using the
// arduino serial port. Note that (if using the Serial Monitor)
// you will need to press Enter after each command.

#include <AccelStepper.h>

// 9 - step
// 8 - dir
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

int spd = 0;

void setup()
{ 
  Serial.begin(9600);
  stepper.setMaxSpeed(10000);
  stepper.setSpeed(0);   
}
// d010
void loop()
{ 
  char c;
  if(Serial.available() >= 4) {
    char incoming[3] = {'0', '0', '0'};
    int i = 0;
    while (Serial.available() > 0) {
        // read the incoming byte:
        incoming[i] = Serial.read();
        i++;
    }
    //Convert the incoming string into an integer!
    int newNumber = atoi(incoming);
    spd = newNumber * 100;
    Serial.println(newNumber);
    stepper.setSpeed(spd);
  }
  stepper.runSpeed();
}
