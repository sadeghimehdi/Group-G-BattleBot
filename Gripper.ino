#include <Servo.h>
Servo gripper;

const int gripperPin = 3;

int servoPos = 0;

void setup() {
  // put your setup code here, to run once:
  gripper.attach(gripperPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  gripperClose();
  delay(1000);
  gripperOpen();
  delay(1000);
}

void gripperOpen(){
  for (servoPos = 0; servoPos <= 180; servoPos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    gripper.write(servoPos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void gripperClose(){
  for (servoPos = 180; servoPos >= 0; servoPos -= 1) { // goes from 180 degrees to 0 degrees
    gripper.write(servoPos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
