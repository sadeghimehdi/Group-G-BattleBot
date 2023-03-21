#include <Servo.h>
Servo gripper;
Servo rotator;

const int gripperPin = 12;
const int rotatorPin = 13;
int servoPos = 0;
int rotatorPos = 0;

const int motorA1 = 11; //Left motor, set to HIGH for backwards
const int motorA2 = 10; //Left motor, Set to HIGH for forwards

const int motorB1 = 6; //Right motor, Set to HIGH for forwards
const int motorB2 = 5; //Right motor, Set to HIGH for backwards

const byte pulsePinLeft = 3; //Pin connected to left encoder
const byte pulsePinRight = 2;  //Pin conneccted to right encoder

unsigned long PulseCountLeft;
unsigned long PulseCountRight;

// Timeout value looking for an encoder pulse
const unsigned MaxPulseLength = 2000;

const int trigPin = 7; //pin for sending the sound
const int echoPin = 8; //pin for receiving the sound

long duration; //time the sound takes to travel
long distance; //distance in cm, will be calculated from the duration

int minDistance = 20;

//variables for special case
//if the robot is in the middle of 2 grid sections and there are walls on both sides
//temp fix until moveToWall works
boolean rightWall;
boolean leftWall;

void setup() {
  // put your setup code here, to run once:
  gripper.attach(gripperPin);
  rotator.attach(rotatorPin);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(pulsePinLeft, INPUT);

  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(pulsePinRight, INPUT);

  pinMode(trigPin, OUTPUT); //output, since arduino tells the trigger when to send the sound
  pinMode(echoPin, INPUT); // input, since the echo tells the arduino how long it was on for

  Serial.begin(9600); //to output the distance to the console
}

void loop() {
  // put your main code here, to run repeatedly:
  gripperClose();
  leftMaze();
  delay(1000);
  
}//end loop

void leftMaze(){

  leftWall = false;
  rightWall = false;
  
  sensorCenter();
  delay(250);
  distance = getDistance();

  if(distance > 12){
    Serial.println("ONWARDS");
    moveToWall();
    stopRobot();
  }

  sensorLeft();
  delay(250);
  distance = getDistance();
  
  if(distance > minDistance){
    
    if(25 < distance && distance < 35){
      leftWall = true;
    } else {
      Serial.println("HARD TO PORT");
      turnLeft();
      waitUntilPulseCount(13);
      stopRobot();
    }//end if else
    
  } else {

    Serial.println("Nothing left");
    sensorCenter();
    delay(250);
    distance = getDistance();

    if(distance > minDistance){
      //do nothing
      Serial.println("Nothing left to do");
    } else {

      Serial.println("Nothing center");
      sensorRight();
      delay(250);
      distance = getDistance();

      if(distance > minDistance){
        
        if(25 < distance && distance < 35){
          rightWall = true;
        } else {
          Serial.println("RIGHT");
          turnRight();
          waitUntilPulseCount(14);
          stopRobot();
        }//end if else
        
      } else {
        
        Serial.println("BACKTRACK");
        turnAround();
        delay(200);
        
      }//end if right
      
    }//end if forward
    
  }//end if left

  if(rightWall && leftWall){
    Serial.println("HARD TO PORT");
    turnLeft();
    waitUntilPulseCount(13);
    stopRobot();
  }

  tooLeft();
  tooRight();
  
}//end leftMaze

void rightMaze(){
  
}//end rightMaze

int getDistance(){
  digitalWrite(trigPin, LOW); //clear the trig pin
  delay(2);
  
  digitalWrite(trigPin, HIGH); //generate sound
  delay(10); //generate sound for 10ms
  digitalWrite(trigPin, LOW); //stop generating sound

  duration = pulseIn(echoPin, HIGH); //reads how long until the echo pin received the sound. echo pin is set to high until it gets the sound, and then its low
  distance = (duration * 0.034)/2; //distance calculation in CM, 

  Serial.print("Distance: ");
  Serial.println(distance);

  return distance;
}//end getDistance

void turnAround(){
  moveRightWheelBackwards();
  waitUntilPulseCountRight(30);
  stopRobot();

  delay(100);

  moveLeftWheelForwards();
  waitUntilPulseCountLeft(30);
  stopRobot();

  delay(100);

  moveForwards();
  waitUntilPulseCount(20);
  stopRobot();
}//end turnAround

void tooLeft(){

  sensorLeft();
  delay(200);
  int distanceLeft = getDistance();
  
  if (distanceLeft < 10){
    turnRight();
    waitUntilPulseCount(3);
    stopRobot();
  }
  
}

void tooRight(){

  sensorRight();
  delay(200);
  int distanceRight = getDistance();
  
  if (distanceRight < 10){
    turnLeft();
    waitUntilPulseCount(3);
    stopRobot();
  }
  
}

void moveToWall(){

  sensorCenter();
  distance = getDistance();
  int lastDistance;
  int counter = 0;
  
  if(distance < 35){
    while(distance > 10){
      lastDistance = distance;
      moveForwards();
      distance = getDistance();
      if(distance == lastDistance){
        counter++;
        if(counter > 5){
          moveBackwards();
          delay(300);
          stopRobot();
        }//end if
      } else {
        counter = 0;
      }//end if else
    }//end while
    stopRobot();
  } else {
    moveForwards();
    waitUntilPulseCount(50);
    stopRobot();
  }//end if else
  
}//end movetowall

void waitUntilPulseCount(unsigned long count){
  int previousPulseStateLeft = digitalRead(pulsePinLeft);
  int previousPulseStateRight = digitalRead(pulsePinRight);
  unsigned long lastPulseTime = millis();

  boolean right = false;
  boolean left = false;

  while (1){
    int pulseStateLeft = digitalRead(pulsePinLeft);
    int pulseStateRight = digitalRead(pulsePinRight);

    if (pulseStateLeft != previousPulseStateLeft){
      // State change
      previousPulseStateLeft = pulseStateLeft;
      PulseCountLeft++;
      lastPulseTime = millis();
      if (PulseCountLeft >= count){
        PulseCountLeft = 0;
        left = true;
        analogWrite(motorA1, 0);
        analogWrite(motorA2, 0);
      }
    }

    if (pulseStateRight != previousPulseStateRight){
      // State change
      previousPulseStateRight = pulseStateRight;
      PulseCountRight++;
      lastPulseTime = millis();
      if (PulseCountRight >= count+1){
        PulseCountRight = 0;
        right = true;
        analogWrite(motorB1, 0);
        analogWrite(motorB2, 0);
      }
    }

    if(right == true && left == true){
      return;
    }

    if (millis() - lastPulseTime >= MaxPulseLength){
      // No pulse state change for a while.  Must have hit a stop
      moveBackwards();
      delay(300);
      stopRobot();
      return;
    }
    
  }//end while
  
}//end waitUntilBoth

void waitUntilPulseCountLeft(unsigned long count){
  int previousPulseStateLeft = digitalRead(pulsePinLeft);
  unsigned long lastPulseTime = millis();

  while (1){
    int pulseStateLeft = digitalRead(pulsePinLeft);

    if (pulseStateLeft != previousPulseStateLeft){
      // State change
      previousPulseStateLeft = pulseStateLeft;
      PulseCountLeft++;
      lastPulseTime = millis();
      if (PulseCountLeft >= count){
        PulseCountLeft = 0;
        return;
      }

      if (millis() - lastPulseTime >= MaxPulseLength){
        // No pulse state change for a while.  Must have hit a stop
        moveBackwards();
        delay(300);
        stopRobot();
        return;
      }
    }
    
  }//end while
  
}//end waitUntilLeft

void waitUntilPulseCountRight(unsigned long count){
  int previousPulseStateRight = digitalRead(pulsePinRight);
  unsigned long lastPulseTime = millis();

  while (1){
    int pulseStateRight = digitalRead(pulsePinRight);

    if (pulseStateRight != previousPulseStateRight){
      // State change
      previousPulseStateRight = pulseStateRight;
      PulseCountRight++;
      lastPulseTime = millis();
      if (PulseCountRight >= count){
        PulseCountRight = 0;
        return;
      }

      if (millis() - lastPulseTime >= MaxPulseLength){
        // No pulse state change for a while.  Must have hit a stop
        moveBackwards();
        delay(300);
        stopRobot();
        return;
      }
    }
    
  }//end while
  
}//end waitUntilRight

void stopRobot(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void moveForwards(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void moveBackwards(){
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void turnLeft(){
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void turnRight(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void moveLeftWheelForwards(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
}

void moveLeftWheelBackwards(){
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
}

void moveRightWheelForwards(){
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void moveRightWheelBackwards(){
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void gripperOpen(){
  gripper.write(130); //instantly opens the gripper to the widest possible opening
}

void gripperClose(){
  gripper.write(50);
}

void sensorLeft(){
  rotator.write(180); //90 degrees left
  delay(200);
}

void sensorRight(){
  rotator.write(0); //90 degrees right
  delay(200);
}

void sensorCenter(){
  rotator.write(80); //Centers the distance sensor
  delay(200);
}
