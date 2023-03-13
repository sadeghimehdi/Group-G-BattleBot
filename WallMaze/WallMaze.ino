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

const int trigPin = 2; //pin for sending the sound
const int echoPin = 3; //pin for receiving the sound

long duration; //time the sound takes to travel
long distance; //distance in cm, will be calculated from the duration

int minDistance = 20;

void setup() {
  // put your setup code here, to run once:
  gripper.attach(gripperPin);
  rotator.attach(rotatorPin);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);

  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

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

  boolean turn = false;
  
  sensorCenter();
  delay(250);
  distance = getDistance();

  if(distance > minDistance){
    Serial.println("ONWARDS");
    moveToWall();
    stopRobot();
  }

  sensorLeft();
  delay(250);
  distance = getDistance();
  
  if(distance > minDistance){

    Serial.println("HARD TO PORT");
    turnLeft();
    delay(400);
    stopRobot();

    turn = true;
    
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
        
        Serial.println("RIGHT");
        turnRight();
        delay(400);
        stopRobot();

        turn = true;
        
      } else {
        
        Serial.println("BACKTRACK");
        turnAround();
        delay(200);

        turn = true;
        
      }//end if right
      
    }//end if forward
    
  }//end if left

  if(!turn){
    isCentered();
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
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);

  delay(775);

  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);

  delay(775);

  moveForwards();
  delay(300);

  stopRobot();
}//end turnAround

void isCentered(){
  sensorLeft();
  delay(200);
  int distanceLeft = getDistance();
  sensorRight();
  delay(200);
  int distanceRight = getDistance();

  int center = distanceLeft - distanceRight;

  if(distanceLeft > 35 || distanceRight > 35){
    //do nothing lol  
  } else if (center < 0){
    turnRight();
    delay(90);
    stopRobot();
  } else if (center > 0){
    turnLeft();
    delay(90);
    stopRobot();
  }
  
}//end isCentered

void tooLeft(){

  sensorLeft();
  delay(200);
  int distanceLeft = getDistance();
  
  if (distanceLeft < 8){
    turnRight();
    delay(150);
    stopRobot();
  }
  
}

void tooRight(){

  sensorRight();
  delay(200);
  int distanceRight = getDistance();
  
  if (distanceRight < 8){
    turnLeft();
    delay(150);
    stopRobot();
  }
  
}

void moveToWall(){

  sensorCenter();
  
  if(distance40()){
    while(distance > 8){
      moveForwards();
      distance = getDistance();
    }//end while
    stopRobot();
  } else {
    moveForwards();
    delay(950);
    stopRobot();
  }//end if else
  
}//end movetowall

boolean distance40(){
  //because you cant measure more than 40cm in the maze due to interference
  sensorCenter();
  
  int distance1 = getDistance();
  moveForwards();
  delay(300);
  stopRobot();
  delay(200);
  int distance2 = getDistance();

  int change = distance1-distance2;

  moveBackwards();
  delay(300);
  stopRobot();

  if(distance1 > 50 || distance2 > 50){
    return false;
  }//if distance is invalid

  return change >= 4;
  
}//end distance40

void stopRobot(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void moveForwards(){
  analogWrite(motorA1, 0);
  analogWrite(motorA2, 255);
  analogWrite(motorB1, 248);
  analogWrite(motorB2, 0);
}//end moveForwards

void moveBackwards(){
  analogWrite(motorB1, 0);
  analogWrite(motorB2, 243);
  analogWrite(motorA1, 255);
  analogWrite(motorA2, 0);
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

void gripperOpen(){
  gripper.write(140); //instantly opens the gripper to the widest possible opening
}

void gripperClose(){
  gripper.write(42);
}

void sensorLeft(){
  rotator.write(170); //90 degrees left
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
