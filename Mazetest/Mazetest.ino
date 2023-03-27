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

int minDistance = 28;
boolean programStart = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(gripperPin, OUTPUT);
  pinMode(rotatorPin, OUTPUT);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(pulsePinLeft, INPUT);

  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(pulsePinRight, INPUT);

  pinMode(trigPin, OUTPUT); //output, since arduino tells the trigger when to send the sound
  pinMode(echoPin, INPUT); // input, since the echo tells the arduino how long it was on for

  Serial.begin(9600); //to output the distance to the console
}//end setup

void loop() {
  // put your main code here, to run repeatedly:
  gripperClose();
  leftMaze();
  delay(500);
  
}//end loop

void leftMaze(){

  if(programStart){
    sensorCenter();
    delay(500);
    distance = getDistance();

  
    if(distance > 8){
      Serial.println("ONWARDS");
      moveToWall();
      stopRobot();
    }
  }

  programStart = true;

  sensorLeft();
  delay(500);
  distance = getDistance();
  
  if(distance > minDistance){
    
    Serial.println("HARD TO PORT");
    turnLeft();
    waitUntilPulseCount(16);
    stopRobot();
    
  } else {

    Serial.println("Nothing left");
    sensorCenter();
    delay(500);
    distance = getDistance();

    if(distance > 8){
      //do nothing
      Serial.println("Nothing left to do");
    } else {

      Serial.println("Nothing center");
      sensorRight();
      delay(500);
      distance = getDistance();

      if(distance > minDistance){
        
        Serial.println("RIGHT");
        turnRight();
        waitUntilPulseCount(15);
        stopRobot();
        
      } else {
        
        Serial.println("BACKTRACK");
        turnAround();
        delay(100);
        
      }//end if right
      
    }//end if forward
    
  }//end if left

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

  sensorLeft();
  delay(500);
  int distanceLeft = getDistance();

  sensorRight();
  delay(500);
  int distanceRight = getDistance();
  
  if(distanceLeft > distanceRight){

    moveLeftWheelBackwards();
    waitUntilPulseCountLeft(40);
    stopRobot();
  
    delay(100);
  
    moveRightWheelForwards();
    waitUntilPulseCountRight(30);
    stopRobot();
    
  } else {

    moveRightWheelBackwards();
    waitUntilPulseCountRight(30);
    stopRobot();
  
    delay(100);
  
    moveLeftWheelForwards();
    waitUntilPulseCountLeft(40);
    stopRobot();
    
  }//end if

  delay(100);

  moveForwards();
  waitUntilPulseCount(20);
  stopRobot();
}//end turnAround

void tooLeft(){

  sensorLeft();
  delay(500);
  int distanceLeft = getDistance();
  
  if (distanceLeft < 8){
    turnRight();
    waitUntilPulseCount(5);
    stopRobot();
  }
  
}

void tooRight(){

  sensorRight();
  delay(500);
  int distanceRight = getDistance();
  
  if (distanceRight < 8){
    turnLeft();
    waitUntilPulseCount(5);
    stopRobot();
  }
  
}

void moveToWall(){

  sensorCenter();
  delay(500);
  distance = getDistance();
  int lastDistance;
  int counter = 0;
  
  if(distance < 40){
    
    while(distance > 8 && distance < 40){
      
      lastDistance = distance;
      moveForwards();
      
      distance = getDistance();
      if(distance == lastDistance || distance == lastDistance+1){
        counter++;
        Serial.println(counter);
        if(counter > 10){
          
          moveBackwards();
          waitUntilPulseCount(20);
          stopRobot();
          delay(100);
          moveBackwards();
          delay(300);
          stopRobot();
          tooLeft();
          tooRight();

          counter = 0;
          
          sensorCenter();
          
        }//end if
      } else {
        counter = 0;
      }//end if else
      delay(20);
      
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
      if (PulseCountRight >= count){
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

  analogWrite(motorA1, 0);
  analogWrite(motorA2, 200);
  analogWrite(motorB1, 200);
  analogWrite(motorB2, 0);
}

void moveBackwards(){
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);

  analogWrite(motorA1, 200);
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 0);
  analogWrite(motorB2, 200);
}

void turnLeft(){
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);

  analogWrite(motorA1, 200);
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 200);
  analogWrite(motorB2, 0);
}

void turnRight(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);

  analogWrite(motorA1, 0);
  analogWrite(motorA2, 200);
  analogWrite(motorB1, 0);
  analogWrite(motorB2, 200);
}

void moveLeftWheelForwards(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);

  analogWrite(motorA1, 0);
  analogWrite(motorA2, 200);
}

void moveLeftWheelBackwards(){
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);

  analogWrite(motorA1, 200);
  analogWrite(motorA2, 0);
}

void moveRightWheelForwards(){
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);

  analogWrite(motorB1, 200);
  analogWrite(motorB2, 0);
}

void moveRightWheelBackwards(){
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);

  analogWrite(motorB1, 0);
  analogWrite(motorB2, 200);
}

void gripperOpen(){
  for(int i = 0;i < 4; i++){
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(1700); // Duration of the pulse in microseconds
    digitalWrite(gripperPin, LOW);
    // Pulses duration: 1700 = open
    delay(20);
  }
}

void gripperClose(){
  for(int i = 0;i < 4; i++){
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(1050); // Duration of the pulse in microseconds
    digitalWrite(gripperPin, LOW);
    // Pulses duration: 1050 = fully closed
    delay(20);
  }
}

void sensorRight(){
  for(int i = 0;i < 4; i++){
    digitalWrite(rotatorPin, HIGH);
    delayMicroseconds(350); // Duration of the pusle in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 350ms = 0 degrees
    delay(20);
  }
}

void sensorLeft(){
  for(int i = 0;i < 4; i++){
    digitalWrite(rotatorPin, HIGH);
    delayMicroseconds(2310); // Duration of the pusle in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 2310ms - 180deg
    delay(20);
  }
}

void sensorCenter(){
  for(int i = 0;i < 4; i++){
    digitalWrite(rotatorPin, HIGH);
    delayMicroseconds(1310); // Duration of the pusle in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 1310ms = 90 degrees
    delay(20);
  }
}
