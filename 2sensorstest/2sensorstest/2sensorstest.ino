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

const int trigPinFront = 7; //pin for sending the sound
const int echoPinFront = 8; //pin for receiving the sound
const int trigPinRight = 4; //pin for sending the sound
const int echoPinRight = 9; //pin for receiving the sound

long duration; //time the sound takes to travel
long distanceFront; //distance in cm, will be calculated from the duration
long distanceRight;

int minDistance = 28;

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

  pinMode(trigPinFront, OUTPUT); //output, since arduino tells the trigger when to send the sound
  pinMode(echoPinFront, INPUT); // input, since the echo tells the arduino how long it was on for

  pinMode(trigPinRight, OUTPUT); //output, since arduino tells the trigger when to send the sound
  pinMode(echoPinRight, INPUT); // input, since the echo tells the arduino how long it was on for

  Serial.begin(9600); //to output the distance to the console

}//end setup

void loop() {
  // put your main code here, to run repeatedly:
  
  rightMaze();

}//end loop

void rightMaze(){

  distanceFront = getDistanceFront();

  if(distanceFront < 8){

    distanceRight = getDistanceRight();
    
    if(distanceRight > minDistance){
      
      turnRight();
      waitUntilPulseCount(16);
      moveForwards();
      waitUntilPulseCount(30);
      stopRobot();
      
    } else {

      turnLeft();
      waitUntilPulseCount(16);
      stopRobot();
      
    }//end if else
    
  } else {

    distanceRight = getDistanceRight();
    
    if(distanceRight > minDistance){

      moveForwards();
      waitUntilPulseCount(20);
      turnRight();
      waitUntilPulseCount(16);
      moveForwards();
      waitUntilPulseCount(30);
      stopRobot();
      
    }//end if distance right

    moveForwards();
    
  }//end if 

  adjustRobot();
  
}//end rightMaze

void adjustRobot(){
  
  distanceRight = getDistanceRight();
  if(distanceRight < 4){
    //get away from the wall
    turnLeft();
    waitUntilPulseCount(4);
  } else if (distanceRight < 8) {
    analogWrite(motorB1, 200);
    analogWrite(motorA2, 200);
  } else if (distanceRight < 12){
    analogWrite(motorB1, 150);
    analogWrite(motorA2, 200);
  } else if (distanceRight < 20){
    analogWrite(motorB1, 110);
    analogWrite(motorA2, 250);
  } else if (distanceRight < minDistance){
    turnRight();
    waitUntilPulseCount(4);
  }//end adjustment
  
}

int getDistanceFront(){
  digitalWrite(trigPinFront, LOW); //clear the trig pin
  delay(2);
  
  digitalWrite(trigPinFront, HIGH); //generate sound
  delay(10); //generate sound for 10ms
  digitalWrite(trigPinFront, LOW); //stop generating sound

  duration = pulseIn(echoPinFront, HIGH); //reads how long until the echo pin received the sound. echo pin is set to high until it gets the sound, and then its low
  distanceFront = (duration * 0.034)/2; //distance calculation in CM, 

  Serial.print("Distance front: ");
  Serial.println(distanceFront);

  return distanceFront;
}//end getDistance

int getDistanceRight(){
  digitalWrite(trigPinRight, LOW); //clear the trig pin
  delay(2);
  
  digitalWrite(trigPinRight, HIGH); //generate sound
  delay(10); //generate sound for 10ms
  digitalWrite(trigPinRight, LOW); //stop generating sound

  duration = pulseIn(echoPinRight, HIGH); //reads how long until the echo pin received the sound. echo pin is set to high until it gets the sound, and then its low
  distanceRight = (duration * 0.034)/2; //distance calculation in CM, 

  Serial.print("Distance right: ");
  Serial.println(distanceRight);

  return distanceRight;
}//end getDistance


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

    if (millis() - lastPulseTime >= MaxPulseLength){
      // No pulse state change for a while.  Must have hit a stop
      moveBackwards();
      delay(300);
      turnLeft();
      waitUntilPulseCountLeft(4); 
      stopRobot();
      PulseCountRight = 0;
      PulseCountLeft = 0;
      return;
    }

    if(right == true && left == true){
      return;
    }
    
  }//end while

  PulseCountRight = 0;
  PulseCountLeft = 0;
  return;
  
}//end waitUntilBoth

void waitUntilPulseCountLeft(unsigned long count){
  int previousPulseStateLeft = digitalRead(pulsePinLeft);
  unsigned long lastPulseTime = millis();

  while (PulseCountLeft < count){
    int pulseStateLeft = digitalRead(pulsePinLeft);

    if (pulseStateLeft != previousPulseStateLeft){
      // State change
      previousPulseStateLeft = pulseStateLeft;
      PulseCountLeft++;
      lastPulseTime = millis();

      if (millis() - lastPulseTime >= MaxPulseLength){
        // No pulse state change for a while.  Must have hit a stop
        moveBackwards();
        delay(300);
        stopRobot();
        PulseCountLeft = 0;
        return;
      }
    }
    
  }//end while

  PulseCountLeft = 0;
  return;
  
}//end waitUntilLeft

void waitUntilPulseCountRight(unsigned long count){
  int previousPulseStateRight = digitalRead(pulsePinRight);
  unsigned long lastPulseTime = millis();

  while (PulseCountRight < count){
    int pulseStateRight = digitalRead(pulsePinRight);

    if (pulseStateRight != previousPulseStateRight){
      // State change
      previousPulseStateRight = pulseStateRight;
      PulseCountRight++;
      lastPulseTime = millis();
      
      if (millis() - lastPulseTime >= MaxPulseLength){
        // No pulse state change for a while.  Must have hit a stop
        moveBackwards();
        delay(300);
        stopRobot();
        PulseCountRight = 0;
        return;
      }//end if
    }//end if
    
  }//end while

  PulseCountRight = 0;
  return;
  
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

  delay(10);

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

  delay(10);

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

  delay(10);

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

  delay(10);

  analogWrite(motorA1, 0);
  analogWrite(motorA2, 200);
  analogWrite(motorB1, 0);
  analogWrite(motorB2, 200);
}

void gripperOpen(){
  for(int i = 0;i < 8; i++){
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(1700); // Duration of the pulse in microseconds
    digitalWrite(gripperPin, LOW);
    // Pulses duration: 1700 = open
    delay(20);
  }
}

void gripperClose(){
  for(int i = 0;i < 8; i++){
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(1050); // Duration of the pulse in microseconds
    digitalWrite(gripperPin, LOW);
    // Pulses duration: 1050 = fully closed
    delay(20);
  }
}
