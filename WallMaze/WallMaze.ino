#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
volatile uint16_t sensorValues[SensorCount];
/****************************************************************************
 ***                             Millis                                   ***
 ****************************************************************************/
unsigned long previousMillis_1 = 0;
const long interval_1 = 5000; //interval for first event (5seconds)
int theEnd = 0;
int lastSensor = NULL;



boolean mazeComplete = false; //for stopping the robot at the end of the maze
boolean startup = true; //for the startup sequence of the robot
boolean inMaze = false; //for the maze algorithm
boolean activateLineSensors = false; //delaying activation of the line sensors
boolean programStart = false; //for delaying checking forwards of the robot
boolean checkDistance = true; //for detecting the robot



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
const unsigned lineSensorDelay = 5000; //wait 5 seconds before starting line sensors during maze algorithm
int mazeStartTime = 0;

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
  if(!mazeComplete){

    if(checkDistance){
      distanceFront = getDistanceFront();
      if(distanceFront < 30){
        Serial.println("Robot detected");
        delay(5000);
        checkDistance = false;
      }//end if
    }
    
    if(!checkDistance && startup){
      startupSequence();
      startFollowingLine();
      startup = false;
      inMaze = true;
      Serial.println("Startup complete");
    }//end startup

    if(inMaze){
      rightMaze();
    }//end inmaze

    if(!inMaze && !startup){
      Serial.println("Line detected");
      lineFollowAndEnd();
    }//end linefollow
    
  }//end if
}//end loop

void rightMaze(){

  uint16_t position = qtr.readLineBlack(sensorValues);

  if(activateLineSensors){
    if(sensorValues[0] > 800 || sensorValues[1] > 800 || sensorValues[2] > 800 || sensorValues[3] > 800 || sensorValues[4] > 800 || sensorValues[5] > 800 || sensorValues[6] > 800 || sensorValues[7] > 800){
      inMaze = false;
      return;
    }//end if
  }//end activate line sensors

  if(!programStart){
    int mazeStartTime = millis();
  }

  if(programStart && !activateLineSensors){
    if (millis() - mazeStartTime >= lineSensorDelay){
      activateLineSensors = true;
    }
  }//end programStart

  programStart = true;

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

void startupSequence(){

  gripperOpen();

// configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
  // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
  // Call calibrate() 15 times to make calibration take about 0,48 seconds.
  moveForwards();

  for (uint16_t i = 0; i < 15; i++){
    qtr.calibrate();
  }
  
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

}//end startUpSequence

void startFollowingLine(){

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  //The the data most on the right is the left sensor, and vice versa
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
//    if(i< SensorCount+1){
//    Serial.print(i+1);}
    Serial.print('\t');
  }
  Serial.println(position);

  gripperOpen();
  waitUntilPulseCount(10);
  
  //close gripper
  gripperClose();

  moveForwards();
  waitUntilPulseCount(20);
  turnLeft();
  waitUntilPulseCount(16);
  moveForwards();
  waitUntilPulseCount(60);
  stopRobot();
  return;
  
}//end startFollowingLine

void lineFollowAndEnd(){

/****************************************************************************
 ***                          Line Sensor with Motor                      ***
 ****************************************************************************/
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  //The the data most on the right is the left sensor, and vice versa
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
//    if(i< SensorCount+1){
//    Serial.print(i+1);}
    Serial.print('\t');
  }
  Serial.println(position);

//if(sensorValues[0] > 800 || sensorValues[1] > 800 || sensorValues[2] > 800 || sensorValues[3] > 800 || sensorValues[4] > 800 || sensorValues[5] > 800 || sensorValues[6] > 800 || sensorValues[7] > 800){
  int amount = 0;

  if(sensorValues[0] > 800){ amount = amount + 1;};
  if(sensorValues[1] > 800){ amount = amount + 1;};
  if(sensorValues[2] > 800){ amount = amount + 1;};
  if(sensorValues[3] > 800){ amount = amount + 1;};
  if(sensorValues[4] > 800){ amount = amount + 1;};
  if(sensorValues[5] > 800){ amount = amount + 1;};
  if(sensorValues[6] > 800){ amount = amount + 1;};
  if(sensorValues[7] > 800){ amount = amount + 1;};
  Serial.println("The amount is" + String(amount));
  


/****************************************************************************
 ***                       Pulse and Stop & Drop                          ***
 ****************************************************************************/

//so when its at the end
unsigned long currentMillisRight = millis();
if(amount == 8 && theEnd == 0){
  if(currentMillisRight - previousMillis_1 >= interval_1){
    PulseCountRight = 0;   
    int previousPulseState = digitalRead(pulsePinRight);
    unsigned long lastPulseTime = millis();
    Motor(200,200,0,0);
    
    while (PulseCountRight < 7)
    {
      Serial.println(PulseCountRight);
      int pulseState = digitalRead(pulsePinRight);
      
      if (pulseState != previousPulseState)
      {
        // State change
        previousPulseState = pulseState;
        PulseCountRight++;
        lastPulseTime = millis();
      }
      
      
        //stop after a set amount of pulses
        Serial.println("STOP AHHHH");
        theEnd = 1;

    } //end while

    //stop after a set amount of pulses
        Serial.println("STOP AHHHH");
        theEnd = 1;
        
  }
}

if(theEnd == 1 && amount == 8){
    stopRobot();
    gripperOpen();
    delay(100);
    
    moveBackwards();
    waitUntilPulseCount(40);
    stopRobot();
    gripperClose();

    mazeComplete = true;                                                                //END OF PROGRAM VARIABLE, PUT EVERYTHING IN THE LOOP() IN AN if(!mazeComplete) statement
    
}else if (theEnd == 1 && amount < 8)
{ 
  theEnd = 0;
}




/****************************************************************************
 ***                         Line folllowing                             ***
 ****************************************************************************/
 else if(sensorValues[0] < 600 && sensorValues[1] < 600 && sensorValues[2] < 600 && sensorValues[3] < 600 && sensorValues[4] < 600 && sensorValues[5] < 600 && sensorValues[6] < 600 && sensorValues[7] < 600){
  if(lastSensor < 4){
//      Right turn
      Motor(0, 190, 190, 0);
      Serial.print("AAHHHH WHERE IS IT !!!!!!!!!!!!!!!! GOO RRIGGHHTHTHTHT!!!!!!");
  }
  else if (lastSensor > 3){
//     Left turn
     Motor(190, 0, 0, 190);
     Serial.print("AAHHHH WHERE IS IT !!!!!!!!!!!!!!!! GOO LEFFFTT!!!!");
  }

 }  
  else if (sensorValues[7] > 600 && sensorValues[6] < 600){
    Motor(200, 100, 0, 0);
    Serial.print("Go hard left");
    lastSensor = 7;
    Serial.println("Last sensor is " + String(lastSensor));
  }
  else if (sensorValues[7] > 600 && sensorValues[6] > 600){
    Motor(200, 110, 0, 0);
    Serial.print("Go hardish left");
    lastSensor = 7;
    Serial.println("Last sensor is " + String(lastSensor));
  }
  
  else if(sensorValues[6] > 600 && sensorValues[5] < 600){
     Motor(200, 120, 0, 0);
     Serial.println("Go left");
     lastSensor = 6;
     Serial.println("Last sensor is " + String(lastSensor));
  }
  else if(sensorValues[6] > 600 && sensorValues[5] > 600){
     Motor(200, 130, 0, 0);
     Serial.println("Goish left");
     lastSensor = 6;
     Serial.println("Last sensor is " + String(lastSensor));
  }

  
  else if(sensorValues[5] > 600 && sensorValues[4] < 600){
    Motor(200, 130, 0, 0);
    Serial.println("go leftish");
    lastSensor = 5;
    Serial.println("Last sensor is " + String(lastSensor));
    }

      else if(sensorValues[5] > 600 && sensorValues[4] > 600){
    Motor(200, 140, 0, 0);
//  Motor(230, 170, 0, 0);
    Serial.println("go leftish"); 
    lastSensor = 5;
    Serial.println("Last sensor is " + String(lastSensor));
    }

    else if (sensorValues[0] > 600 && sensorValues[1] < 600){
      Motor(100, 200, 0, 0);
      Serial.println("Go hard right");
      lastSensor = 0;
      Serial.println("Last sensor is " + String(lastSensor));
    }

 else if(sensorValues[0] > 600 && sensorValues[1] > 600){
     Motor(110, 200, 0, 0);
     Serial.println("Go right");
     lastSensor = 1;
     Serial.println("Last sensor is " + String(lastSensor));
  }
    
  else if(sensorValues[1] > 600 && sensorValues[2] < 600){
     Motor(120, 200, 0, 0);
     Serial.println("Go right");
     lastSensor = 1;
     Serial.println("Last sensor is " + String(lastSensor));
  }
 
    else if(sensorValues[1] > 600 && sensorValues[2] > 600){
    Motor(120, 200, 0, 0);
    Serial.println("go rightish");
    lastSensor = 2;
    Serial.println("Last sensor is " + String(lastSensor));
  }
  
  else if(sensorValues[2] > 600 && sensorValues[3] < 600){
    Motor(130, 200, 0, 0);
    Serial.println("go rightish");
    lastSensor = 2;
    Serial.println("Last sensor is " + String(lastSensor));
  }

  else if(sensorValues[2] > 600 && sensorValues[3] > 600){
    Motor(140, 200, 0, 0);
//Motor(170, 230, 0, 0);
    Serial.println("go rightish");
    lastSensor = 2;
    Serial.println("Last sensor is " + String(lastSensor));
  }

  else if(sensorValues[3] > 600 && sensorValues[4] > 600){
//    Forward();
//    Motor(170, 170, 0, 0);
    Motor(250, 250, 0, 0);
    Serial.print("go forward");
//    lastSensor = NULL;
    } 
    
  else if(sensorValues[3] > 600 && sensorValues[4] < 600){
    Motor(170, 180, 0, 0);
//  Motor(240, 250, 0, 0);
//    Forward();

    Serial.print("go forwardish");
    lastSensor = 3;
  }
  
  else if(sensorValues[4] > 600 && sensorValues[3] < 600){
    Motor(180, 170, 0, 0);
//        Motor(250, 240, 0, 0);
//    Forward();

    Serial.print("go forwardish");
    lastSensor = 4;
    }  

}//end linefollowandend

void Motor(int x1, int x2, int x3, int x4){
   analogWrite(motorB1, x1);
   analogWrite(motorA2, x2);
   analogWrite(motorB2, x3);
   analogWrite(motorA1, x4);    
}
void Right(){
   analogWrite(motorB1, 0);
   analogWrite(motorA2, 255);
   analogWrite(motorB2, 0);
   analogWrite(motorA1, 0);
  }
  
void Left(){
   analogWrite(motorB1, 255);
   analogWrite(motorA2, 0);
   analogWrite(motorB2, 0);
   analogWrite(motorA1, 0);
  }

void Forward(){
   analogWrite(motorB1, 255);
   analogWrite(motorA2, 255);
   analogWrite(motorB2, 0);
   analogWrite(motorA1, 0);
  }
void Backward(){
   analogWrite(motorB1, 0);
   analogWrite(motorA2, 0);
   analogWrite(motorB2, 255);
   analogWrite(motorA1, 255);
  }

void Stop(){
   analogWrite(motorB1, 0);
   analogWrite(motorA2, 0);
   analogWrite(motorB2, 0);
   analogWrite(motorA1, 0);
  }
