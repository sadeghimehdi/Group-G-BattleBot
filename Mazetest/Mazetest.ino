#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/****************************************************************************
 ***                             Millis                                   ***
 ****************************************************************************/
unsigned long previousMillis_1 = 0
const long interval_1 = 5000; //interval for first event (5seconds)
int theEnd = 0;
int lastSensor = NULL;



boolean mazeComplete = false;
boolean startup = true;
boolean inmaze = false;
boolean activateLineSensors = false;
boolean programStart = false;



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

long distanceLeft;
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

  pinMode(trigPin, OUTPUT); //output, since arduino tells the trigger when to send the sound
  pinMode(echoPin, INPUT); // input, since the echo tells the arduino how long it was on for

  Serial.begin(9600); //to output the distance to the console

  startupSequence();
}//end setup

void loop() {
  // put your main code here, to run repeatedly:
  if(!mazeComplete){
    
    if(startup){
      startFollowingLine();
      startup = false;
      inMaze = true;
    }//end startup

    if(inMaze){
      gripperClose();
      rightMaze();
      delay(500);
    }//end inmaze

    if(!inMaze && !startup){
      lineFollowAndEnd();
    }//end linefollow
    
  }//end if
}//end loop

void leftMaze(){

  if(activateLineSensors){
    if(sensorValues[0] > 600 || sensorValues[1] > 600 || sensorValues[2] > 600 || sensorValues[3] > 600 || sensorValues[4] > 600 || sensorValues[5] > 600 || sensorValues[6] > 600 || sensorValues[7] > 600){
      inMaze = false;
      return;
    }//end if
  }//end activate line sensors
  
  if(programStart){
    sensorCenter();
    delay(300);
    distance = getDistance();
  
    if(distance > 8){
      Serial.println("ONWARDS");
      moveToWall();
      stopRobot();
    }//end if distance

    activateLineSensors = true;
    
  }//end if programstart

  programStart = true;

  sensorLeft();
  delay(600);
  distance = getDistance();

  //double checks if the left distance is actually open
  //too many special cases for it to be random chance
  if(distance < minDistance){
    turnLeftSlow();
    waitUntilPulseCount(5);
    stopRobot();

    distance = getDistance();

    if(distance < minDistance){
      turnRightSlow();
      waitUntilPulseCount(5);
      stopRobot();
    }//end if
  }//end doublecheck for left
  
  if(distance > minDistance){
    
    Serial.println("HARD TO PORT");
    turnLeft();
    waitUntilPulseCount(16);
    stopRobot();
    
  } else {

    Serial.println("Nothing left");
    sensorCenter();
    delay(300);
    distance = getDistance();

    if(distance > 8){
      //do nothing
      Serial.println("Nothing left to do");
    } else {

      Serial.println("Nothing center");
      sensorRight();
      delay(300);
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

  if(activateLineSensors){
    if(sensorValues[0] > 600 || sensorValues[1] > 600 || sensorValues[2] > 600 || sensorValues[3] > 600 || sensorValues[4] > 600 || sensorValues[5] > 600 || sensorValues[6] > 600 || sensorValues[7] > 600){
      inMaze = false;
      return;
    }//end if
  }//end activate line sensors
  
  if(programStart){
    sensorCenter();
    delay(300);
    distance = getDistance();
  
    if(distance > 8){
      Serial.println("ONWARDS");
      moveToWall();
      stopRobot();
    }//end if distance

    activateLineSensors = true;
    
  }//end if programstart

  programStart = true;

  sensorRight();
  delay(600);
  distance = getDistance();

  //double checks if the left distance is actually open
  //too many special cases for it to be random chance
  if(distance < minDistance){
    turnRightSlow();
    waitUntilPulseCount(5);
    stopRobot();

    distance = getDistance();

    if(distance < minDistance){
      turnLeftSlow();
      waitUntilPulseCount(5);
      stopRobot();
    }//end if
  }//end doublecheck for left
  
  if(distance > minDistance){
    
    Serial.println("HARD TO STARBOARD");
    turnRight();
    waitUntilPulseCount(16);
    stopRobot();
    
  } else {

    Serial.println("Nothing right");
    sensorCenter();
    delay(300);
    distance = getDistance();

    if(distance > 8){
      //do nothing
      Serial.println("Nothing right to do");
    } else {

      Serial.println("Nothing center");
      sensorLeft();
      delay(300);
      distance = getDistance();

      if(distance > minDistance){
        
        Serial.println("LEFT");
        turnLeft();
        waitUntilPulseCount(16);
        stopRobot();
        
      } else {
        
        Serial.println("BACKTRACK");
        turnAround();
        delay(100);
        
      }//end if left
      
    }//end if forward
    
  }//end if left

  tooRight();
  tooLeft();
  
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
  delay(600);
  distanceLeft = getDistance();

  sensorRight();
  delay(600);
  distanceRight = getDistance();
  
  if(distanceLeft > distanceRight){

    moveLeftWheelBackwards();
    waitUntilPulseCountLeft(40);
    stopRobot();
    
    moveRightWheelForwards();
    waitUntilPulseCountRight(30);
    stopRobot();
    
  } else {

    moveRightWheelBackwards();
    waitUntilPulseCountRight(30);
    stopRobot();
    
    moveLeftWheelForwards();
    waitUntilPulseCountLeft(40);
    stopRobot();
    
  }//end if

  moveForwards();
  waitUntilPulseCount(20);
  stopRobot();
  
}//end turnAround

void tooLeft(){

  sensorLeft();
  delay(600);
  distanceLeft = getDistance();
  
  if (distanceLeft < 8){
    turnRightSlow();
    waitUntilPulseCount(5);
    stopRobot();
  }
  
}

void tooRight(){

  sensorRight();
  delay(600);
  distanceRight = getDistance();
  
  if (distanceRight < 8){
    turnLeftSlow();
    waitUntilPulseCount(5);
    stopRobot();
  }
  
}

void moveToWall(){

  sensorCenter();
  delay(300);
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
          
          stopRobot();
          delay(100);

          sensorLeft();
          delay(600);
          distanceLeft = getDistance();
          sensorRight();
          delay(600);
          distanceRight = getDistance();

          if(distanceLeft > 30 || distanceRight > 30){

            moveBackwards();
            delay(300);
            stopRobot();
  
            if(distanceLeft > distanceRight(){
              turnRightSlow();
              waitUntilPulseCount(5);
              stopRobot();
            } else {
               turnLeftSlow();
               waitUntilPulseCount(5);
               stopRobot();
            }//end if distance

          } else {
            tooLeft();
            tooRight();
          }//end if else

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
    waitUntilPulseCount(45);
    stopRobot();
    
  }//end if else
  
}//end movetowall

void waitUntilPulseCount(unsigned long count){
  int previousPulseStateLeft = digitalRead(pulsePinLeft);
  int previousPulseStateRight = digitalRead(pulsePinRight);
  unsigned long lastPulseTime = millis();

  boolean right = false;
  boolean left = false;

  while (!right && !left){
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
      stopRobot();
      PulseCountRight = 0;
      PulseCountLeft = 0;
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

void turnLeftSlow(){
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);

  analogWrite(motorA1, 150);
  analogWrite(motorA2, 0);
  analogWrite(motorB1, 150);
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

void turnRightSlow(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);

  analogWrite(motorA1, 0);
  analogWrite(motorA2, 150);
  analogWrite(motorB1, 0);
  analogWrite(motorB2, 150);
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
    delayMicroseconds(350); // Duration of the pulse in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 350ms = 0 degrees
    delay(20);
  }
}

void sensorLeft(){
  for(int i = 0;i < 4; i++){
    digitalWrite(rotatorPin, HIGH);
    delayMicroseconds(2310); // Duration of the pulse in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 2310ms - 180deg
    delay(20);
  }
}

void sensorCenter(){
  for(int i = 0;i < 4; i++){
    digitalWrite(rotatorPin, HIGH);
    delayMicroseconds(1310); // Duration of the pulse in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 1310ms = 90 degrees
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
  Motor(255,255,0,0);

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
  
  // put your main code here, to run repeatedly:
  if(sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600 && sensorValues[7] > 600 && gripperClosed == 0){
    stopRobot();
    //close gripper
    gripperClose();
   
    delay(100);
    turnLeft();
    waitUntilPulseCount(16);
    moveForwards();
    waitUntilPulseCount(40);
    stopRobot();
    return;
    
  }//end if
  
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
    int previousPulseState = digitalRead(PulsePinRight);
    unsigned long lastPulseTime = millis();
    Motor(200,200,0,0);
    
    while (PulseCountRight < 7)
    {
      Serial.println(PulseCountRight);
      int pulseState = digitalRead(PulsePinRight);
      
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
}//end motor
