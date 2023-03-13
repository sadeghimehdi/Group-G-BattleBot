#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
#include <Servo.h>
Servo gripper; 
const int gripperPin = 2;
int gripperClosed = 0;
int servoPos = 5;
/****************************************************************************
 ***                          Line Sensor                                 ***
 ****************************************************************************/
// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is. Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 5000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.







QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


/****************************************************************************
 ***                             Millis                                   ***
 ****************************************************************************/
unsigned long previousMillis_1 = 0;

const long interval_1 = 5000; //interval for first event (5seconds)


/****************************************************************************
 ***                          Neo Pixels                                  ***
 ****************************************************************************/

// Which pin on the Arduino is connected to the NeoPixels?
#define NEOPIN        10 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 4 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

/****************************************************************************
 ***                      Sonic Sensor(distance)                          ***
 ****************************************************************************/
const int pingPin = 8; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 7; // Echo Pin of Ultrasonic Sensor

/****************************************************************************
 ***                                Motor                                 ***
 ****************************************************************************/

const int motorLeftBack = 5; 
const int motorLeftForward = 6; 
const int motorRightBack = 9; 
const int motorRightForward = 3;
int lastSensor = NULL;


/****************************************************************************
 ***                                Functions                             ***
 ****************************************************************************/

/************************** MOTORS *****************************/
void Motor(int x1, int x2, int x3, int x4){
  Serial.print(x1);
  Serial.print("-");
  Serial.print(x2);
  Serial.print("-");
  Serial.print(x3);
  Serial.print("-");
  Serial.println(x4);

  analogWrite(motorRightForward, x1);
   analogWrite(motorLeftForward, x2);
   analogWrite(motorRightBack, x3);
   analogWrite(motorLeftBack, x4);    
  }

void Right(){
   analogWrite(motorRightForward, 0);
   analogWrite(motorLeftForward, 255);
   analogWrite(motorRightBack, 0);
   analogWrite(motorLeftBack, 0);
  }
  
void Left(){
   analogWrite(motorRightForward, 255);
   analogWrite(motorLeftForward, 0);
   analogWrite(motorRightBack, 0);
   analogWrite(motorLeftBack, 0);
  }

void Forward(){
   analogWrite(motorRightForward, 255);
   analogWrite(motorLeftForward, 255);
   analogWrite(motorRightBack, 0);
   analogWrite(motorLeftBack, 0);
  }
void Backward(){
   analogWrite(motorRightForward, 0);
   analogWrite(motorLeftForward, 0);
   analogWrite(motorRightBack, 255);
   analogWrite(motorLeftBack, 255);
  }

void Stop(){
   analogWrite(motorRightForward, 0);
   analogWrite(motorLeftForward, 0);
   analogWrite(motorRightBack, 0);
   analogWrite(motorLeftBack, 0);
  }


/***************** NEOPIXELS *********************/
void Neo(int x1, int x2, int x3, int x4){   //x1 is which neopixel, x2 is red, x3 is green, x4 is blue.
   pixels.setPixelColor(x1, pixels.Color(x3, x2, x4));
  }

void neoBack(){
   pixels.setPixelColor(0, pixels.Color(125, 125, 125));
   pixels.setPixelColor(1, pixels.Color(125, 125, 125));
   pixels.setPixelColor(2, pixels.Color(0, 125, 0));
   pixels.setPixelColor(3, pixels.Color(0, 125, 0));
   pixels.show(); 
  }

void neoForward(){
   pixels.setPixelColor(0, pixels.Color(0, 125, 0));
   pixels.setPixelColor(1, pixels.Color(0, 125, 0));
   pixels.setPixelColor(2, pixels.Color(125, 125, 125));
   pixels.setPixelColor(3, pixels.Color(125, 125, 125));
   pixels.show(); 
}

void neoRight(){
   pixels.setPixelColor(1, pixels.Color(75, 255, 0));
   pixels.setPixelColor(2, pixels.Color(75, 255, 0));
   pixels.show(); 
  }

void neoLeft(){
   pixels.setPixelColor(0, pixels.Color(75, 255, 0));
   pixels.setPixelColor(3, pixels.Color(75, 255, 0));
   pixels.show(); 
  }



void setup()
{

/****************************************************************************
 ***                          Gripper                                     ***
 ****************************************************************************/
  gripperOpen();

    gripper.attach(gripperPin);

/****************************************************************************
 ***                          Line Sensor                                 ***
 ****************************************************************************/
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
  Serial.begin(9600);
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

/****************************************************************************
 ***                          Neo Pixels                                  ***
 ****************************************************************************/
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)


/****************************************************************************
 ***                                Motor                                 ***
 ****************************************************************************/
   pinMode(motorLeftForward, OUTPUT);
   pinMode(motorLeftBack, OUTPUT);
   pinMode(motorRightBack, OUTPUT);
   pinMode(motorRightForward, OUTPUT);
}

void loop()
{



/****************************************************************************
 ***         Experimental check for if 4 sensors see black                ***
 ****************************************************************************/





  

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
//  delay(100);

//0 is the most right sensor
//7 is the most left sensor


if(sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600 && sensorValues[7] > 600 && gripperClosed == 0){
  Stop();
  //close gripper
  gripperClose();
  gripperClosed = 1;
 
  delay(200);
  Left();
  delay(300);
  Forward();
  delay(200);
}
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
  

//makes it stop at the end, but has the chance to drop the thing also at an intersection if all sensors see a line. 

// if (amount > 4 && amount < 8){
//  Forward();
//  delay(50);
//  }
//
//unsigned long currentMillis = millis();
//
//if(amount > 7 ){
//  if(currentMillis - previousMillis_1 >= interval_1){
//    Forward();
//    delay(100);
//    if (amount > 7){
//      Stop();
//      delay(500);
//      gripperOpen();
//      delay(500);
//      Backward();
//      delay(1000);
//      gripperClosed = 0;  
//      Stop();
//      delay(99999);
//    }    
//  }
//}


// maybe i shoud try a for loop

 if (amount > 4 && amount < 8){
  Forward();
  delay(50);
  }

unsigned long currentMillis = millis();

if(amount == 8 ){
  
    Motor(200, 200, 0, 0);
    delay(400);
    if (amount == 8){
      if(currentMillis - previousMillis_1 >= interval_1){
        Backward();
        delay(200);
        Stop();
        delay(500);
        gripperOpen();
        delay(500);
        Backward();
        delay(1000);
        gripperClosed = 0;  
        Stop();
        delay(99999);
    }
  }
}


//Line following

if (amount < 4 || amount > 7){

 if(sensorValues[0] < 600 && sensorValues[1] < 600 && sensorValues[2] < 600 && sensorValues[3] < 600 && sensorValues[4] < 600 && sensorValues[5] < 600 && sensorValues[6] < 600 && sensorValues[7] < 600){
  if(lastSensor < 4){
//      Right();
      Motor(0, 190, 190, 0);
      Serial.print("AAHHHH WHERE IS IT !!!!!!!!!!!!!!!! GOO RRIGGHHTHTHTHT!!!!!!");
  }
  else if (lastSensor > 3){
//     Left();
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
    Serial.println("go rightish");
    lastSensor = 2;
    Serial.println("Last sensor is " + String(lastSensor));
  }

  else if(sensorValues[3] > 600 && sensorValues[4] > 600){
//    Forward();
    Motor(170, 170, 0, 0);
    Serial.print("go forward");
  //  lastSensor = NULL;
    }
    
  else if(sensorValues[3] > 600 && sensorValues[4] < 600){
    Motor(170, 180, 0, 0);
    Serial.print("go forwardish");
    lastSensor = 3;
  }
  
  else if(sensorValues[4] > 600 && sensorValues[3] < 600){
    Motor(180, 170, 0, 0);
    Serial.print("go forwardish");
    lastSensor = 4;
    }
}
/****************************************************************************
 ***                      Sonic Sensor(distance)                          ***
 ****************************************************************************/
 long duration, inches, cm;
   pinMode(pingPin, OUTPUT);
   digitalWrite(pingPin, LOW);
//   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
//   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   inches = microsecondsToInches(duration);
   cm = microsecondsToCentimeters(duration);
//   Serial.print(inches);
//   Serial.print("in, ");
//   Serial.print(cm);
//   Serial.print("cm");

    Serial.println();
  //  delay(250);
  
   
/****************************************************************************
 ***                        Neo Pixels and Motors                         ***
 ****************************************************************************/
  pixels.clear(); // Set all pixel colors to 'off'
  
  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  // pixels.Color() takes GRB values, from 0,0,0 up to 255,255,255

  if(cm<=1){
     neoBack();
     Motor(0, 0, 210, 200);
     delay(1000);
     
     neoForward();
     Stop();
     delay(200);
   
     neoLeft();
     delay(200);

     neoForward();
     Motor(175, 0, 0, 150);
     delay(200); 

     neoLeft();
     delay(200);
 
     neoForward();
     delay(200);
 
     neoLeft();
     delay(200);
  
     neoForward();
     delay(200);
 
     neoLeft();
     delay(200);

     neoForward();
     delay(200);

     neoLeft();
     delay(200);

     neoForward();
     delay(200);
  
     neoLeft();
     delay(200);
     
     neoForward();
  }else{
    neoForward();
//    Motor(160, 150, 0, 0);
  }  
}

/****************************************************************************
 ***                      Sonic Sensor(distance                           ***
 ****************************************************************************/
 
long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

/****************************************************************************
 ***                      Gripper                                         ***
 ****************************************************************************/

void gripperOpen(){
//  for (servoPos = 5; servoPos <= 130; servoPos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    gripper.write(servoPos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
  gripper.write(100);
}

void gripperClose(){
//  for (servoPos = 130; servoPos >= 5; servoPos -= 1) { // goes from 180 degrees to 0 degrees
//    gripper.write(servoPos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  
//  }
  gripper.write(40);
}
