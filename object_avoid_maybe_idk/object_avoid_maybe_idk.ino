#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>

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
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

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

const int motorLeftBack = 3; // Trigger Pin of Ultrasonic Sensor
const int motorLeftForward = 9; // Trigger Pin of Ultrasonic Sensor
const int motorRightBack = 5; // Trigger Pin of Ultrasonic Sensor
const int motorRightForward = 6; // Trigger Pin of Ultrasonic Sensor
int lastSensor = NULL;

int avoidObstacle = 0;

/****************************************************************************
 ***                                Functions                             ***
 ****************************************************************************/

/************************** MOTORS *****************************/
void Motor(int x1, int x2, int x3, int x4){
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
 ***                          Line Sensor                                 ***
 ****************************************************************************/
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++){
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
//5 is the most left sensor



//Line following

  if(sensorValues[0] < 600 && sensorValues[1] < 600 && sensorValues[2] < 600 && sensorValues[3] < 600 && sensorValues[4] < 600 && sensorValues[5] < 600){
    if(lastSensor == 1){
  //      Right();
        Motor(0, 255, 255, 0);
        Serial.print("AAHHHH WHERE IS IT !!!!!!!!!!!!!!!! GOO RRIGGHHTHTHTHT!!!!!!");
    }
    else if (lastSensor == 4){
  //     Left();
       Motor(255, 0, 0, 255);
       Serial.print("AAHHHH WHERE IS IT !!!!!!!!!!!!!!!! GOO LEFFFTT!!!!");
    }
  }
  else if (sensorValues[0] > 600){
      Motor(100, 255, 0, 0);
  //    Right();
      Serial.print("Go hard right");
      lastSensor = 1;
    }
  else if(sensorValues[1] > 600){
     Motor(150, 255, 0, 0);
     Serial.print("Go right");
     lastSensor = 1;
  }
  else if(sensorValues[4] > 600){
    Motor(255, 150, 0, 0);
    Serial.print("Go left");
     lastSensor = 4;
  }
  else if(sensorValues[5] > 600){
  //  Left();
     Motor(255, 100, 0, 0);
     Serial.print("Go hard left");
     lastSensor = 4;
    }
  else if(sensorValues[2] > 600 && sensorValues[3] > 600){
    Forward();
    Serial.print("go forward");
    }
  else if(sensorValues[2] > 600){
    Motor(255, 220, 0, 0);
    Serial.print("go forwardish");
    }
  else if(sensorValues[3] > 600){
    Motor(220, 255, 0, 0);
    Serial.print("go forwardish");
    }
  
/****************************************************************************
 ***                      Sonic Sensor(distance)                          ***
 ****************************************************************************/
 long duration, inches, cm;
   pinMode(pingPin, OUTPUT);
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
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

//  if(cm<1){
//     neoBack();
//     Motor(0, 0, 210, 200);
//     delay(1000);
//     
//     neoForward();
//     Stop();
//     delay(200);
//   
//     neoLeft();
//     delay(200);
//
//     neoForward();
//     Motor(175, 0, 0, 150);
//     delay(200); 
//
//     neoLeft();
//     delay(200);
// 
//     neoForward();
//     delay(200);
// 
//     neoLeft();
//     delay(200);
//  
//     neoForward();
//     delay(200);
// 
//     neoLeft();
//     delay(200);
//
//     neoForward();
//     delay(200);
//
//     neoLeft();
//     delay(200);
//
//     neoForward();
//     delay(200);
//  
//     neoLeft();
//     delay(200);
//     
//     neoForward();
//  }else{
//    neoForward();
//    Motor(160, 150, 0, 0);
//  }  

  if(cm<=20){
      avoidObstacle = 1;
    }

  if (avoidObstacle == 1){
      Left();
      delay(300);

     Forward();
     delay(300);

     Right();
     delay(300);
     

     Forward();
     delay(300);

      Right();
      delay(300);

      Forward();
      delay(300);
      avoidObstacle = 0;
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
