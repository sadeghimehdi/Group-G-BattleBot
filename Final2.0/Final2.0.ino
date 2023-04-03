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
const uint8_t sensorCount = 8;
//valatile needs to be checked
volatile uint16_t sensorValues[sensorCount];


/****************************************************************************
 ***                             Millis                                   ***
 ****************************************************************************/
unsigned long previousMillis_1 = 0;
const long interval_1 = 5000; //interval for first event (5seconds)

unsigned long previousMillis_2 = 0;
const long interval_2 = 10000; //interval for second event, which checks whether the robot is moving or not (5seconds)


/****************************************************************************
 ***                             Millis                                   ***
 ****************************************************************************/
const byte pulsePinRight = 4;  // Pin conneccted to encoder, for the right wheel
const byte pulsePinLeft = 12; //Pin pulse for the left wheel 

unsigned long pulseCountRight;
unsigned long pulseCountLeft;
unsigned long midPoint;

// Timeout value looking for an encoder pulse
const unsigned maxPulseLength = 5000;


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
int theEnd = 0;


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

void neoBackward(){
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
  pixels.setPixelColor(0, pixels.Color(0, 125, 0));
  pixels.setPixelColor(1, pixels.Color(75, 255, 0));
  pixels.setPixelColor(2, pixels.Color(75, 255, 0));
  pixels.setPixelColor(3, pixels.Color(125, 125, 125));
  pixels.show(); 
}

void neoLeft(){
  pixels.setPixelColor(0, pixels.Color(75, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 125, 0));
  pixels.setPixelColor(2, pixels.Color(125, 125, 125));
  pixels.setPixelColor(3, pixels.Color(75, 255, 0));
  pixels.show(); 
}

void neoStop(){
  pixels.setPixelColor(0, pixels.Color(75, 255, 0));
  pixels.setPixelColor(1, pixels.Color(75, 255, 0));
  pixels.setPixelColor(2, pixels.Color(75, 255, 0));
  pixels.setPixelColor(3, pixels.Color(75, 255, 0));
  pixels.show(); 
}
  

void setup()
{
 pinMode(pulsePinRight, INPUT);

 
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
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);

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
  for (uint8_t i = 0; i < sensorCount; i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  } 
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < sensorCount; i++)  {
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
  for (uint8_t i = 0; i < sensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  if(sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600 && sensorValues[7] > 600 && gripperClosed == 0){
    Stop();
    gripperClose();
    gripperClosed = 1;
    delay(200);
    Forward();
    delay(200);
    Motor(250,0,0,250);
    delay(300);
  }

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
 ***                      Sonic Sensor(distance)                          ***
 ****************************************************************************/
  long duration, inches, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  digitalWrite(pingPin, HIGH);
  digitalWrite(pingPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  Serial.println();


/****************************************************************************
 ***                       Pulse and Stop & Drop                          ***
 ****************************************************************************/
  unsigned long currentMillisRight = millis();
  if(amount >= 6 && theEnd == 0){
    if(currentMillisRight - previousMillis_1 >= interval_1){
      pulseCountRight = 0;   
      int previousPulseState = digitalRead(pulsePinRight);
      unsigned long lastPulseTime = millis();
      Motor(200,200,0,0);
      neoForward();
      while (1){
        Serial.println(pulseCountRight);
        int pulseState = digitalRead(pulsePinRight);
        
        if (pulseState != previousPulseState){
          // State change
          previousPulseState = pulseState;
          pulseCountRight++;
          lastPulseTime = millis();
        }
        
        if (pulseCountRight == 7){
          //stop after a set amount of pulses
          Serial.println("STOP AHHHH");
          theEnd = 1;
          return;
        }
      }
    }
  }

  if(theEnd == 1 && amount >= 6){
    Backward();
    neoBackward();
    delay(400);
    Stop();
    neoStop();
    delay(500);
    gripperOpen();
    delay(500);
    Backward();
    neoBackward();
    delay(1000);
    //gripperClosed = 0;  
    Stop();
    delay(10000);
    Right();
    neoRight();
    delay(500);
    lastSensor = 2;
    //delay(99999);
  }
  else if (theEnd == 1 && amount < 6){ 
    theEnd = 0;
  }


/****************************************************************************
 ***                         Line folllowing                             ***
 ****************************************************************************/
  else if(sensorValues[0] < 600 && sensorValues[1] < 600 && sensorValues[2] < 600 && sensorValues[3] < 600 && sensorValues[4] < 600 && sensorValues[5] < 600 && sensorValues[6] < 600 && sensorValues[7] < 600){
    
    if(lastSensor < 4){
      //Right turn
      Motor(0, 190, 190, 0);
      Serial.print("AAHHHH WHERE IS IT !!!!!!!!!!!!!!!! GOO RRIGGHHTHTHTHT!!!!!!");
      neoRight();
    }
    else if (lastSensor > 3){
      //Left turn
      Motor(190, 0, 0, 190);
      Serial.print("AAHHHH WHERE IS IT !!!!!!!!!!!!!!!! GOO LEFFFTT!!!!");
      neoLeft();
    }
  }  
  
  else if (sensorValues[7] > 600 && sensorValues[6] < 600){
    Motor(200, 100, 0, 0);
    Serial.print("Go hard left");
    lastSensor = 7;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoLeft();
  }
  
  else if (sensorValues[7] > 600 && sensorValues[6] > 600){
    Motor(200, 110, 0, 0);
    Serial.print("Go hardish left");
    lastSensor = 7;
        // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoLeft();
  }
  
  else if(sensorValues[6] > 600 && sensorValues[5] < 600){
    Motor(200, 120, 0, 0);
    Serial.println("Go left");
    lastSensor = 6;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoLeft();
  }
  
  else if(sensorValues[6] > 600 && sensorValues[5] > 600){
    Motor(200, 130, 0, 0);
    Serial.println("Goish left");
    lastSensor = 6;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoLeft();
  } 
  
  else if(sensorValues[5] > 600 && sensorValues[4] < 600){
    Motor(200, 130, 0, 0);
    Serial.println("go leftish");
    lastSensor = 5;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoLeft();
  }

  else if(sensorValues[5] > 600 && sensorValues[4] > 600){
    Motor(200, 140, 0, 0);
    Serial.println("go leftish"); 
    lastSensor = 5;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoLeft();
  }

  else if (sensorValues[0] > 600 && sensorValues[1] < 600){
    Motor(100, 200, 0, 0);
    Serial.println("Go hard right");
    lastSensor = 0;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoRight();
  }

 else if(sensorValues[0] > 600 && sensorValues[1] > 600){
     Motor(110, 200, 0, 0);
     Serial.println("Go right");
     lastSensor = 1;
     // = 0;
     Serial.println("Last sensor is " + String(lastSensor));
     neoRight();
  }
    
  else if(sensorValues[1] > 600 && sensorValues[2] < 600){
    Motor(120, 200, 0, 0);
    Serial.println("Go right");
    lastSensor = 1;
    Serial.println("Last sensor is " + String(lastSensor));
    neoRight();
  }
 
  else if(sensorValues[1] > 600 && sensorValues[2] > 600){
    Motor(120, 200, 0, 0);
    Serial.println("go rightish");
    lastSensor = 2;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoRight();
  }

  else if(sensorValues[2] > 600 && sensorValues[3] < 600){
    Motor(130, 200, 0, 0);
    Serial.println("go rightish");
    lastSensor = 2;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoRight();
  }

  else if(sensorValues[2] > 600 && sensorValues[3] > 600){
    Motor(140, 200, 0, 0);
    Serial.println("go rightish");
    lastSensor = 2;
    // = 0;
    Serial.println("Last sensor is " + String(lastSensor));
    neoRight();
  }

  else if(sensorValues[3] > 600 && sensorValues[4] > 600){
    Motor(250, 250, 0, 0);
    Serial.print("go forward");
    neoForward();
  } 
    
  else if(sensorValues[3] > 600 && sensorValues[4] < 600){
    Motor(170, 180, 0, 0);
    Serial.print("go forwardish");
    lastSensor = 3;
    // = 0;
    neoForward();
  }
  
  else if(sensorValues[4] > 600 && sensorValues[3] < 600){
    Motor(180, 170, 0, 0);
    Serial.print("go forwardish");
    lastSensor = 4;
    // = 0;
    neoForward();
  }

  if(cm < 20){
    if(currentMillisRight - previousMillis_1 >= interval_1){
      Motor(0,250,250,0);
      neoRight();
      delay(350);
      Motor(255,170,0,0);
      neoLeft();
      delay(1750);
      lastSensor = 7;
// = 1;
    }
  }




//if(cm < 20){
//  if(currentMillisRight - previousMillis_1 >= interval_1){
//    int previousPulseState = digitalRead(pulsePinRight);
//    unsigned long lastPulseTime = millis();
//          pulseCountRight = 0;   
//
//   while (1){
////    Motor(0,250,250,0);
//Motor(250,0,0,250);
//      Serial.println(pulseCountRight);
//      int pulseState = digitalRead(pulsePinRight);
//      
//      if (pulseState != previousPulseState){
//        // State change
//        previousPulseState = pulseState;
//        pulseCountRight++;
//        lastPulseTime = millis();
//      }
//      
//      if (pulseCountRight == 20){
//        //stop after a set amount of pulses
//        Serial.println("STOP AHHHH");
//        lastSensor = 2;
//        return;
//      }
//    }
////           Motor(255,170,0,0);
//Motor(170,250,0,0);
////    Motor(0,250,250,0);
//     delay(1000);
//
//  }
//
//}

//
///****************************************************************************
// ***                        if not move, push                             ***
// ****************************************************************************/
//
//if(currentMillisRight - previousMillis_2 >= interval_1){
//    int previousPulseState = digitalRead(pulsePinRight);
//    unsigned long lastPulseTime = millis();
//     pulseCountRight = 0;   
//
//    while (1){
//      Serial.println(pulseCountRight);
//      int pulseState = digitalRead(pulsePinRight);
//      
//      if (pulseState != previousPulseState){
//        // State change
//        previousPulseState = pulseState;
//        pulseCountRight++;
//        lastPulseTime = millis();
//      }
//
//      if(millis() - lastPulseTime >= maxPulseLength){
//        Motor(250,250,0,0);
//        delay(100);  
//        previousMillis_2 = currentMillisRight;
//       
//      } else {
//        pulseCountRight = 0;
//        previousMillis_2 = currentMillisRight;
//        
//        }
//
//
//
//    
//    }
//  }


/****************************************************************************
 ***                        Neo Pixels and Motors                         ***
 ****************************************************************************/
  pixels.clear(); // Set all pixel colors to 'off'
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
  gripper.write(100);
}

void gripperClose(){
  gripper.write(40);
}
