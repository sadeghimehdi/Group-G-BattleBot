#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
#include <Servo.h>
Servo gripper; 
const int gripperPin = 2;
int gripperClosed = 0;
int duhh = 0;
int servoPos = 5;
/****************************************************************************
 ***                          Line Sensor                                 ***
 ****************************************************************************/

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


/****************************************************************************
 ***                             Millis                                   ***
 ****************************************************************************/
unsigned long previousMillis_1 = 0;

const long interval_1 = 5000; //interval for first event (5seconds)


/****************************************************************************
 ***                             Millis                                   ***
 ****************************************************************************/

const byte PulsePinRight = 4;  // Pin conneccted to encoder, for the right wheel
const byte PulsePinLeft = 12; //Pin pulse for the left wheel 

unsigned long PulseCountRight;
unsigned long PulseCountLeft;
unsigned long Midpoint;

// Timeout value looking for an encoder pulse
const unsigned MaxPulseLength = 1000;



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
 pinMode(PulsePinRight, INPUT);
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
    Serial.print('\t');
  }
  Serial.println(position);

//0 is the most right sensor
//7 is the most left sensor

if(sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600 && sensorValues[7] > 600 && gripperClosed == 0){
  Stop();
  gripperClose();
  gripperClosed = 1;
 
  delay(200);
  Left();
  delay(300);
  Forward();
  delay(200);
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
  

//makes it stop at the end, but has the chance to drop the thing also at an intersection if all sensors see a line. 

/****************************************************************************
 ***                       Pulse and Stop & Drop                          ***
 ****************************************************************************/

unsigned long currentMillisRight = millis();

if(amount >= 6 && theEnd == 0){
  if(currentMillisRight - previousMillis_1 >= interval_1){
    PulseCountRight = 0;   
    int previousPulseState = digitalRead(PulsePinRight);
    unsigned long lastPulseTime = millis();
    Motor(200,200,0,0);
    
    while (1)
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
      
      if (PulseCountRight == 7)
      {
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
    delay(400);
    Stop();
    delay(500);
    gripperOpen();
    delay(500);
    Backward();
    delay(1000);
    gripperClosed = 0;  
    Stop();
    delay(99999);
}else if (theEnd == 1 && amount < 6)
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
    Motor(250, 250, 0, 0);
    Serial.print("go forward");
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

  if(cm < 20){
    if(currentMillisRight - previousMillis_1 >= interval_1){
      Motor(0,250,250,0);
      delay(350);
      Motor(255,170,0,0);
      delay(2000);
    }
  }   
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
