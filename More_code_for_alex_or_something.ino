
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
 ***                                Motor                                 ***
 ****************************************************************************/

const int motorLeftBack = 5; 
const int motorLeftForward = 6; 
const int motorRightBack = 9; 
const int motorRightForward = 3;
int lastSensor = NULL;
int theEnd = 0;





void setup()
{
 pinMode(PulsePinRight, INPUT);
/****************************************************************************
 ***                          Gripper                                     ***
 ****************************************************************************/
  gripperOpen();

    gripper.attach(gripperPin);

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

if(theEnd == 1 && amount == 8){
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
