#include <QTRSensors.h>


QTRSensors qtr;
const uint8_t SensorCount = 6; //amount of sensors you wanna use, this is on 6 cus im using only 6 of the 8 atm.
uint16_t sensorValues[SensorCount];


void setup()
{
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

}


void loop()
{
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
      Motor(0, 255, 255, 0); //this is a function form the motor.ino file, so if you wanna use your own motor control, delete this. 
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

}
