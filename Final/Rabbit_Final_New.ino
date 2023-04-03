/****************************************************************************
 ***                          The Rabbit Final Code                       ***
 ***                          31-03-2023 - V1.0                           ***
 ****************************************************************************/
 
/****************************************************************************
 ***                          Ultrasonic Pin                               ***
 ****************************************************************************/

const int pingPin = 11;
const int echoPin = 10;

long duration, cm;
bool catched = true;

/****************************************************************************
 ***                          Gripper Pin                                 ***
 ****************************************************************************/

 const int gripperPin = 13;

/****************************************************************************
 ***                          Motor Pins                                  ***
 ****************************************************************************/

int left_motor = 9;
int left_motor_back = 6;
int right_motor = 3;
int right_motor_back = 5;


/****************************************************************************
 ***                          Line Sensor Pins                            ***
 ****************************************************************************/

int sensor_one = A7;
int sensor_two = A6;
int sensor_three = A5;
int sensor_four = A4;
int sensor_five = A3;
int sensor_six = A2;
int sensor_seven = A1;
int sensor_eight = A0;

const int sensetive = 600;

/****************************************************************************
 ***                          Set Time                                    ***
 ****************************************************************************/

unsigned long time = 0;

/****************************************************************************
 ***                          Setup                                       ***
 ****************************************************************************/

void setup() {

  Serial.begin(9600);

/****************************************************************************
  ***                          INPUT/OUTPUT                            ***
  ****************************************************************************/

  /* Distence senser */
  
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);

  /* Left motor (back and forward) */

    pinMode(left_motor, OUTPUT);
    pinMode(left_motor_back, OUTPUT);

  /* Right motor (back and forward) */

    pinMode(right_motor, OUTPUT);
    pinMode(right_motor_back, OUTPUT);

  /* Line sensors */

    pinMode(sensor_one, INPUT);
    pinMode(sensor_two, INPUT);
    pinMode(sensor_three, INPUT);
    pinMode(sensor_four, INPUT);
    pinMode(sensor_five, INPUT);
    pinMode(sensor_six, INPUT);
    pinMode(sensor_seven, INPUT);
    pinMode(sensor_eight, INPUT);

   /* Gripper */

    pinMode(gripperPin, OUTPUT);

    openGripper();

}

/****************************************************************************
 ***                          Time Functions                              ***
 ****************************************************************************/

void wait(int timeToWait_1){

  time = millis();
  
  while(millis() < time + timeToWait_1){

    Stop(); 

  }

}

void littleForward(int timeToWait_2){

  time = millis();
  
  while(millis() < time + timeToWait_2){

    Forward(); 

  }

}

void rotateHardLeft(int timeToWait_2){

  time = millis();
  
  while(millis() < time + timeToWait_2){

    Motor(255, 0, 0, 255); 

  }

}

void rotateHardRight(int timeToWait_2){

  time = millis();

  while(millis() < time + timeToWait_2){

    Motor(0, 255, 255, 0); 

  }

}

void littleBack(int timeToWait_2){

  time = millis();

  while(millis() < time + timeToWait_2){

    Motor(0, 0, 255, 255); 

  }

}


/****************************************************************************
 ***                          Loop                                        ***
 ****************************************************************************/

void loop(){

  if(catched == true){

    digitalWrite(pingPin, LOW);
    digitalWrite(pingPin, HIGH);
    digitalWrite(pingPin, LOW); 
    duration = pulseIn(echoPin, HIGH);
    cm = microsecondsToCentimeters(duration);

    if(cm < 30){

      wait(3000);
      littleForward(950);
      wait(100);
      closeGripper();
      wait(100);
      rotateHardLeft(350);
      wait(100);
      littleForward(250);
      wait(100);

      catched = false;

    }

  }else if(catched == false){

    /* If all of the sensors detect, then go a little forward. */

    if(fsensorOne() && fsensorTwo() && fsensorThree() && fsensorFour() && fsensorFive() && fsensorSix() && fsensorSeven() && fsensorEight()){

      wait(100);
      littleForward(350);
      wait(100);

      /* then check if the sensors detect again,open the gripper otherwise turn right */

      if(fsensorOne() && fsensorTwo() && fsensorThree() && fsensorFour() && fsensorFive() && fsensorSix() && fsensorSeven() && fsensorEight()){
        
        wait(100);
        littleBack(350);
        wait(100);
        openGripper();
        wait(100);
        littleBack(1000);
        rotateHardRight(840);
        wait(200);
        littleForward(1000);

      }else{

        rotateHardRight(450);
        wait(100);

      }

    }

    /* if left sensors detect, go a little straight and check the sensors again */
    
    if(!fsensorSix() && !fsensorSeven() && !fsensorEight() && fsensorOne() && fsensorTwo() && fsensorThree() && fsensorFour() && fsensorFive()){
    
      wait(100);

      if(fsensorOne() && fsensorTwo() && fsensorThree() && fsensorFour() && fsensorFive() && fsensorSix() && fsensorSeven() && fsensorEight()){

        wait(100);
        littleForward(350);
        wait(100);
        rotateHardRight(450);
        wait(100);

      }else{

        wait(100);
        littleForward(350);
        wait(100);

        /* if one of the sensors detect the line, then wait otherwise, turn left */

        if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

          wait(100);
          rotateHardLeft(450);
          wait(100);

        }else{

          wait(100);

        }

      }

    }

    /* If Right sensors detect, then go right */
    
    if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && fsensorFour() && fsensorFive() && fsensorSix() && fsensorSeven() && fsensorEight()){
      
      wait(100);
      littleForward(350);
      wait(100);
      rotateHardRight(450);
      wait(100);

    }

    /* If non of the sensors detect, then turn 180 */

    if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

      littleForward(350);
      wait(100);
      rotateHardRight(880);
      wait(200);

    }

    if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && fsensorFour() && fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

      Motor(190, 190, 0, 0);

    }else{
    
      if(!fsensorOne() && !fsensorTwo() && fsensorThree() && fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();

      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();

      }else if(!fsensorOne() && !fsensorTwo() && fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();

      }else if(!fsensorOne() && fsensorTwo() && fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();

      }else if(!fsensorOne() && !fsensorTwo() && fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();

      }else if(!fsensorOne() && fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();

      }else if(fsensorOne() && fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();

      }else if(!fsensorOne() && fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();
      
      }else if(fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Left();
      
      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && fsensorFive() && fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Right();

      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && fsensorFive() && !fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Right();

      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Right();
      
      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && fsensorSix() && fsensorSeven() && !fsensorEight()){

        Right();

      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && fsensorSeven() && !fsensorEight()){

        Right();

      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && fsensorSix() && !fsensorSeven() && !fsensorEight()){

        Right();

      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && fsensorSeven() && fsensorEight()){

        Right();

      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && fsensorSeven() && !fsensorEight()){

        Right();

      }else if(!fsensorOne() && !fsensorTwo() && !fsensorThree() && !fsensorFour() && !fsensorFive() && !fsensorSix() && !fsensorSeven() && fsensorEight()){

        Right();

      }

    }

  }

}

/****************************************************************************
 ***                          Functions                                   ***
 ****************************************************************************/

/****************************************************************************
 ***                          Ultrasonic Function                         ***
 ****************************************************************************/

  long microsecondsToCentimeters(long microseconds){

   return microseconds / 29 / 2;

  }


/****************************************************************************
 ***                          Gripper Function                            ***
 ****************************************************************************/


  void openGripper(){

    for(int i = 0; i<10; i++){

      digitalWrite(gripperPin, HIGH);
      delayMicroseconds(1700); /* pulse duration in microseconds */
      digitalWrite(gripperPin, LOW);
      delay(20);

    }

  }

  void closeGripper(){

    for(int i = 0; i<10; i++){

      digitalWrite(gripperPin, HIGH);
      delayMicroseconds(1100); /* pulse duration in microseconds */
      digitalWrite(gripperPin, LOW);
      delay(20);

    }

  }


/****************************************************************************
 ***                          Sensor Function                             ***
 ****************************************************************************/

  bool fsensorOne(){

    if(analogRead(sensor_one) > sensetive){

      return true;

    }else{

      return false;

    }
    
  }

  bool fsensorTwo(){

    if(analogRead(sensor_two) > sensetive){
      
      return true;

    }else{

      return false;

    }

  }

  bool fsensorThree(){

    if(analogRead(sensor_three) > sensetive){
      
      return true;

    }else{

      return false;

    }

  } 

  bool fsensorFour(){

    if(analogRead(sensor_four) > sensetive){
      
      return true;

    }else{

      return false;

    }

  } 

  bool fsensorFive(){

    if(analogRead(sensor_five) > sensetive){
      
      return true;

    }else{

      return false;

    }

  } 

  bool fsensorSix(){

    if(analogRead(sensor_six) > sensetive){
      
      return true;

    }else{

      return false;

    }

  } 

  bool fsensorSeven(){

    if(analogRead(sensor_seven) > sensetive){
      
      return true;

    }else{

      return false;

    }

  } 

  bool fsensorEight(){

    if(analogRead(sensor_eight) > sensetive){
      
      return true;

    }else{

      return false;

    }
    
  } 

/****************************************************************************
 ***                          Motor Function                              ***
 ****************************************************************************/

  void Motor(int x1, int x2, int x3, int x4){

    analogWrite(right_motor, x1);
    analogWrite(left_motor, x2);
    analogWrite(right_motor_back, x3);
    analogWrite(left_motor_back, x4); 

  }

  void Right(){

    analogWrite(right_motor, 150);
    analogWrite(left_motor, 255);
    analogWrite(right_motor_back, 0);
    analogWrite(left_motor_back, 0);

  }
    
  void Left(){

    analogWrite(right_motor, 255);
    analogWrite(left_motor, 150);
    analogWrite(right_motor_back, 0);
    analogWrite(left_motor_back, 0);

  }

  void Forward(){

    analogWrite(right_motor, 255);
    analogWrite(left_motor, 255);
    analogWrite(right_motor_back, 0);
    analogWrite(left_motor_back, 0);

  }

  void Backward(){
    
    analogWrite(right_motor, 0);
    analogWrite(left_motor, 0);
    analogWrite(right_motor_back, 255);
    analogWrite(left_motor_back, 255);

  }

  void Stop(){
    
    analogWrite(right_motor, 0);
    analogWrite(left_motor, 0);
    analogWrite(right_motor_back, 0);
    analogWrite(left_motor_back, 0);

  }
