
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

int sensor_one = A5;
int sensor_two = A4;
int sensor_three = A3;
int sensor_four = A2;
int sensor_five = A1;
int sensor_six = A0;

/* Start time which is 0 */

unsigned long start_time = 0;

/* Motor Speed which is 255 */

int motor_speed = 255;

void setup() {

/* Left motor (back and forward) */

  pinMode(left_motor, OUTPUT);
  pinMode(left_motor_back, OUTPUT);

  /* Right motor (back and forward) */

  pinMode(right_motor, OUTPUT);
  pinMode(right_motor_back, OUTPUT);

  /* sensors */

  pinMode(sensor_one, INPUT);
  pinMode(sensor_two, INPUT);
  pinMode(sensor_three, INPUT);
  pinMode(sensor_four, INPUT);
  pinMode(sensor_five, INPUT);
  pinMode(sensor_six, INPUT);

}

void loop() {

/****************************************************************************
 ***                          Line Sensor analog                          ***
 ****************************************************************************/

/* The value of each sensor is stored here */

  int sensor_one_val = analogRead(sensor_one);
  int sensor_two_val = analogRead(sensor_two);
  int sensor_three_val = analogRead(sensor_three);
  int sensor_four_val = analogRead(sensor_four);
  int sensor_five_val = analogRead(sensor_five);
  int sensor_six_val = analogRead(sensor_six);

/* The two middle sensors which are sensor 3 and 4 are checked here. if both of them see the black line,
    left motor and right motor are turned on with specific speed*/

  if (sensor_three_val < 500 && sensor_four_val < 500) {

    digitalWrite(left_motor, HIGH);  // turn on the left motor
    analogWrite(left_motor, 160);  // set left motor speed

    digitalWrite(right_motor, HIGH);  // turn on the right motor
    analogWrite(right_motor, 160);  // set right motor speed

    /* The sensor one is checked here. if the sensor can see the black line,
    left motor and right motor are turned on and the robot will trun right*/

  }else if (sensor_one_val < 500) {

    digitalWrite(left_motor, HIGH);  // turn on the left motor
    analogWrite(left_motor, 110);  // set left motor speed

    digitalWrite(right_motor, HIGH);  // turn on the right motor
    analogWrite(right_motor, 240);  // set right motor speed

    /* The sensor two is checked here. if the sensor can see the black line,
    left motor and right motor are turned on and the robot will trun right*/

  }else if (sensor_two_val < 500) {

    digitalWrite(left_motor, HIGH);  // turn on the left motor
    analogWrite(left_motor, 110);  // set left motor speed

    digitalWrite(right_motor, HIGH);  // turn on the right motor
    analogWrite(right_motor, 240);  // set right motor speed

    /* The sensor five is checked here. if the sensor can see the black line,
    left motor and right motor are turned on and the robot will trun left*/

  }else if (sensor_five_val < 500) {

    digitalWrite(left_motor, HIGH);  // turn on the left motor
    analogWrite(left_motor, 240);  // set left motor speed

    digitalWrite(right_motor, HIGH);  // turn on the right motor
    analogWrite(right_motor, 110);  // set right motor speed

    /* The sensor six is checked here. if the sensor can see the black line,
    left motor and right motor are turned on and the robot will trun left*/

  }else if (sensor_six_val < 500){

    digitalWrite(left_motor, HIGH);  // turn on the left motor
    analogWrite(left_motor, 240);  // set left motor speed

    digitalWrite(right_motor, HIGH);  // turn on the right motor
    analogWrite(right_motor, 110);  // set right motor speed

    /* If sensor 6, 5, 4, 3 recognize the black line than means the line breakes to right
      therefor the left motor starts for specific time and the robot will turn hard right*/

  }else if ((sensor_six_val < 500) && (sensor_five_val < 500) && (sensor_four_val < 500) && (sensor_three_val < 500)){

    analogWrite(left_motor, 200);  // turn off the left motor
    analogWrite(right_motor, 50);  // turn off the right motor
    delay(200);

    /* If sensor 1, 2, 3, 4 recognize the black line than means the line breakes to right
      therefor the right motor starts for specific time and the robot will turn hard left*/
    
  }else if ((sensor_one_val < 500) && (sensor_two_val < 500) && (sensor_three_val < 500) && (sensor_four_val < 500)){

    analogWrite(left_motor, 50);  // turn off the left motor
    analogWrite(right_motor, 200);  // turn off the right motor
    delay(200);

  }else{

    // analogWrite(left_motor_back, 200);  // turn off the left motor
    // analogWrite(right_motor, 200);  // turn off the right motor
    // delay(200);

  }

}