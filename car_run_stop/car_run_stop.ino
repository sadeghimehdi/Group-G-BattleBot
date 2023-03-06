/****************************************************************************
 ***                       Car move stop with distance                    ***
 ****************************************************************************/
const int trigPin = 4;
const int echoPin = 2;
#include <Servo.h>
int Left_motor_back = 9;   //Left motor back //9
int Left_motor_go = 6;   // Left motor go
int Right_motor_back = 11;  // Right motor back//11
int Right_motor_go = 10;  // Right motor go
int distance;
//int line_gray_3=2;
//int line_gray_4=7;
//int line_gray_5=8;
//int line_gray_6=12;
void setup()
{
  // Initialize the motor drive IO as the output mode
  pinMode(Left_motor_go, OUTPUT); // PIN 6 (PWM)
  pinMode(Left_motor_back, OUTPUT); // PIN 9 (PWM)
  pinMode(Right_motor_go, OUTPUT); // PIN 10 (PWM)
  pinMode(Right_motor_back, OUTPUT); // PIN 11 (PWM)
  Serial.begin(9600);
  //pinMode(2, OUTPUT);
  //digitalWrite(2,HIGH);
  Serial.println("test ok");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
//  pinMode(line_gray_3, INPUT);
//  pinMode(line_gray_4, INPUT);
//  pinMode(line_gray_5, INPUT);
//  pinMode(line_gray_6, INPUT);
}
void run(int time)     // forward
{
  analogWrite(Right_motor_go, 200); // The right motor moves forward, and the PWM ratio is 0~255 to adjust the speed
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Left_motor_go, 200); //The left motor moves forward, and the PWM ratio is 0~255 to adjust the speed
  digitalWrite(Left_motor_back, LOW);
  delay(time * 100);   // Execution time can be adjusted
}
void left(int time)     // 左转 ture left
{
  analogWrite(Right_motor_go, 200);
  analogWrite(Right_motor_back, 0);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 200);
  delay(time * 100);   //Execution time can be adjusted
}
void right(int time)     //  turn right
{
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 200);
  analogWrite(Left_motor_go, 200);
  analogWrite(Left_motor_back, 0);
  delay(time * 100);   // Execution time can be adjusted
}
void back(int time)     // backward
{
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 200);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 200);
  delay(time * 100);   // Execution time can be adjusted
}

void brake(int time)         // stop
{
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  delay(time * 100);// Execution time can be adjusted
}

void dist() // Test and obstacle distance
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  distance = pulseIn(echoPin, HIGH) / 58.3;

}

void loop()
{
  delay(200); // Start after 2s delay

  dist();

  Serial.println(distance);

//  if (distance > 50)
//  {
//    run(15);//1s
//    left(10);
//    if (distance < 80)
//    {
//      right(10);
//      brake(20);
//    }
//    else
//    {
//      run(15);
//    }
//  }
//  else
//  {
//    back(10);
//  }
  run(10);
  left(12);
  right(12);
  back(10);
}
