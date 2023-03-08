//Pins
const int leftMotorFront = 11;
const int leftMotorBack = 10;
const int rightMotorFront = 9;
const int rightMotorBack = 6;
const int sensorTrigger = 2;
const int sensorRetreiver = 3;


//Global Variables
int speed = 255;
int minDistance = 15;
int distance;
long duration;


//Setup = executed once when motor first starts
void setup() {
 pinMode(leftMotorFront,OUTPUT); //defines motors as OUTPUT
 pinMode(rightMotorFront,OUTPUT);
 pinMode(sensorTrigger,OUTPUT); // Sets the trigger pin as an OUTPUT
 pinMode(sensorRetreiver, INPUT); // Sets the receiver pin as an INPUT
    
 Serial.begin(9600);
 // The text to be printed in serial monitor
 Serial.println(
  "Distance measurement using Arduino Uno.");
 delay(500);
}


//Loop = executed repeatedly through the program
void loop() {
  digitalWrite(sensorTrigger, LOW);
  delayMicroseconds(2); // wait for 2 ms to avoid collision in serial monitor
    
  digitalWrite(sensorTrigger,HIGH); // turn on the Trigger to generate pulse
  delayMicroseconds(10); // keep the trigger "ON" for 10 ms to generate pulse for 10 ms.
  digitalWrite(sensorTrigger,LOW); // Turn off the pulse trigger to stop pulse generation
 
  duration = pulseIn(sensorRetreiver, HIGH);
     
  distance = duration * 0.0344 / 2; // Expression to calculate distance using time
      
    Serial.print("Distance: ");
    Serial.print(distance); // Print the output in serial monitor
    Serial.println(" cm");
    delay(100);
  
  findDistance(distance, duration);
}


//Functions
void findDistance(int distance, long duration){
  if (distance > minDistance){ //the distance between the robot and the obstacle is enough
    while (distance > minDistance){
      digitalWrite(leftMotorFront, HIGH);
      digitalWrite(rightMotorFront, HIGH);
      delay(50);  //wait for 5 seconds

      digitalWrite(sensorTrigger, LOW);
      delayMicroseconds(2); // wait for 2 ms to avoid collision in serial monitor
    
      digitalWrite(sensorTrigger,HIGH); // turn on the Trigger to generate pulse
      delayMicroseconds(10); // keep the trigger "ON" for 10 ms to generate pulse for 10 ms.
      digitalWrite(sensorTrigger,LOW); // Turn off the pulse trigger to stop pulse generation
      duration = pulseIn(sensorRetreiver, HIGH);
         
      distance = duration * 0.0344 / 2; // Expression to calculate distance using time
          
      Serial.print("Distance: ");
      Serial.print(distance); // Print the output in serial monitor
      Serial.println(" cm");
      delay(100);
    }
  }
  else { //distance is not enough
    digitalWrite(leftMotorFront, LOW);
    digitalWrite(rightMotorFront, LOW);
  }
}
  
