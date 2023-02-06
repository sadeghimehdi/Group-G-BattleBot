const int trigPin = 9; //pin for sending the sound
const int echoPin = 3; //pin for receiving the sound
const int redLED = 13; //LED indicator

long duration; //time the sound takes to travel
int distance; //distance in cm, will be calculated from the duration

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT); //output, since arduino tells the trigger when to send the sound
  pinMode(echoPin, INPUT); // input, since the echo tells the arduino how long it was on for

  pinMode(redLED, OUTPUT);
  Serial.begin(9600); //to output the distance to the console

}

void loop() {
  // put your main code here, to run repeatedly:
  distance = getDistance(); //runs getDistance function

  if(distance < 20){
    digitalWrite(redLED, LOW);
  } else {
    digitalWrite(redLED, HIGH);
  }
  
  delay(100);

}

int getDistance(){
  digitalWrite(trigPin, LOW); //clear the trig pin
  delay(2);
  
  digitalWrite(trigPin, HIGH); //generate sound
  delay(10); //generate sound for 10ms
  digitalWrite(trigPin, LOW); //stop generating sound

  duration = pulseIn(echoPin, HIGH); //reads how long until the echo pin received the sound. echo pin is set to high until it gets the sound, and then its low
  distance = (duration * 0.034)/2; //distance calculation in CM, 

  Serial.print("Distance: ");
  Serial.println(distance);

  return distance;
}
