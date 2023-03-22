const int gripperPin = 12;
const int rotatorPin = 13;

int servoPos = 0;
int rotatorPos = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(gripperPin, OUTPUT);
  pinMode(rotatorPin, OUTPUT);
}

void loop() {
    sensorLeft();
    delay(1000);
    sensorCenter();
    gripperOpen();
    delay(1000);
    sensorRight();
    delay(1000);
    sensorCenter();
    gripperClose();
    delay(1000);
}

void gripperOpen(){
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(1600); // Duration of the pulse in microseconds
    digitalWrite(gripperPin, LOW);
    // Pulses duration: 1600 = open
}

void gripperClose(){
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(1010); // Duration of the pulse in microseconds
    digitalWrite(gripperPin, LOW);
    // Pulses duration: 1000 = fully closed
}

void sensorRight(){
    digitalWrite(rotatorPin, HIGH);
    delayMicroseconds(350); // Duration of the pusle in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 350ms = 0 degrees
}

void sensorLeft(){
    digitalWrite(rotatorPin, HIGH);
    delayMicroseconds(2310); // Duration of the pusle in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 2310ms - 180deg
}

void sensorCenter(){
    digitalWrite(rotatorPin, HIGH);
    delayMicroseconds(1310); // Duration of the pusle in microseconds
    digitalWrite(rotatorPin, LOW);
    //Pulse duration: 1310ms = 90 degrees
}
