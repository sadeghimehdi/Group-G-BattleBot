void setup() {
  pinMode(2, OUTPUT);//define arduino pin
  pinMode(4, INPUT);//define arduino pin
  Serial.begin(9600);//enable serial monitor

}
void loop() {
  //pulse output
  digitalWrite(2, LOW);
  delayMicroseconds(4);
  digitalWrite(2, HIGH);
  delayMicroseconds(10);
  digitalWrite(2, LOW);
  
  long t = pulseIn(4, HIGH);//input pulse and save it veriable

  long cm = t / 29 / 2; //time convert distance
  if(cm < 23){
    for(int i = 5; i<=0; i--){
      //do the main code
    }
  }
}