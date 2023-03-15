
const int LED = 5;

char switchstate;

void setup()
  {
Serial.begin(9600);
pinMode(LED_BUILTIN, OUTPUT);

}
void loop() {//This code
while(Serial.available()>0){ 
 
switchstate = Serial.read();

Serial.print(switchstate);

Serial.print("\
");
delay(15);

if(switchstate
  == '1'){
 Serial.write("connected with bluetooth1");
 digitalWrite(LED_BUILTIN, HIGH);
  
}
else if(switchstate == '0'){//Else,
 Serial.write("connected with bluetooth0");
 digitalWrite(LED_BUILTIN, LOW);
}
}
}