const int motorLeftBack = 3; // Trigger Pin of the Left motor to go back
const int motorLeftForward = 9; // Trigger Pin of Right motor to go forward
const int motorRightBack = 5; // Trigger Pin of Right motor to go back
const int motorRightForward = 6; // Trigger Pin of Right motor to go forward



void Motor(int x1, int x2, int x3, int x4){
   analogWrite(motorRightForward, x1);
   analogWrite(motorLeftForward, x2);
   analogWrite(motorRightBack, x3);
   analogWrite(motorLeftBack, x4);    
  }

void Right(){
   analogWrite(motorRightForward, 0);
   analogWrite(motorLeftForward, 255);
   analogWrite(motorRightBack, 0);
   analogWrite(motorLeftBack, 0);
  }
  
void Left(){
   analogWrite(motorRightForward, 255);
   analogWrite(motorLeftForward, 0);
   analogWrite(motorRightBack, 0);
   analogWrite(motorLeftBack, 0);
  }

void Forward(){
   analogWrite(motorRightForward, 255);
   analogWrite(motorLeftForward, 255);
   analogWrite(motorRightBack, 0);
   analogWrite(motorLeftBack, 0);
  }
void Backward(){
   analogWrite(motorRightForward, 0);
   analogWrite(motorLeftForward, 0);
   analogWrite(motorRightBack, 255);
   analogWrite(motorLeftBack, 255);
  }

void Stop(){
   analogWrite(motorRightForward, 0);
   analogWrite(motorLeftForward, 0);
   analogWrite(motorRightBack, 0);
   analogWrite(motorLeftBack, 0);
  }



 
void setup()
{
   pinMode(motorLeftForward, OUTPUT);
   pinMode(motorLeftBack, OUTPUT);
   pinMode(motorRightBack, OUTPUT);
   pinMode(motorRightForward, OUTPUT);
}
void loop()
{
 //either call the specific functions or call Motor(x1, x2, x3, x4) with the speeds in it. x1 being the rightforward, x2 being rightback, x3 being left forward, x4 being leftback.
}
