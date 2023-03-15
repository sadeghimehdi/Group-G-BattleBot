const byte PulsePin = 4;  // Pin conneccted to encoder

unsigned long PulseCount;
unsigned long Midpoint;

// Timeout value looking for an encoder pulse
const unsigned MaxPulseLength = 1000;

void setup(){
  pinMode(PulsePin, INPUT);
  Serial.begin(9600);
}

void loop(){
  CountPulsesUntilTheyStop();
  PulseCount = 0;
}

void CountPulsesUntilTheyStop()
{
  int previousPulseState = digitalRead(PulsePin);
  unsigned long lastPulseTime = millis();

  while (1)
  {
    Serial.println(PulseCount);
    int pulseState = digitalRead(PulsePin);

    if (pulseState != previousPulseState)
    {
      // State change
      previousPulseState = pulseState;
      PulseCount++;
      lastPulseTime = millis();
    }


    if (PulseCount == 10)
    {
        //stop after a set amount of pulses
        Serial.println("STOP AHHHH");
        delay(1000);
        return;
    }

    
  }
}
