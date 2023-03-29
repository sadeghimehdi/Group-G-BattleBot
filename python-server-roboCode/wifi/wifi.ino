// #include <Adafruit_ESP8266.h>

#include <SoftwareSerial.h>


SoftwareSerial esp8266(9, 4);
// Adafruit_ESP8266 wifi(&esp8266, &Serial, 4);

long time = millis();
String stringOne = "";

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    esp8266.begin(9600);

    Serial.begin(9600);
    //esp8266.write("AT+CIPSTART=\"TCP\",\"192.168.0.52\",8041\r\n");
    delay(5000);

    esp8266.write("AT+CIPSEND=5\r\n");
    delay(200);
    esp8266.write("robo2\r");
    digitalWrite(LED_BUILTIN, LOW);

    //esp8266.write("hello\r\n");
    //delay(200);

    //esp8266.write("AT+CIPSEND=5\r\n");
    //delay(200);
    // while (!Serial)
    //     ;

    // Serial.println("Started");

    // Serial.print("resetting...");
    //esp8266.write("AT+RST\r\n");
    //delay(5000);

    // Serial.print("reset!");

    // esp8266.write("AT+CWMODE_DEF=1\r\n");
    // delay(5000);

    // esp8266.write("AT+CWJAP_DEF=\"Battle_Bot_AP\",\"\"\r\n");
    // delay(5000);

    //esp8266.write("AT+CIPSTART=\"TCP\",\"10.232.8.40\",8090\r\n");
    //delay(5000);

    //esp8266.write("AT+CIPSEND=5\r\n");
    //delay(200);

    //esp8266.write("hello\r\n");
    //delay(200);
}

void loop()
{
    if (esp8266.available())
    {
      stringOne = esp8266.read();
      if (stringOne == "star3"){
        Serial.write("hell yeah");
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
        esp8266.write("robo3\r");
      }
      else if(esp8266.read() == "star3"){
        Serial.write("hell");
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }

    if (Serial.available()) 
    {
      Serial.write("he send you" + esp8266.read());
      Serial.write(esp8266.read());
      esp8266.write(Serial.read());

    }


    // if (millis() > 5000 + time)
    // {
    //     esp8266.write("AT+CIPSEND=5\r\n");
    //     delay(200);

    //     esp8266.write("hello\r\n");
    //     delay(200);

    //     time = millis();
    // }
}