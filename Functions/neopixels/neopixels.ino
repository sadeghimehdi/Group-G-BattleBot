#include <Adafruit_NeoPixel.h>


// Which pin on the Arduino is connected to the NeoPixels?
#define NEOPIN        10 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 4 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);



void Neo(int x1, int x2, int x3, int x4){   //x1 is which neopixel, x2 is red, x3 is green, x4 is blue.
   pixels.setPixelColor(x1, pixels.Color(x3, x2, x4));
  }

void neoBack(){
   pixels.setPixelColor(0, pixels.Color(125, 125, 125));
   pixels.setPixelColor(1, pixels.Color(125, 125, 125));
   pixels.setPixelColor(2, pixels.Color(0, 125, 0));
   pixels.setPixelColor(3, pixels.Color(0, 125, 0));
   pixels.show(); 
  }

void neoForward(){
   pixels.setPixelColor(0, pixels.Color(0, 125, 0));
   pixels.setPixelColor(1, pixels.Color(0, 125, 0));
   pixels.setPixelColor(2, pixels.Color(125, 125, 125));
   pixels.setPixelColor(3, pixels.Color(125, 125, 125));
   pixels.show(); 
}

void neoRight(){
   pixels.setPixelColor(1, pixels.Color(75, 255, 0));
   pixels.setPixelColor(2, pixels.Color(75, 255, 0));
   pixels.show(); 
  }

void neoLeft(){
   pixels.setPixelColor(0, pixels.Color(75, 255, 0));
   pixels.setPixelColor(3, pixels.Color(75, 255, 0));
   pixels.show(); 
  }

void setup()
{
 pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
 
}

void loop(){
  //Either use the premade functions or use Neo(x1,x2,x3,x4).  
  }
