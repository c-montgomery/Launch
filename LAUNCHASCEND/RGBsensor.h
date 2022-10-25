#include <Wire.h>
#include "Adafruit_TCS34725.h"

// set to false if using a common cathode LED
#define commonAnode true

// our RGB -> eye-recognized gamma color
byte gammatable[256];


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//for (int i=0; i<256; i++) {
//    float x = i;
//    x /= 255;
//    x = pow(x, 2.5);
//    x *= 255;
//
//    if (commonAnode) {
//      gammatable[i] = 255 - x;
//    } else {
//      gammatable[i] = x;
//    }
//    //Serial.println(gammatable[i]);
//  }

void measureColor(){
  float red, green, blue;
tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
}
