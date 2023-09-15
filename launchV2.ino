#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LTR390.h>
#include <TinyGPS++.h>
#include <AHT20.h>
#include <SD.h>
#include <SPI.h>
#include "SensorData.h"



////////////////////////////////////////////////////////////
//////////////////////////////   Object declarations and constants
////////////////////////////////////////////////////////////  


#define PPS_PIN 19
volatile bool ppsFlag = false;

//SD card header count and header fields
const uint8_t ANALOG_COUNT = 43;
const char* dataPoints[ANALOG_COUNT] = {
  "ms", "UTCTime", "Altitude", "lat", "long", 
  "Altitude", "BNOonboardTemp", "temperatureOutside", "OrientX", "OrientY", "OrientZ", 
  "AccelX", "AccelY", "AccelZ", "GravityX", "GravityY", "GravityZ", "Magx", "Magy", 
  "BMPAltitude", "BMPTemp", "BMPPressure", "BMETemp", "BMEPressure", "BMEHumidity", 
  "BMEGas", "Red", "Green", "Blue", "IR", "UV", "Pressure", "02"
};


// Define a SensorData instance to encapsulate sensor objects
// Why? Global vars are stored on SRAM in arduino. SRAM is limited. 
//One SensorData object vs X separate sensor instances means less wasted SRAM
SensorData sensorData;



//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////FUNCTIONS
/////////////////////////////////////////////////////////////////////////////

void PPS_ISR() {
  ppsFlag = true;
}

void writeHeader() {
  // Your header writing code here
}

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////SETUP FUNCTIONS             
//////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(53, INPUT);
  digitalWrite(53, HIGH);
  Serial.begin(115200);
  Wire.begin();
  pinMode(chipSelect, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    writeHeader();
    return;
  }
  Serial.println("Card initialized.");

  if (!sensorData.uvSensor.begin(&Wire)) {
    Serial.println("Failed to find LTR-390 sensor!");
  }

  if (!sensorData.rgbSensor.begin()) {
    Serial.println("Failed to find TCS34725 sensor!");
  }

  if (!sensorData.bnoSensor.begin()) {
    Serial.println("Failed to find BNO055 sensor!");
  }
  if (!sensorData.aht20Sensor1.begin()){
    Serial.println("Failed to find Temp sensor 1");
  }

  attachInterrupt(digitalPinToInterrupt(PPS_PIN), PPS_ISR, RISING);
}
//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////LOOPS GON' LOOP             
//////////////////////////////////////////////////////////////////////////////
void loop() {
  unsigned long currentMillis = millis();
  // Your loop code here
  sensorData.logData();
}





