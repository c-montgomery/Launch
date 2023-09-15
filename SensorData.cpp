#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LTR390.h>
#include <TinyGPS++.h>
#include <AHT20.h>
#include <SD.h>
#include <SPI.h>
////////////////////////////////////////////////////////////
//////////////////////////////   SensorData Class 
////////////////////////////////////////////////////////////  
//Enclapsulates instances of various sensors to reduce SRAM usage
///////////////////////////////////////////////////////////

class SensorData {

public:
  
  SensorData() : uvSensor(Adafruit_LTR390()), rgbSensor(Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X)), bnoSensor(Adafruit_BNO055(55)),ahtSensor1(AHT20()) {}

  Adafruit_LTR390 uvSensor;
  Adafruit_TCS34725 rgbSensor;
  Adafruit_BNO055 bnoSensor;
  AHT20 aht20Sensor1;
  TinyGPSPlus gps;
  File dataFile;
  
};