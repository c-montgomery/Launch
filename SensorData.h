#include <Adafruit_BNO055.h>

#ifndef SENSORDATA_H
#define SENSORDATA_H

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
//////////////////////////////   Analog Pins
//////////////////////////////////////////////////////////// 

// GUVA-S12SD
const int guva_s12sd_pin = A1;

// IR sensor
const int ir_sensor_pin = A4;

// MICS-2714
const int mics_2714_pin = A0;

// MICS-4514
const int mics_4514_no2_pin = A2;
const int mics_4514_co_pin = A3;

// SD card
const int chipSelect = 53;

// Time variables
unsigned long prevPPSTime = 0;


////////////////////////////////////////////////////////////
//////////////////////////////   SensorData Class 
////////////////////////////////////////////////////////////  
//Enclapsulates instances of various sensors to reduce SRAM usage

class SensorData {

public:
  
  SensorData() : uvSensor(Adafruit_LTR390()), rgbSensor(Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X)), bnoSensor(Adafruit_BNO055(55)),aht20Sensor1(AHT20()) {}

  Adafruit_LTR390 uvSensor;
  Adafruit_TCS34725 rgbSensor;
  Adafruit_BNO055 bnoSensor;
  AHT20 aht20Sensor1;
  TinyGPSPlus gps;
  File dataFile;
  unsigned long elapsed;
 


  void writeHeader() {
    boolean isWritten = false;
    while(!isWritten){
      // Create the file on the SD card if it doesn't exist
      dataFile = SD.open("data.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println("HH:MM:SS,ms,UTCTime,Altitude,lat,long,Altitude,BNOonboardTemp,temperatureOutside,OrientX,OrientY,OrientZ,AccelX,AccelY,AccelZ,GravityX,GravityY,GravityZ,Magx,Magy,BMPAltitude,BMPTemp,BMPPressure,Red,Green,Blue,IR,UV1 DFrobot,UV2,AdafruitUV");
        dataFile.flush(); // Save changes to the file
        isWritten = true;
      } else {
        Serial.println("Error opening data.csv for writing header");
        SD.open("data.csv", FILE_WRITE);
      }
    }
    
    
 
}

  void logData() {
    // Open the SD card
    dataFile = SD.open("data.csv", FILE_WRITE);

    // If there's a file, iterate through data points
    if (dataFile) {
      //placeholder for UTC time converted to AZ (-7hrs)
      dataFile.print(" ");
      dataFile.print(",");

      //Log elapsed time in ms
      elapsed = millis();
      dataFile.print(elapsed);
      dataFile.print(",")
      // Log time from GPS
      dataFile.print(gps.time.value());
      dataFile.print(",");

      // Log GPS data
      dataFile.print(gps.location.lat(), 6);
      dataFile.print(",");
      dataFile.print(gps.location.lng(), 6);
      dataFile.print(",");
      dataFile.print(gps.altitude.meters());
      dataFile.print(",");
      dataFile.print(gps.speed.kmph());
      dataFile.print(",");
      dataFile.print(gps.course.deg());
      dataFile.print(",");
      dataFile.print(gps.satellites.value());
      dataFile.print(",");

      // Log UV sensor data
      float uv = uvSensor.readUVS();

      dataFile.print(uv);
      dataFile.print(",");

      // Log RGB color sensor data
      uint16_t r, g, b, c;
      rgbSensor.getRawData(&r, &g, &b, &c);
      dataFile.print(r);
      dataFile.print(",");
      dataFile.print(g);
      dataFile.print(",");
      dataFile.print(b);
      dataFile.print(",");

      // Log MICS-4514 data
      int mics_4514_no2_reading = analogRead(mics_4514_no2_pin);
      int mics_4514_co_reading = analogRead(mics_4514_co_pin);

      
      
      // Get the new temperature and humidity value
      dataFile.print(aht20Sensor1.getTemperature());
      dataFile.print(aht20Sensor1.getHumidity());

      dataFile.print(mics_4514_no2_reading);
      dataFile.print(",");
      dataFile.println(mics_4514_co_reading);

      // End the line
      dataFile.println();

      // Close the file
      dataFile.close();
      Serial.println("Data logged to data.csv");
    } else {
      dataFile.close();
      Serial.println("Error opening data.csv");
    }
  }
  
};
#endif // SENSORDATA_H
