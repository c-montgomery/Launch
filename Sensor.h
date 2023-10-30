
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
const int UV1 = A1;
const int UV2 = A2;
const int UV3 = A3;
const int UV4 = A4;

// SD card
const int chipSelect = 53;


////////////////////////////////////////////////////////////
//////////////////////////////   Sensor Class
////////////////////////////////////////////////////////////
//Enclapsulates instances of various sensors to reduce SRAM usage

class Sensor {

public:

  Sensor()
    : uvSensor(Adafruit_LTR390()), rgbSensor(Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X)), bnoSensor(Adafruit_BNO055(55)), aht20Sensor1(AHT20()) {}

  Adafruit_LTR390 uvSensor;
  Adafruit_TCS34725 rgbSensor;
  Adafruit_BNO055 bnoSensor;
  AHT20 aht20Sensor1;
  TinyGPSPlus gps;
  File dataFile;
  //Analog1-4
  int UV1, UV2, UV3, UV4; 
  //bmp280

  unsigned long elapsed;


  void writeHeader() {
    boolean isWritten = false;
    while (!isWritten) {
      // Create the file on the SD card if it doesn't exist
      dataFile = SD.open("data.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println(" "" ,elapsed(ms),UTCTime,Altitude,lat,long,km/h,heading_in_degrees, "" ,UV_value1,UV_value2, UV_value3, UV_value4, Red_level,Green_level,Blue_level,temp_sensor1,humidity_sensor1");
        dataFile.flush();  // Save changes to the file
        isWritten = true;
      } else {
        Serial.println("Error opening data.csv for writing header");
        SD.open("data.csv", FILE_WRITE);
      }
    }
  }
  

  void logData() {
    
    while (Serial3.available() > 0) {
      gps.encode(Serial3.read());
      if (gps.location.isValid()>0){
        Serial.println("FIX");
      }
      
    }
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
      dataFile.print(",");
      // Log time from GPS
      dataFile.print(gps.time.value());
      dataFile.print(",");

      int convertedHr = gps.time.hour()+5;
      if (convertedHr > 24){
        convertedHr -= 24;
      }
      int convertedSecond = gps.time.second();
      if (convertedSecond <10){
        char array[2];
        convertedSecond = snprintf(array, 2, "%05d", convertedSecond);
      }

      String currentTime = String(convertedHr)+ ":" + String(gps.time.minute() + ":" + String(convertedSecond));
      Serial.println(gps.time.value());
      Serial.println("<Gpstime^^");
      Serial.print("Hour:  ");
        Serial.print(gps.time.hour()+17);
        Serial.print(gps.time.minute());
        Serial.print(gps.time.second());
      // Log GPS data
      dataFile.print(gps.altitude.meters());
      dataFile.print(",");
      dataFile.print(gps.location.lat(), 6);
      dataFile.print(",");
      dataFile.print(gps.location.lng(), 6);
      dataFile.print(",");
      dataFile.print(gps.speed.kmph());
      dataFile.print(",");
      dataFile.print(gps.course.deg());
      dataFile.print(",");
      
      //update UV values
      UV1 = analogRead(A0);
      UV2 = analogRead(A1);
      UV3 = analogRead(A2);
      UV4 = analogRead(A3);
      // Log UV sensor data
      dataFile.print(UV1);
      dataFile.print(",");
      dataFile.print(UV2);
      dataFile.print(",");
      dataFile.print(UV3);
      dataFile.print(",");
      dataFile.print(UV4);
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



      // Get the new temperature and humidity value
      dataFile.print(aht20Sensor1.getTemperature());
      dataFile.print(aht20Sensor1.getHumidity());


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
#endif  // SENSORDATA_H
