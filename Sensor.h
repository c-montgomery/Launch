
#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LTR390.h>
#include <BMP280.h>
#include <TinyGPS++.h>
#include <AHT20.h>
#include <SD.h>
#include <SPI.h>


////////////////////////////////////////////////////////////
//////////////////////////////   Analog Pins
////////////////////////////////////////////////////////////



// SD card
const int chipSelect = 53;

//////////////////////////////   Sensor Class
////////////////////////////////////////////////////////////
//Enclapsulates instances of various sensors to reduce SRAM usage

class Sensor {

public:

  Sensor()
    : uvSensor(Adafruit_LTR390()), rgb(Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X)), aht20Sensor1(AHT20()), bmp280(BMP280()) {}

////////////////////////////////////////////////////////////
  Adafruit_LTR390 uvSensor;
  Adafruit_TCS34725 rgb;
  BMP280 bmp280;
  AHT20 aht20Sensor1;
  TinyGPSPlus gps;
  File dataFile;
  // rgb vars
  uint16_t r, g, b, c;

  //Analog1-4
  int UV1, UV2, UV3, UV4; 
  

  unsigned long elapsed;


  void writeHeader() {
    Serial.println("running writeHeader()");
    boolean isWritten = false;
    while (!isWritten) {
      // Create the file on the SD card if it doesn't exist
      dataFile = SD.open("data.csv", FILE_WRITE);
      Serial.println("ran partial1 writeHeader()");
      if (dataFile) {
        dataFile.println(" "" ,elapsed(ms),UTCTime,Altitude,lat,long,km/h,heading_in_degrees,UV_value1,UV_value2, UV_value3, UV_value4, Red_level,Green_level,Blue_level,temp_sensor1,humidity_sensor1, pressure_bmp280, temp_bmp280");
        dataFile.flush();  // Save changes to the file
        Serial.println("HEADER PRINTED");
        isWritten = true;
      } else {
        Serial.println("Error opening data.csv for writing header");
        SD.open("data.csv", FILE_WRITE);
      }
    }
    
  }
  

  void logData() {
    Serial.println("logDATA starts");
    while (Serial3.available() > 0) {
      gps.encode(Serial3.read());
      if (gps.location.isValid()>0){
        
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
      UV2 = analogRead(A2);
      UV3 = analogRead(A4);
      UV4 = analogRead(A6);
      Serial.println("UV1 - A0");
      Serial.println(UV1);
      Serial.println("UV2 - A2");
      Serial.println(UV2);
      Serial.println("UV2 - A4");
      Serial.println(UV3);
      Serial.println("UV4 - A6");
      Serial.println(UV4);
      
      // Log UV sensor data
      dataFile.print(UV1);
      dataFile.print(",");
      dataFile.print(UV2);
      dataFile.print(",");
      dataFile.print(UV3);
      dataFile.print(",");
      dataFile.print(UV4);
      dataFile.print(",");
     
      rgb.getRawData(&r, &g, &b, &c);
   
      dataFile.print(r);
      dataFile.print(",");
      dataFile.print(g);
      dataFile.print(",");
      dataFile.print(b);
      dataFile.print(",");
      //update temp/humidity
      float temp = aht20Sensor1.getTemperature();
      float humidity = aht20Sensor1.getHumidity();


      //update pressure/temp from bmp280
      uint32_t pressure = bmp280.getPressure();
      float bmp280_temperature = bmp280.getTemperature();
      dataFile.print(temp);
      dataFile.print(",");
      dataFile.print(humidity);
      dataFile.print(",");
      dataFile.print(pressure);
      dataFile.print(",");
      dataFile.print(bmp280_temperature);
      dataFile.print(",");



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
