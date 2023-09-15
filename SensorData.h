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
//////////////////////////////   SensorData Class 
////////////////////////////////////////////////////////////  
//Enclapsulates instances of various sensors to reduce SRAM usage


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

class SensorData {

public:
  
  SensorData() : uvSensor(Adafruit_LTR390()), rgbSensor(Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X)), bnoSensor(Adafruit_BNO055(55)),aht20Sensor1(AHT20()) {}

  Adafruit_LTR390 uvSensor;
  Adafruit_TCS34725 rgbSensor;
  Adafruit_BNO055 bnoSensor;
  AHT20 aht20Sensor1;
  TinyGPSPlus gps;
  File dataFile;

  void logData() {
    // Open the SD card
    dataFile = SD.open("/Ascend 2023/Data_Log/data.csv", FILE_WRITE);

    // If there's a file, iterate through data points
    if (dataFile) {
      // Log time
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

      // NOT IN LOOP. MUST BE LOGGED
      
      // Get the new temperature and humidity value
      float temperature = aht20Sensor1.getTemperature();
      float humidity = aht20Sensor1.getHumidity();

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