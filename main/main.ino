#include <bme68x.h>
#include <Adafruit_BME680.h>
#include <bme68x_defs.h>

#include <SD.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include <Wire.h>

////------------------------------------------TO-DO-------------------------------
//GPS  XXXXXXXXfunctional!
//MPU6050 ACCEL
//SDCARD
//9DOF
//TEMP
//SOLAR PANELS
//BMP280 --Not functional?
//BME680 XXXXXXfunctional!

#define Analog7 A7

//Create sensor objects
Adafruit_BME680 bme;
#define SEALEVELPRESSURE_HPA (1013.25)


void setup() {
  Serial.begin(115200);
  Serial.println("test");
  while (!Serial) {
    //Start Serial and SoftwareSerial
    Serial.begin(115200);
    Serial.println("Setting up ...");
    delay(2000);
    Serial.println("Serial started ");

  }
}
void loop() {

  Serial.println(Serial.available());
  if (Serial.available()) {
    Serial.println("GPS ");
    //mySerial.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  }
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);

  //Solar panel pin reading
  // Serial.println(AnalogRead(Analog7));

}
