#include <MemoryFree.h>;
#include <pgmStrToRAM.h>;
////////////////////////////////
#include <Adafruit_VEML6070.h>

#include <AltSoftSerial.h>

#include <TinyGPS++.h>
#include <TinyGPSPlus.h>

//#include <RingBuf.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>
#include <DFRobot_OxygenSensor.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_BNO055.h>
//#include <BMP280_DEV.h>  // Include the BMP280_DEV.h library
#include "Adafruit_BME680.h"
#include <Adafruit_BMP280.h>




////------------------------------------------TO-DO-------------------------------
//GPS SDCARD 9DOF TEMP
//
///////////////////////////////////////////////////constants//////////////////////////////////////////
#define RXpin 17
#define TXpin 16
#define FILE_BASE_NAME "Data"        //sd
#define Oxygen_IICAddress ADDRESS_3  //02
#define COLLECT_NUMBER 10            // collect number, the collection range is 1-100. //02
const int chipSelect = 53;
#define SEALEVELPRESSURE_HPA (1013.25)



#define ms 0
#define UTCTime 1
#define Altitude 2
#define Lat 3
#define Long 4
#define Altitude 5
#define BNOonboardTemp 6
#define temperatureOutside 7
#define OrientX 8
#define OrientY 9
#define OrientZ 10
#define AccelX 11
#define AccelY 12
#define AccelZ 13
#define GravityX 14
#define GravityY 15
#define GravityZ 16
#define Magx 17
#define Eighteen 18
#define BMPAltitude 19
#define BMPTemp 20
#define BMPPressure 21
#define BMETemp 22
#define BMEPressure 23
#define BMEHumidity 24
#define BMEGas 25
#define Red 26
#define Green 27
#define Blue 28
#define IR 29
#define UV 30
#define barPressure 31
#define O2 32

//SD card header count and header fields
const uint8_t ANALOG_COUNT = 33;
String dataPoints[ANALOG_COUNT] = { "ms", "UTCTime", "Altitude", "lat", "long", "Altitude", "BNOonboardTemp", "temperatureOutside", "OrientX", "OrientY", "OrientZ", "AccelX", "AccelY", "AccelZ", "GravityX", "GravityY", "GravityZ", "Magx", "Magy", "BMPAltitude", "BMPTemp", "BMPPressure", "BMETemp", "BMEPressure", "BMEHumidity", "BMEGas", "Red", "Green", "Blue", "IR", "UV", "Pressure", "02" };
//Sd storage array
String data[ANALOG_COUNT];

#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER 10  // collect number, the collection range is 1-100.

#define GPSSerial Serial1

//SD card
const uint32_t SAMPLE_INTERVAL_MS = 1000;

/* Set the delay between fresh samples BNO055 */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1000;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////Objects///////////////////////////////////////////////////
////Create instance Software Serial
SoftwareSerial mySerial(RXpin, TXpin);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  //Instantiate BNO-055

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);  //RGB

DFRobot_OxygenSensor oxygen;

Adafruit_VEML6070 uv = Adafruit_VEML6070();  //UV SENSOR

float temperature, pressure, altitude;  // Create the temperature, pressure and altitude variables
Adafruit_BMP280 bmp; // I2C

Adafruit_BME680 bme; // I2C

////------------------------------------------------------------------------------
//////SD Card
#define FILE_BASE_NAME "Data"
//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
//-------------------------------------------

// File system object.
SdFat sd;

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime;

//GPS obj
TinyGPSPlus gps;

AltSoftSerial altSerial;

long msElapsed = millis();

boolean isStarted = false;

void setup() {
  //SD card base file name
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";
  //  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  altSerial.begin(115200);
  uv.begin(VEML6070_HALF_T);
  delay(1000);

  //sd HALT ERROR
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1)
      ;
  }
  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
    error("file.open");
  }
  //INITIALIZE BNO 055 ACCELEROMETER
  //if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // while (1);
  }
  //oversampling and filter init
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms
  oxygen.begin(Oxygen_IICAddress);

  bmp.begin();
   /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  writeHeader();
  int msElapsed = millis();
}




void loop() {
  Serial.println("begin");

  // Time for next record.
  logTime += 1000UL * SAMPLE_INTERVAL_MS;
  // Wait for log time.
  int32_t diff;
  do {
    diff = micros() - logTime;
  } while (diff < 0);
  unsigned long endTime = bme.beginReading();
  // // Check for data rate too high.
  // if (diff > 10) {
  //   error("Missed data record");
  // }

  // Serial.println("Start ");
  // Serial.println(millis());
  // Serial.println(gps.satellites.value());

  // char c;
  // Serial.println("Latitude");

  // Serial.println(gps.location.lat());


  // if (Serial.available()) {
  //   c = Serial.read();

  //   altSerial.print(c);
  // }
  // if (altSerial.available()) {

  //   c = gps.encode(altSerial.read());
  //   delay(100);
  //   Serial.print(c);
  // }
  // Serial.println("--");
  //GPSloop();
  //  if (Serial.available()) {
  //    char c = Serial.read();
  //    GPSSerial.println(c);
  //  }
  //  if (GPSSerial.available()) {
  //    char c = GPSSerial.read();
  //    Serial.print(c);
  //  }
  //measureGPS();
  //measureBNO();
  measureUV();
  measureBMP280();
  measureBME680();
  measureColor();
  measure02();
  measureIR();
  logData();
}


////////////////////////////////////////////Functions///////////////////////////////////
////--------------------------
void measureIR() {
  data[IR] = analogRead(A0);
}
void measureGPS() {

  data[ms] = logTime / 1000;  //fills micros field
  data[UTCTime] = gps.time.value();
  data[Lat] = gps.location.lat();
  data[Long] = gps.location.lng();
  data[Altitude] = gps.altitude.meters();
}
void measureBME680(){
  bme.performReading();
  data[BMETemp] = bme.temperature;
  data[BMEPressure] = bme.pressure/100.0;
  data[BMEHumidity] = bme.humidity;
  data[BMEGas] = bme.gas_resistance;
}
void measureBMP280() {
  
    data[BMPTemp] = bmp.readTemperature();
    data[BMPPressure] = bmp.readPressure();
    data[BMPAltitude] = bmp.readAltitude(1013.25);
 
}
void measureUV() {
  data[UV] = uv.readUV();
}
void measureColor() {

  float red, green, blue;
  //acquire values
  tcs.getRGB(&red, &green, &blue);
  //store in array

  data[Red] = (int(red));
  data[Green] = (int(green));
  data[Blue] = (int(blue));
}
void measure02() {
  float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER);
  data[O2] = oxygenData;
  // Serial.println("Oxygen");
  // Serial.println(oxygenData);
}
void measureBNO() {

  uint8_t system, gyro, accel, mag = 0;
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

  bno.getCalibration(&system, &gyro, &accel, &mag);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
}
// void printEvent(sensors_event_t* event) {
//   double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem
//   if (event->type == SENSOR_TYPE_ACCELEROMETER) {

//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//     data[AccelX] = x;
//     data[AccelY] = y;
//     data[AccelZ] = z;
//   } else if (event->type == SENSOR_TYPE_ORIENTATION) {

//     x = event->orientation.x;
//     y = event->orientation.y;
//     z = event->orientation.z;
//     data[OrientX] = x;
//     data[OrientY] = y;
//     data[OrientZ] = z;
//   } else if (event->type == SENSOR_TYPE_GYROSCOPE) {

//     x = event->gyro.x;
//     y = event->gyro.y;
//     z = event->gyro.z;
//     data[GyroX] = x;
//     data[GyroY] = y;
//     data[GyroZ] = z;
  // } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {

  //   x = event->gyro.x;
  //   y = event->gyro.y;
  //   z = event->gyro.z;
  //   data[RotX] = x;
  //   data[RotY] = y;
  //   data[RotZ] = z;
  // } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {

//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   } else if (event->type == SENSOR_TYPE_GRAVITY) {

//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//     data[GravityX] = x;
//     data[GravityY] = y;
//     data[GravityZ] = z;
//   } else {
//     Serial.print("Unk:");
//   }
// }

//------------------------------------------------------------------------------
//Write data header.
void writeHeader() {
  file.print(F("micros,"));

  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.print(String(dataPoints[i]));
    file.write(",");
  }
  file.println();
}
//-----------------------------------------------------------
//------------------------------------------------------------------------------
// Log a data record. Captured in array
void logData() {
  int column;
  if (!file.open("data00.csv", O_WRONLY | O_CREAT | O_APPEND)) {
    error("open CSV failed");
  }

  // Write ADC data to CSV record.
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.write(',');
    file.print(data[i]);
  }

  //return carriage //newline
  //file.print("\r");
  file.write("\n");
  file.close();
}