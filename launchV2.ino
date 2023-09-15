#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LTR390.h>
#include <TinyGPS++.h>
#include <AHT20.h>
#include <SD.h>
#include <SPI.h>
#include "SensorData.cpp"



////////////////////////////////////////////////////////////
//////////////////////////////   Object declarations and constants
////////////////////////////////////////////////////////////  


#define PPS_PIN 19
volatile bool ppsFlag = false;

//SD card header count and header fields
const uint8_t ANALOG_COUNT = 43;
String dataPoints[ANALOG_COUNT] = { "ms", "UTCTime", "Altitude", "lat", "long", 
"Altitude", "BNOonboardTemp", "temperatureOutside", "OrientX", "OrientY", "OrientZ", 
"AccelX", "AccelY", "AccelZ", "GravityX", "GravityY", "GravityZ", "Magx", "Magy", 
"BMPAltitude", "BMPTemp", "BMPPressure", "BMETemp", "BMEPressure", "BMEHumidity", 
"BMEGas", "Red", "Green", "Blue", "IR", "UV", "Pressure", "02" };

//Sd storage array
String data[sizeof(dataPoints)];

// Define a SensorData instance to encapsulate sensor objects
// Why? Global vars are stored on SRAM in arduino. SRAM is limited. 
//One SensorData object vs X separate sensor instances means less wasted SRAM
SensorData sensorData;



/////////////////////////////////Analog pins

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
  logData();
}

  ////////////////////////////Object names in SensorData()
  // Adafruit_LTR390 uvSensor;
  // Adafruit_TCS34725 rgbSensor;
  // Adafruit_BNO055 bnoSensor;
  // AHT20 aht20Sensor1;
  // TinyGPSPlus gps;
  // File dataFile;

void logData() {
  //open the SD card
  sensorData.dataFile = SD.open("/Ascend 2023/Data_Log/data.csv", FILE_WRITE);
  

  //If there's a file, iterate through data points
  if (sensorData.dataFile) {
    // Log time
    sensorData.dataFile.print(gps.time.value());
    sensorData.dataFile.print(",");

    // Log GPS data
    sensorData.dataFile.print(gps.location.lat(), 6);
    sensorData.dataFile.print(",");
    sensorData.dataFile.print(gps.location.lng(), 6);
    sensorData.dataFile.print(",");
    sensorData.dataFile.print(gps.altitude.meters());
    sensorData.dataFile.print(",");
    sensorData.dataFile.print(gps.speed.kmph());
    sensorData.dataFile.print(",");
    sensorData.dataFile.print(gps.course.deg());
    sensorData.dataFile.print(",");
    sensorData.dataFile.print(gps.satellites.value());
    sensorData.dataFile.print(",");

    // Log UV sensor data
    float uv = sensorData.ltr390.readUVS();

    sensorData.dataFile.print(uv);
    sensorData.dataFile.print(",");

    // Log RGB color sensor data
    uint16_t r, g, b, c;
    sensorData.tcs.getRawData(&r, &g, &b, &c);
    sensorData.dataFile.print(r);
    sensorData.dataFile.print(",");
    sensorData.dataFile.print(g);
    sensorData.dataFile.print(",");
    sensorData.dataFile.print(b);
    sensorData.dataFile.print(",");

    // Log MICS-4514 data
    int mics_4514_no2_reading = analogRead(mics_4514_no2_pin);
    int mics_4514_co_reading = analogRead(mics_4514_co_pin);
    

    //NOT IN LOOP. MUST BE LOGGED
    
    //Get the new temperature and humidity value
    float temperature = sensorData.aht20Sensor1.getTemperature();
    float humidity = sensorData.aht20Sensor1.getHumidity();

    
    sensorData.dataFile.print(mics_4514_no2_reading);
    sensorData.dataFile.print(",");
    sensorData.dataFile.println(mics_4514_co_reading);

    // End the line
    sensorData.dataFile.println();

    // Close the file
    sensorData.dataFile.close();
    Serial.println("Data logged to data.csv");

  } else {
    sensorData.dataFile.close();
    Serial.println("Error opening data.csv");
    
  }
}

