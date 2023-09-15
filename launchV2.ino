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



volatile bool ppsFlag = false;

//SD card header count and header fields
const uint8_t ANALOG_COUNT = 43;
String dataPoints[ANALOG_COUNT] = { /* your data points */ };

//Sd storage array
String data[sizeof(dataPoints)];

// Define a SensorData instance to encapsulate sensor objects
// Why? Global vars are stored on SRAM in arduino. SRAM is limited. 
//One SensorData object vs X separate sensor instances means less wasted SRAM
SensorData sensorData;

#define PPS_PIN 19
volatile bool ppsTriggered = false;

// GPS
TinyGPSPlus gps;

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
File dataFile;

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
    Serial.println("Failed to find Temp sensor 1")
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



void logData() {
  //open the SD card
  dataFile = SD.open("/Ascend 2023/Data_Log/data.csv", FILE_WRITE);
  

  //If there's a file, iterate through data points
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
    float uv = ltr390.readUVS();

    dataFile.print(uv);
    dataFile.print(",");

    // Log RGB color sensor data
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    dataFile.print(r);
    dataFile.print(",");
    dataFile.print(g);
    dataFile.print(",");
    dataFile.print(b);
    dataFile.print(",");

    // Log MICS-4514 data
    int mics_4514_no2_reading = analogRead(mics_4514_no2_pin);
    int mics_4514_co_reading = analogRead(mics_4514_co_pin);
    

    //NOT IN LOOP. MUST BE LOGGED
    
    //Get the new temperature and humidity value
    float temperature = aht20.getTemperature();
    float humidity = aht20.getHumidity();

    
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

