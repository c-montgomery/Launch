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

#include <Adafruit_TCS34725.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>
#include <DFRobot_OxygenSensor.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_BNO055.h>


////------------------------------------------TO-DO-------------------------------
//GPS SDCARD 9DOF TEMP
//
///////////////////////////////////////////////////constants//////////////////////////////////////////
#define RXpin 17
#define TXpin 16
#define FILE_BASE_NAME "Data"  //sd
const int chipSelect = 53;


//SD card header count and header fields
const uint8_t ANALOG_COUNT = 19;
String dataPoints[ANALOG_COUNT] = { "micros", "lat", "long", "Altitude", "OrientX", "OrientY", "OrientZ", "AccelX", "AccelY", "AccelZ", "GravityX", "GravityY", "GravityZ", "Red", "Green", "Blue", "IR", "Temperature", "UV" };
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


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

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

//GPS objs
TinyGPSPlus gps;

AltSoftSerial altSerial;

// data[0] = logTime;
// data[1] = gps.time.value();
// data[2] = gps.location.lat();
// data[3] = gps.location.lng();
// data[3] = gps.altitude.meters();
// data[4] = "OrientX ";
// data[5] = "OrientY";
// data[6] = "OrientZ";
// data[8] = "AccelX";
// data[9] = "AccelY";
// data[10] = "AccelZ";
// data[11] = "GravityX";
// data[12] = "GravityY";
// data[13] = "GravityZ";
// data[14] = "Red";
// data[15] = "Green";
// data[16] = "Blue";
// data[17] = "";
// data[18] = "";

////==============================================================================

long msElapsed = millis();


void setup() {
  //SD card base file name
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";
  //  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  altSerial.begin(9600);
  delay(1000);


  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
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

  writeHeader();

  //--------------------------
  //  //INITIALIZE BNO 055 ACCELEROMETER
  //  if (!bno.begin())
  //  {
  //    /* There was a problem detecting the BNO055 ... check your connections */
  //    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //    // while (1);
  //  }
  //  delay(1000);
  //  // Define pin modes for TX and RX
  //  pinMode(RXpin, INPUT);
  //
  //  pinMode(TXpin, OUTPUT);

  //   DEBUG_PORT.begin(9600);
  //   while (!DEBUG_PORT)
  //     ;

  //   DEBUG_PORT.print(F("NMEA.INO: started\n"));
  //   DEBUG_PORT.print(F("  fix object size = "));
  //   DEBUG_PORT.println(sizeof(gps.fix()));
  //   DEBUG_PORT.print(F("  gps object size = "));
  //   DEBUG_PORT.println(sizeof(gps));
  //   DEBUG_PORT.println(F("Looking for GPS device on " GPS_PORT_NAME));

  // #ifndef NMEAGPS_RECOGNIZE_ALL
  // #error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
  // #endif

  // #ifdef NMEAGPS_INTERRUPT_PROCESSING
  // #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
  // #endif

  // #if !defined(NMEAGPS_PARSE_GGA) & !defined(NMEAGPS_PARSE_GLL) & !defined(NMEAGPS_PARSE_GSA) & !defined(NMEAGPS_PARSE_GSV) & !defined(NMEAGPS_PARSE_RMC) & !defined(NMEAGPS_PARSE_VTG) & !defined(NMEAGPS_PARSE_ZDA) & !defined(NMEAGPS_PARSE_GST)

  //   DEBUG_PORT.println(F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed."));

  // #else
  //   if (gps.merging == NMEAGPS::NO_MERGING) {
  //     DEBUG_PORT.print(F("\nWARNING: displaying data from "));
  //     DEBUG_PORT.print(gps.string_for(LAST_SENTENCE_IN_INTERVAL));
  //     DEBUG_PORT.print(F(" sentences ONLY, and only if "));
  //     DEBUG_PORT.print(gps.string_for(LAST_SENTENCE_IN_INTERVAL));
  //     DEBUG_PORT.println(F(" is enabled.\n"
  //                          "  Other sentences may be parsed, but their data will not be displayed."));
  //   }
  // #endif

  //   DEBUG_PORT.print(F("\nGPS quiet time is assumed to begin after a "));
  //   DEBUG_PORT.print(gps.string_for(LAST_SENTENCE_IN_INTERVAL));
  //   DEBUG_PORT.println(F(" sentence is received.\n"
  //                        "  You should confirm this with NMEAorder.ino\n"));

  //   trace_header(DEBUG_PORT);
  //   DEBUG_PORT.flush();

  //   gpsPort.begin(9600);
}

void loop() {
  // Time for next record.
  logTime += 1000UL * SAMPLE_INTERVAL_MS;

  // Wait for log time.
  int32_t diff;
  do {
    diff = micros() - logTime;
  } while (diff < 0);

  // // Check for data rate too high.
  // if (diff > 10) {
  //   error("Missed data record");
  // }

  Serial.println("beginning of loop");
  Serial.println(gps.satellites.value());
  delay(50);
  char c;
  Serial.println("Latitude");

  Serial.println(gps.location.lat());


  if (Serial.available()) {
    c = Serial.read();
    
    altSerial.print(c);
  }
  if (altSerial.available()) {
    
    c = gps.encode(altSerial.read());
    delay(100);
    Serial.print(c);
    
  }






  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);


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
  measureColor();
  delay(300);
  logData();
}


////////////////////////////////////////////Functions///////////////////////////////////
////--------------------------
void measureColor() {

  float red, green, blue;
  //acquire values
  tcs.getRGB(&red, &green, &blue);
  //store in array
  data[0] = logTime;
  data[1] = gps.time.value();
  data[2] = gps.location.lat();
  data[3] = gps.location.lng();
  data[3] = gps.altitude.meters();
  data[13] = (int(red));
  data[14] = (int(green));
  data[15] = (int(blue));
  Serial.println();
}
//--------------------------
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;

    y = event->orientation.y;
    z = event->orientation.z;
    data[3] = x;
    data[4] = y;
    data[5] = z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
  //Print elapsed time
  Serial.println("ms Passed: ");
  Serial.println(millis());
  Serial.println(msElapsed);
}

//------------------------------------------------------------------------------
//Write data header.
void writeHeader() {
  file.print(F("micros,"));

  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.print(String(dataPoints[i]));
    file.write(",");
  }
}
//-----------------------------------------------------------
//------------------------------------------------------------------------------
// Log a data record. Captured in array
void logData() {
  int column;
  if (!file.open("data00.csv", O_WRONLY | O_CREAT | O_APPEND)) {
    error("open CSV failed");
  }

  // file.write(data[14].toInt());
  // file.write(",");

  Serial.println("just logged data[14]: ^^^^^^^^^^^^^^^^^^^^^^^");


  // Write ADC data to CSV record.
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.write(',');
    file.print(data[i]);
  }

  //return carriage //newline
  file.print("\r");
  file.write("\n");
  file.close();
}