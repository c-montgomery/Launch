#include <RingBuf.h>
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
//
//

//
//
//
///////////////////////////////////////////////////constants//////////////////////////////////////////
#define RXpin 17
#define TXpin 16
#define FILE_BASE_NAME "Data"                            //sd
const int chipSelect = 53;


//SD card header count and header fields
const uint8_t ANALOG_COUNT = 18;
String dataPoints[18] = {"lat", "long", "Altitude", "OrientX", "OrientY", "OrientZ", "AccelX", "AccelY", "AccelZ", "GravityX", "GravityY", "GravityZ", "Red", "Green", "Blue", "IR", "Temperature", "UV"};
//Sd storage array
String data[ANALOG_COUNT];

#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.

#define GPSSerial Serial1

//SD card
const uint32_t SAMPLE_INTERVAL_MS = 1000;

/* Set the delay between fresh samples BNO055 */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1000;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////Objects///////////////////////////////////////////////////
////Create instance Software Serial
SoftwareSerial mySerial(RXpin, TXpin);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);//Instantiate BNO-055


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

////------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime;

////==============================================================================

long msElapsed = millis();


void setup() {
  //  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  //mySerial.begin(9600);
  while (!Serial) {
    delay(10);
  }
  Serial.write("test");
  // wait for hardware serial to appear
  while (!Serial) delay(10);

  // 9600 baud is the default rate for the Ultimate GPS
  //GPSSerial.begin(9600);


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



//
//    Serial.print("Initializing SD card...");
//  
//   // see if the card is present and can be initialized:
//    if (!SD.begin(chipSelect)) { //ChipSelect on mega is 53
//      Serial.println("Card failed, or not present");
//      // don't do anything more:
//      // while (1);//REMOVE after SD is on breadboard
//    }
//    Serial.println("card initialized.");


}

void loop() {
  //  Serial.println("beginning of loop");
  //  Serial.println("ms Passed: ");
  //  Serial.println(millis());
  //      File dataFile = SD.open("holymoley.txt", FILE_WRITE);//Instantiate SD card ONLY 1 open at a time!!!!!
  //      Serial.println(dataFile);
  //      Serial.println("^^^^^dataFile");
  //      // if the file is available, write to it:
  //      if (dataFile) {
  //        dataFile.println("fartso, fatso");
  //        dataFile.close();
  //        // print to the serial port too:
  //        Serial.println("closed card");//////////////////////////////////////////
  //      }
  //      // if the file isn't open, pop up an error:
  //      else {
  //        Serial.println("ERROR opening holeymoley.txt");
  //      }




  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);


  Serial.println("--");
  measureColor();

  //  if (Serial.available()) {
  //    char c = Serial.read();
  //    GPSSerial.println(c);
  //  }
  //  if (GPSSerial.available()) {
  //    char c = GPSSerial.read();
  //    Serial.print(c);
  //  }
}


////////////////////////////////////////////Functions///////////////////////////////////
////--------------------------
void measureColor() {
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  Serial.print("R:\t"); Serial.print(int(red));
  Serial.print("\tG:\t"); Serial.print(int(green));
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.println();
}
//--------------------------
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;

    y = event->orientation.y;
    z = event->orientation.z;
    data[3] = x;
    data[4] = y;
    data[5] = z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
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
// Write data header.
void writeHeader() {
  file.print(F("micros"));
  file.write(",");
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.print(dataPoints[i]);
    file.write(",");
  }
  file.println();
}
//-----------------------------------------------------------


//------------------------------------------------------------------------------
// Log a data record.
void logData() {
  int column;
  //data[0] = logTime;
  data[1] = "Lat";
  data[2] = "Long";

  data[6] = "";
  data[7] = "";
  data[8] = "";
  data[9] = "";
  data[10] = "";
  data[11] = "";
  data[12] = "";
  data[13] = "";
  data[14] = "";
  data[15] = "";
  data[16] = "";
  data[17] = "";
  // Read all channels to avoid SD write latency between readings.
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    data[i] = analogRead(i);
  }
  // Write data to file.  Start with log time in micros.
  file.print(logTime);

  // Write ADC data to CSV record.
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.write(',');
    file.print(data[i]);
  }
  file.println();
}
//============================================================================
