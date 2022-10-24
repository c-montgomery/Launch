#include <Adafruit_TCS34725.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>
#include <DFRobot_OxygenSensor.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BNO055.h>


////------------------------------------------TO-DO-------------------------------
//GPS SDCARD 9DOF TEMP
//
//
//


#include <SoftwareSerial.h>

/////////////////////////////////////////////////constants//////////////////////////////////////////
#define RXpin 19
#define TXpin 18
const int chipSelect = 53;




#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.

#define GPSSerial Serial1

// set to false if using a common cathode LED//FOR RGB sensor
#define commonAnode true

// our RGB -> eye-recognized gamma color
byte gammatable[256];

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1000;
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////Objects///////////////////////////////////////////////////
//Create instance Software Serial
SoftwareSerial mySerial(RXpin, TXpin);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);//Instantiate BNO-055







void setup() {
//  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  //mySerial.begin(9600);

// wait for hardware serial to appear
  while (!Serial) delay(10);

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);


//--------------------------
//INITIALIZE BNO 055 ACCELEROMETER
if (!bno.begin())
{
  /* There was a problem detecting the BNO055 ... check your connections */
  Serial2.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  // while (1);
}
delay(1000);
// Define pin modes for TX and RX
pinMode(RXpin, INPUT);

pinMode(TXpin, OUTPUT);




//Serial.print("Initializing SD card...");
//
//// see if the card is present and can be initialized:
//if (!SD.begin(chipSelect)) { //ChipSelect on mega is 53
//  Serial.println("Card failed, or not present");
//  // don't do anything more:
//  // while (1);//REMOVE after SD is on breadboard
//}
//Serial2.println("card initialized.");


}

void loop() {
//    File dataFile = SD.open("holymoley.txt", FILE_WRITE);//Instantiate SD card ONLY 1 open at a time!!!!!
//    Serial2.println(dataFile);
//    Serial2.println("^^^^^dataFile");
//    // if the file is available, write to it:
//    if (dataFile) {
//      dataFile.println("fartso, fatso");
//      dataFile.close();
//      // print to the serial port too:
//      Serial2.println("closed card");//////////////////////////////////////////
//    }
//    // if the file isn't open, pop up an error:
//    else {
//      Serial2.println("ERROR opening holeymoley.txt");
//    }


 
    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    sensors_event_t orientationData , angVelocityData , linearAccelData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  
    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
  
    printEvent(&accelerometerData);
  printEvent(&gravityData);

    int8_t boardTemp = bno.getTemp();
    Serial2.println();
    Serial2.print(F("temperature: "));
    Serial2.println(boardTemp);
  
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial2.println();
    Serial2.print("Calibration: Sys=");
    Serial2.print(system);
    Serial2.print(" Gyro=");
    Serial2.print(gyro);
    Serial2.print(" Accel=");
    Serial2.print(accel);
  
  
    Serial2.println("--");


  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.println(c);
  }
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.print(c);
  }
}


//////////////////////////////////////////Functions///////////////////////////////////

//--------------------------
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial2.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial2.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial2.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial2.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial2.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial2.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial2.print("Unk:");
  }

  Serial2.print("\tx= ");
  Serial2.print(x);
  Serial2.print(" |\ty= ");
  Serial2.print(y);
  Serial2.print(" |\tz= ");
  Serial2.println(z);
}