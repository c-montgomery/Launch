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



/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1000;
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////Objects///////////////////////////////////////////////////
//Create instance Software Serial
SoftwareSerial mySerial(RXpin, TXpin);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);//Instantiate BNO-055


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);



long msElapsed = millis();


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
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
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
  //Serial.println("card initialized.");


}

void loop() {
  Serial.println("beginning of loop");
  Serial.println("ms Passed: ");
  Serial.println(millis());
  //    File dataFile = SD.open("holymoley.txt", FILE_WRITE);//Instantiate SD card ONLY 1 open at a time!!!!!
  //    Serial.println(dataFile);
  //    Serial.println("^^^^^dataFile");
  //    // if the file is available, write to it:
  //    if (dataFile) {
  //      dataFile.println("fartso, fatso");
  //      dataFile.close();
  //      // print to the serial port too:
  //      Serial.println("closed card");//////////////////////////////////////////
  //    }
  //    // if the file isn't open, pop up an error:
  //    else {
  //      Serial.println("ERROR opening holeymoley.txt");
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
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

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


//////////////////////////////////////////Functions///////////////////////////////////
//--------------------------
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
