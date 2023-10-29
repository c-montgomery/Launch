
#include "Sensor.h"


////////////////////////////////////////////////////////////
//////////////////////////////   Object declarations and constants
////////////////////////////////////////////////////////////  

#define PPS_PIN 19
volatile bool ppsFlag = false;

//Create
Sensor sensor;

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////SETUP FUNCTIONS             
//////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(chipSelect, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  digitalWrite(chipSelect, HIGH);
  Serial.begin(115200);
  Serial3.begin(9600);
  Wire.begin();


  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    
    
  }
  Serial.println("Card initialized.");
   sensor.writeHeader();

  if (!sensor.uvSensor.begin(&Wire)) {
    Serial.println("Failed to find LTR-390 sensor!");
  }

  if (!sensor.rgbSensor.begin()) {
    Serial.println("Failed to find TCS34725 sensor!");
  }

  if (!sensor.bnoSensor.begin()) {
    Serial.println("Failed to find BNO055 sensor!");
  }
  if (!sensor.aht20Sensor1.begin()){
    Serial.println("Failed to find Temp sensor 1");
  }

  //attachInterrupt(digitalPinToInterrupt(PPS_PIN), PPS_ISR, RISING);
}
//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////LOOPS GON' LOOP             
//////////////////////////////////////////////////////////////////////////////

void loop() {
  unsigned long currentMillis = millis();
  // Your loop code here
  sensor.logData();
}
