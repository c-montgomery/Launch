
#include "Sensor.h"


////////////////////////////////////////////////////////////
//////////////////////////////   Object declarations and constants
////////////////////////////////////////////////////////////  


//Create sensor object
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
 
  if (!sensor.rgb.begin()) {
    Serial.println("inrgbsensor");
    Serial.println("Failed to find TCS34725 sensor!");
  }

  if (!sensor.aht20Sensor1.begin()){
    Serial.println("Failed to find Temp sensor 1");
  }
  Serial.println("started temp");

  if (!sensor.bmp280.begin()){
    Serial.println("Failed to find pressure sensor!");
  }
  Serial.println("started pressure");

}
//////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////LOOPS GON' LOOP             
//////////////////////////////////////////////////////////////////////////////

void loop() {
  unsigned long currentMillis = millis();
  // Your loop code here
  sensor.logData();
  Serial.println("data logged");
}
