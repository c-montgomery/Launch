////------------------------------------------TO-DO-------------------------------
//GPS SDCARD 9DOF TEMP
//
//
//
//
//
//



#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>


#define RXpin 4
#define TXpin 3

//Create instance Software Serial
SoftwareSerial mySerial(4, 3);

void setup(){
  Serial.println("Setting up ...");
  while(!Serial){
    //Start Serial and SoftwareSerial
   Serial.begin(115200);
   mySerial.begin(9600);
   
   delay(2000);
   Serial.println("Serial started "); 
  }
}

void loop(){
  
  Serial.println(Serial.available());
    if(Serial.available()){
      Serial.println("GPS ");
      mySerial.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    }
  
  
}
