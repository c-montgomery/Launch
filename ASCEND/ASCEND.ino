//----Current working on
//Functionality check -- 2/9
//
//
//
//////////////----------Sensors-----------/////////////////////////////////////////////////////////
//Button
//GPS
//Temperature
//GUVA-S12 UV Sensor
//UV sensor/LTR390-UV
//GEIGER COUNTER
//Altitude/humidity/BMP280
//MICS2714 breakout
//MICS-4514
//////////////----------Functions---------//////////////////////////////////

//READS A1's signal and saves to array.
int getUV(){
  analogRead(A1);
  
}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

void setup() {
  //Begin Serial
  Serial.begin(115200);
  //create analog in to read GUVA UV SENSOR
  pinMode(A0, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  getUV();

}
