//----Current working on
//Functionality check -- 3/9
//
//
//
//////////////----------Sensors-----------/////////////////////////////////////////////////////////
//Button
//GPS
//Temperature
//GUVA-S12 UV Sensor X
//UV sensor/LTR390-UV --DOESNT WORK
//GEIGER COUNTER
//Altitude/humidity/BMP280
//MICS2714 breakout
//MICS-4514
//////////////----------Functions---------//////////////////////////////////

//READS A1's signal and saves to array?.
//getUV uses the GUVA S12SD
void getUV(){
  analogRead(A1);
  
}
//4 wire temp probe's code
void getTempHumidity(){

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
