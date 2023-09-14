/**************************************************************************
   Tests the getTemperature and getHumidity functions of the aht20 library
 **************************************************************************/
#include <Wire.h>
#include <AHT20.h>
AHT20 aht20;

void setup()
{
  Serial.begin(115200);
  Serial.println("Humidity AHT20 examples");

  Wire.begin(); //Join I2C bus
  //Check if the AHT20 will acknowledge
  if (aht20.begin() == false)
  {
 
  }
  Serial.println("AHT20 acknowledged.");
}

void loop()
{
  //If a new measurement is available
  if (aht20.available() == true)
  {
    //Get the new temperature and humidity value
    float temperature = aht20.getTemperature();
    float humidity = aht20.getHumidity();

  }

  //The AHT20 can respond with a reading every ~50ms. However, increased read time can cause the IC to heat around 1.0C above ambient.
  //The datasheet recommends reading every 2 seconds.
  delay(2000);
}
