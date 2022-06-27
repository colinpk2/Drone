#include "mag.h"

MAG::MAG(Adafruit_HMC5883_Unified* mag) {
    mag_ = mag;
}

void MAG::displaySensorDetails(void)
{
  sensor_t sensor;
  mag_->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void MAG::INIT()
{
    Serial.begin(9600);

    /* Initialise the sensor */
    if(!mag_->begin())
    {
    /* There was a problem detecting the HMC5883 */
    Serial.println("No HMC5883 detected");
    while(1);
    }
}