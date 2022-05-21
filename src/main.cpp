#include <Arduino.h>
#include "gps.h"
#include "imu.h"
// what's the name of the hardware serial port?
#define GPSSerial Serial1

uint32_t timer = millis();

// Connect to the GPS on the hardware port
Adafruit_GPS gps(&GPSSerial);

GPS gps_(gps, timer);
IMU imu_();




void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  

  gps_.INIT();

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() // run over and over again
{
  gps_.GPS_tick();
}
