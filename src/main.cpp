#include <Arduino.h>
#include "data.h"
#include "gps.h"
#include "imu.h"
#include <ChRt.h>
#include <vector>
#include <QMC5883LCompass.h>

#include "mag.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

using std::vector;

#define debug
vector<float> home = {0, 0, 0};
bool cal = true;
int iter = 4;
// Hardware serial port
#define GPSSerial Serial1
uint32_t timer_ = millis();
// Connect to the GPS on the hardware port
Adafruit_GPS gps(&GPSSerial);
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
data data_struct;

GPS gps_(&gps, &data_struct);
IMU imu_(&mpu, &data_struct);
MAG mag_(&mag);

void setup()
{
  Serial.begin(115200);
  delay(100);
  imu_.INIT();
  gps_.INIT();
  mag_.INIT();
  while (gps.lat == 0) {
    Serial.println("getting GPS");
    gps.read();
    if (gps.newNMEAreceived()) {
      gps.lastNMEA(); // this also sets the newNMEAreceived() flag to false
      if (!gps.parse(gps.lastNMEA())){} // this also sets the newNMEAreceived() flag to false
    }
    Serial.println(gps.fix);
    data_struct.gps_fix = gps.fix;
  }
}

void loop()
{
  if (cal) {
    Serial.println("calibrating");
    delay(3000);
    for (int i=0; i < iter; i++) {
      if (gps.newNMEAreceived()) {
        gps.lastNMEA(); // this also sets the newNMEAreceived() flag to false
        if (!gps.parse(gps.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return;
      }

      home[0] += gps.latitude;
      home[1] += gps.longitude;
      home[2] += gps.altitude;
      Serial.println(gps.latitude, 6);
      delay(2500);
    }
    cal = false;
    for (int i=0; i < 3; i++) {
      home[i] /= iter;
    }
    for (int i = 0; i < 2; i++) {
      float deg = (trunc(home[i]/100));
      float min = (home[i] - (deg*100));
      home[i] = (deg + (min/60));
    }
  }
  char c = gps.read();
  if (gps.newNMEAreceived()) {
    gps.lastNMEA(); // this also sets the newNMEAreceived() flag to false
    if (!gps.parse(gps.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;
  }

  if (millis() - timer_ > 100) {
    timer_ = millis(); // reset the timer

    #ifdef debug
    // Serial.print("Fix: "); Serial.print((int)gps.fix);
    // Serial.print(" quality: "); Serial.println((int)gps.fixquality);
    #endif
    data_struct.gps_fix = gps.fix;
    data_struct.gps_x = gps.latitude;
    data_struct.gps_y = gps.longitude;
    data_struct.gps_alt = gps.altitude;
    // Serial.println(data_struct.gps_x, 5);
    // Serial.println(data_struct.gps_y, 5);
  }
  sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    data_struct.a_x = a.acceleration.x;
    data_struct.a_y = a.acceleration.y;
    data_struct.a_z = a.acceleration.z;

    data_struct.g_x = g.gyro.x;
    data_struct.g_y = g.gyro.y;
    data_struct.g_z = g.gyro.z;


    float deg_lat = (trunc(gps.latitude/100));
    float min_lat = (gps.latitude - (deg_lat*100));
    float lat = (deg_lat + (min_lat/60));

    float deg_lon = (trunc(gps.longitude/100));
    float min_lon = (gps.longitude - (deg_lon*100));
    float lon = (deg_lon + (min_lon/60));

    Serial.print("X: "); Serial.println((lon - home[1])*111139, 6);
    // Serial.print("Y: "); Serial.println((lat - home[0])*111139, 6);
    // Serial.println(gps.angle);
    /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  // Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  // Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  // Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
}
