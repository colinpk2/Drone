#include <Arduino.h>
#include "data.h"
#include "gps.h"
#include "imu.h"
#include <ChRt.h>

// Hardware serial port
#define GPSSerial Serial1
uint32_t timer = millis();
// Connect to the GPS on the hardware port
Adafruit_GPS gps(&GPSSerial);
Adafruit_MPU6050 mpu;
data data_struct;
GPS gps_(&gps, &data_struct);
IMU imu_(&mpu, &data_struct);

static THD_WORKING_AREA(GPS_WA, 1024);
// static THD_WORKING_AREA(IMU_WA, 8192);

static THD_FUNCTION(GPS_THD, arg) {
    
    while(true) {
        // gps_.GPS_tick();
        Serial.println("GPS Thread");

        chThdSleepMilliseconds(10);
    }
}

// static THD_FUNCTION(IMU_THD, arg) {
    
//     while(true) {
//         imu_.imu_tick();

//         chThdSleepMilliseconds(10);
//     }
// }

void chSetup() {
    chThdCreateStatic(GPS_WA, sizeof(GPS_WA), NORMALPRIO,
                      GPS_THD, NULL);
    while (true);
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Initialize");
  imu_.INIT();
  gps_.INIT();
  Serial.println("Starting ChibiOS");
  // chBegin(chSetup);
}

void loop()
{
  imu_.imu_tick();
  // gps_.GPS_tick();
  // Serial.println("Alt: ");
  // Serial.println(data_struct.gps_alt);
  // Serial.println(data_struct.a_z);
}
