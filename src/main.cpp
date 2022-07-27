#include <Arduino.h>
#include "data.h"
#include "gps.h"
#include "imu.h"
#include <ChRt.h>
#include <vector>
#include <QMC5883LCompass.h>
#include <MPU9250.h>

#include "mag.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <PWMServo.h>

#include "kalman.h"

using std::vector;

#define debug
// #define GPS_run

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

PWMServo FR;
PWMServo FL;
PWMServo BR;
PWMServo BL;
PWMServo servo;

void setup()
{
  int min = 1000;
  int max = 2000;

  FR.attach(2, min, max);
  FL.attach(4, min, max);

  BR.attach(6, min, max);
  BL.attach(9, min, max);

  servo.attach(29, 900, 2000);

  Serial.begin(115200);
  delay(100);
  imu_.INIT();
  gps_.INIT();
  mag_.INIT();

  // Serial.println(0);te(0);
  BR.write(0);
  delay(100);
  float i = 180;
  FL.write(i);
  FR.write(i);
  BL.write(i);
  BR.write(i);
  
  delay(5000);

  i = 0;
  FL.write(i);
  FR.write(i);
  BL.write(i);
  BR.write(i);

  delay(10000);

  i = 50;
  FL.write(i);
  FR.write(i);
  BL.write(i);
  BR.write(i);

  #ifdef GPS_run
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
  #endif
}

void loop()
{
  #ifdef GPS_run
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
  #endif

  char c = gps.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (gps.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    gps.lastNMEA(); // this also sets the newNMEAreceived() flag to false
    if (!gps.parse(gps.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  if (millis() - timer_ > 1000) {
    if (gps.fix) {
      // Serial.println("update");
      data_struct.gps_fix = gps.fix;
      float raw_lat = gps.latitude;
      float raw_lon = gps.longitude;
      data_struct.gps_alt = gps.altitude;

      float deg_lat = (trunc(raw_lat/100));
      float min_lat = (raw_lat - (deg_lat*100));
      float lat = (deg_lat + (min_lat/60));

      float deg_lon = (trunc(raw_lon/100));
      float min_lon = (raw_lon - (deg_lon*100));
      float lon = (deg_lon + (min_lon/60));

      data_struct.gps_x = (lon - home[1])*111139;
      data_struct.gps_y = (lat - home[0])*111139;
    }
  }
  
  sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    data_struct.a_x = a.acceleration.x - .47;
    data_struct.a_y = a.acceleration.y + .2;
    data_struct.a_z = a.acceleration.z + .54;

    data_struct.g_x = g.gyro.x;
    data_struct.g_y = g.gyro.y;
    data_struct.g_z = g.gyro.z;

    // Serial.print("X: "); Serial.println(data_struct.gps_y, 6);
    // Serial.print("Y: "); Serial.println(data_struct.gps_x, 6);
    // Serial.print("X: "); Serial.println(lon, 7);
    // Serial.println(gps.angle);
    /* Get a new sensor event */ 
  // Serial.println(data_struct.g_x);
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
  float headingDegrees = heading * 180/M_PI - 110; 
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  Serial.println(data_struct.a_x);
  Serial.println(data_struct.a_y);
  Serial.println(data_struct.a_z);
}


// #include <SparkFunMPU9250-DMP.h>

// #define SerialPort SerialUSB

// MPU9250_DMP imu;

// void setup() 
// {
//   SerialPort.begin(115200);

//   // Call imu.begin() to verify communication with and
//   // initialize the MPU-9250 to it's default values.
//   // Most functions return an error code - INV_SUCCESS (0)
//   // indicates the IMU was present and successfully set up
//   if (imu.begin() != INV_SUCCESS)
//   {
//     while (1)
//     {
//       SerialPort.println("Unable to communicate with MPU-9250");
//       SerialPort.println("Check connections, and try again.");
//       SerialPort.println();
//       delay(5000);
//     }
//   }

//   // Use setSensors to turn on or off MPU-9250 sensors.
//   // Any of the following defines can be combined:
//   // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
//   // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
//   // Enable all sensors:
//   imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

//   // Use setGyroFSR() and setAccelFSR() to configure the
//   // gyroscope and accelerometer full scale ranges.
//   // Gyro options are +/- 250, 500, 1000, or 2000 dps
//   imu.setGyroFSR(2000); // Set gyro to 2000 dps
//   // Accel options are +/- 2, 4, 8, or 16 g
//   imu.setAccelFSR(2); // Set accel to +/-2g
//   // Note: the MPU-9250's magnetometer FSR is set at 
//   // +/- 4912 uT (micro-tesla's)

//   // setLPF() can be used to set the digital low-pass filter
//   // of the accelerometer and gyroscope.
//   // Can be any of the following: 188, 98, 42, 20, 10, 5
//   // (values are in Hz).
//   imu.setLPF(5); // Set LPF corner frequency to 5Hz

//   // The sample rate of the accel/gyro can be set using
//   // setSampleRate. Acceptable values range from 4Hz to 1kHz
//   imu.setSampleRate(10); // Set sample rate to 10Hz

//   // Likewise, the compass (magnetometer) sample rate can be
//   // set using the setCompassSampleRate() function.
//   // This value can range between: 1-100Hz
//   imu.setCompassSampleRate(10); // Set mag rate to 10Hz
// }

// void printIMUData(void)
// {  
//   // After calling update() the ax, ay, az, gx, gy, gz, mx,
//   // my, mz, time, and/or temerature class variables are all
//   // updated. Access them by placing the object. in front:

//   // Use the calcAccel, calcGyro, and calcMag functions to
//   // convert the raw sensor readings (signed 16-bit values)
//   // to their respective units.
//   float accelX = imu.calcAccel(imu.ax);
//   float accelY = imu.calcAccel(imu.ay);
//   float accelZ = imu.calcAccel(imu.az);
//   float gyroX = imu.calcGyro(imu.gx);
//   float gyroY = imu.calcGyro(imu.gy);
//   float gyroZ = imu.calcGyro(imu.gz);
//   float magX = imu.calcMag(imu.mx);
//   float magY = imu.calcMag(imu.my);
//   float magZ = imu.calcMag(imu.mz);
  
//   // Serial.println("Accel: " + String(accelX) + ", " +
//   //             String(accelY) + ", " + String(accelZ) + " g");
//   Serial.println("Gyro: " + String(gyroX) + ", " +
//               String(gyroY) + ", " + String(gyroZ) + " dps");
//   // Serial.println("Mag: " + String(magX) + ", " +
//   //             String(magY) + ", " + String(magZ) + " uT");
//   // Serial.println("Time: " + String(imu.time) + " ms");
//   // Serial.println();
// }

// void loop() 
// {
//   // dataReady() checks to see if new accel/gyro data
//   // is available. It will return a boolean true or false
//   // (New magnetometer data cannot be checked, as the library
//   //  runs that sensor in single-conversion mode.)
//   if ( imu.dataReady() )
//   {
//     // Call update() to update the imu objects sensor data.
//     // You can specify which sensors to update by combining
//     // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
//     // UPDATE_TEMPERATURE.
//     // (The update function defaults to accel, gyro, compass,
//     //  so you don't have to specify these values.)
//     imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
//   }
//   printIMUData();
//   // delay(2000);
// }

// #include <SparkFunMPU9250-DMP.h>

// #define SerialPort SerialUSB

// MPU9250_DMP imu;

// unsigned long stepCount = 0;
// unsigned long stepTime = 0;
// unsigned long lastStepCount = 0;

// const signed char orientationMatrix[9] = {
//   1, 0, 0,
//   0, 1, 0,
//   0, 0, 1
// };
// unsigned char lastOrient = 0;

// void setup() 
// {
//   SerialPort.begin(115200);

//   // Call imu.begin() to verify communication and initialize
//   if (imu.begin() != INV_SUCCESS)
//   {
//     while (1)
//     {
//       SerialPort.println("Unable to communicate with MPU-9250");
//       SerialPort.println("Check connections, and try again.");
//       SerialPort.println();
//       delay(5000);
//     }
//   }
  
//   imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
//   imu.dmpSetOrientation(orientationMatrix);
// }

// void loop() 
// {
//   if ( imu.fifoAvailable() )
//   {
//     imu.dmpUpdateFifo();
//     unsigned char orient = imu.dmpGetOrientation();
//     if (orient != lastOrient)
//     {
//       switch (orient)
//       {
//       case ORIENT_PORTRAIT:
//         Serial.println("Portrait");
//         break;
//       case ORIENT_LANDSCAPE:
//         Serial.println("Landscape");
//         break;
//       case ORIENT_REVERSE_PORTRAIT:
//         Serial.println("Portrait (Reverse)");
//         break;
//       case ORIENT_REVERSE_LANDSCAPE:
//         Serial.println("Landscape (Reverse)");
//         break;
//       }
//       lastOrient = orient;
//     }
//   }
// }
