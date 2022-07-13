#include "imu.h"

IMU::IMU(Adafruit_MPU6050 * mpu, data * data_struct) {
    mpu_ = mpu;
}

void IMU::INIT() {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
    #ifdef debug
    Serial.println("MPU6050 Setup");
    #endif
  // Try to initialize!
    if (!mpu_->begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    #ifdef debug
    Serial.println("MPU6050 Found");
    #endif
    Serial.println("Init IMU Setup");
    mpu_->setAccelerometerRange(MPU6050_RANGE_8_G);

    mpu_->setGyroRange(MPU6050_RANGE_500_DEG);

    mpu_->setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(100);
}

void IMU::imu_tick() {
    
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu_->getEvent(&a, &g, &temp);
    Serial.println(a.acceleration.x - 0.65);
    // data_->a_x = a.acceleration.x;
    // data_->a_y = a.acceleration.y;
    // data_->a_z = a.acceleration.z;

    // data_->g_x = g.gyro.x;
    // data_->g_y = g.gyro.y;
    // data_->g_z = g.gyro.z;

  delay(500);
}
