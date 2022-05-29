#include <Arduino.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_MPU6050.h>
#include "data.h"

class IMU {
    public:
    IMU(Adafruit_MPU6050 * mpu, data * data_struct);
    void INIT();
    void imu_tick();

    private:
    Adafruit_MPU6050 * mpu_;
    data * data_;
};