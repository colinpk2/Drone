#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#pragma once

class MAG {
    public:
    MAG(Adafruit_HMC5883_Unified* mag);
    void displaySensorDetails(void);
    void INIT();

    private:
    Adafruit_HMC5883_Unified* mag_;
};