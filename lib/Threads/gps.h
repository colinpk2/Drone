#include <Adafruit_GPS.h>
#include <Arduino.h>
#include "data.h"

class GPS {
    public:
    GPS(Adafruit_GPS * GPS, data * data_struct);
    void GPS_tick();
    void INIT();

    private:
    Adafruit_GPS * GPS_;
    uint32_t timer_;
    data * data_;
};