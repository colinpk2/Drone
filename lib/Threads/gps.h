#include <Adafruit_GPS.h>
#include <Arduino.h>

class GPS {
    public:
    GPS(Adafruit_GPS GPS, uint32_t timer);
    void GPS_tick();
    void INIT();

    private:
    Adafruit_GPS GPS_;
    uint32_t timer_;
};