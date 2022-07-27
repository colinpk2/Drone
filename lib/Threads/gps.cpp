#include "gps.h"

GPS::GPS(Adafruit_GPS * GPS, data * data_struct) {
    GPS_ = GPS;
    timer_ = millis();
}

void GPS::GPS_tick() {
    // read data from the GPS in the 'main loop'
  char c = GPS_->read();
  // if you want to debug, this is a good time to do it!
  // if (GPSECHO)
  //   if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS_->newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    GPS_->lastNMEA(); // this also sets the newNMEAreceived() flag to false
    if (!GPS_->parse(GPS_->lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer_ > 2000) {
    timer_ = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS_->hour < 10) { Serial.print('0'); }
    Serial.print(GPS_->hour, DEC); Serial.print(':');
    if (GPS_->minute < 10) { Serial.print('0'); }
    Serial.print(GPS_->minute, DEC); Serial.print(':');
    if (GPS_->seconds < 10) { Serial.print('0'); }
    Serial.print(GPS_->seconds, DEC); Serial.print('.');
    if (GPS_->milliseconds < 10) {
      Serial.print("00");
    } else if (GPS_->milliseconds > 9 && GPS_->milliseconds < 100) {
      Serial.print("0");
    }
    Serial.print("Fix: "); Serial.print((int)GPS_->fix);
    Serial.print(" quality: "); Serial.println((int)GPS_->fixquality);
    if (GPS_->fix) {
      Serial.print("Location: ");
      Serial.print(GPS_->latitude, 4); Serial.print(GPS_->lat);
      Serial.print(", ");
      Serial.print(GPS_->longitude, 4); Serial.println(GPS_->lon);
      Serial.print("Speed (knots): "); Serial.println(GPS_->speed);
      Serial.print("Angle: "); Serial.println(GPS_->angle);
      Serial.print("Altitude: "); Serial.println(GPS_->altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS_->satellites);
    }
    // data_->gps_x = GPS_->latitude;
    // data_->gps_y = GPS_->longitude;
    // data_->gps_alt = GPS_->altitude;
  }
}

void GPS::INIT() {
    
    Serial.println("Init GPS Setup");

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS_->begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS_->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS_->sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS_->sendCommand(PGCMD_ANTENNA);
    // Ask for firmware version
    Serial1.println(PMTK_Q_RELEASE);
    // delay(1000);
}