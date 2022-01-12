#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

bool gpsFixAcquired = false;

bool isGPSFixAcquired(void)
{
  return(gpsFixAcquired);
}

void setupGPS()
{
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  // Ask for firmware version
  //GPSSerial.println(PMTK_Q_RELEASE);
}

bool loopGPSfirst(unsigned char dataBuffer[]) // called from main loop
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  //if (GPSECHO&& SERIAL_DEBUG)
  //  if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    if (GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    {
#ifdef LOTS_OF_DEBUG
      Serial.print("\nTime: ");
      if (GPS.hour < 10) { Serial.print('0'); }
      Serial.print(GPS.hour, DEC); Serial.print(':');
      if (GPS.minute < 10) { Serial.print('0'); }
      Serial.print(GPS.minute, DEC); Serial.print(':');
      if (GPS.seconds < 10) { Serial.print('0'); }
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.println(""); // EOL

      Serial.print("Date: ");
      Serial.print(GPS.day, DEC); Serial.print('/');
      Serial.print(GPS.month, DEC); Serial.print("/20");
      Serial.println(GPS.year, DEC);
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitudeDegrees, 8);// Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.println(GPS.longitudeDegrees, 8);// Serial.println(GPS.lon);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
#endif // LOTS_OF_DEBUG


      unsigned long int secondWithinDay = 0;
      secondWithinDay = (GPS.hour*3600)+(GPS.minute*60)+(GPS.seconds); // [0,86400] too big for 2 bytes, 3 should work

      //Serial.print("secondWithinDay: "); Serial.println((int)secondWithinDay);

      dataBuffer[0]  = 0x01; // S1;
      dataBuffer[1]  = 0x02; // S2;
      dataBuffer[2]  = 0x03; // S3;
      dataBuffer[3]  = 0x04; // X1;
      dataBuffer[4]  = 0x05; // X2;
      dataBuffer[5]  = 0x06; // X3;
      dataBuffer[6]  = 0x07; // Y1;
      dataBuffer[7]  = 0x08; // Y2;
      dataBuffer[8]  = 0x09; // Y3;
      dataBuffer[9]  = 0x0A; // A1;
      dataBuffer[10] = 0x0B; // A2;
      

      if (GPS.fix && GPS.altitude != 0) {
        gpsFixAcquired = true;
        
        // assemble 11 byte packet
        //012 345 678 90
        //ttt xxx yyy aa

        unsigned long int altitude_fixed;

        // copy time
        unsigned char S1, S2, S3;

        S1 = ((secondWithinDay & 0x00FF0000) >> 16);
        S2 = ((secondWithinDay & 0x0000FF00) >> 8); 
        S3 = ((secondWithinDay & 0x000000FF) >> 0); 

        // copy x
        unsigned char X1, X2, X3;

        unsigned long int long_abs = (unsigned long int)fabs(GPS.longitudeDegrees * 10000000.0);
        unsigned long int lat_abs  = (unsigned long int)fabs(GPS.latitudeDegrees  * 10000000.0);
        unsigned long int long_fixed_shifted = long_abs >> 7; // remove 7 bits of precision to fit 180*(10^7) into 24 bits
        unsigned long int lat_fixed_shifted  = lat_abs >> 7; // remove 7 bits of precision to fit 90*(10^7) into 24 bits (6 might actually work)


        X1 = ((long_fixed_shifted & 0x00FF0000) >> 16);
        X2 = ((long_fixed_shifted & 0x0000FF00) >> 8);
        X3 = ((long_fixed_shifted & 0x000000FF) >> 0);

        // copy y
        unsigned char Y1, Y2, Y3;
        Y1 = ((lat_fixed_shifted & 0x00FF0000) >> 16);
        Y2 = ((lat_fixed_shifted & 0x0000FF00) >> 8);
        Y3 = ((lat_fixed_shifted & 0x000000FF) >> 0);

        // copy altitude
        altitude_fixed = (unsigned long int)GPS.altitude;
        unsigned char A1, A2;
        A1 = ((altitude_fixed & 0x0000FF00) >> 8); 
        A2 = ((altitude_fixed & 0x000000FF) >> 0); 

        // form Voltron
        dataBuffer[0]  = S1;
        dataBuffer[1]  = S2;
        dataBuffer[2]  = S3;
        dataBuffer[3]  = X1;
        dataBuffer[4]  = X2;
        dataBuffer[5]  = X3;
        dataBuffer[6]  = Y1;
        dataBuffer[7]  = Y2;
        dataBuffer[8]  = Y3;
        dataBuffer[9]  = A1;
        dataBuffer[10] = A2;

        return(true); // packet ready
      } // if fix and altitude valid

    } // if parsed
    return(false); // nothing to do
  } // if newNMEAreceived

  return(false); // nothing to do
} // loopGPSfirst
