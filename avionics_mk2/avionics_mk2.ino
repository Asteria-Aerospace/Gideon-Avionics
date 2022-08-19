// Asteria Aerospace Gideon Mk2 Arduino Feather M0 Avionics stack

// enable this to stream telemetry data to serial/USB.
// This is blocking and will stall operations if no USB/serial
// receiver is listening.
//#define SERIAL_DEBUG 1

//#define ENABLE_INS
//#define ENABLE_BMP
//#define ENABLE_SD
#define ENABLE_GPS
#define ENABLE_RADIO
// this boosts every 100th packet to power level 20
#define USE_RADIO_BOOST
#define ENABLE_BATTERY

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>

// Config for diagnostic LEDs
#define GREEN_LED_PORT 8
#define RED_LED_PORT 13

unsigned long int radioPacketCount = 0;

// These variables accumulate the most current data as it arrives, for later reporting
double Time, Baro, TempC, TempF, BaroCal, AltiM, AltiF, AccelX, AccelY, AccelZ;

float batVoltage(0.0);


// set up devices
void setup() {
#ifdef SERIAL_DEBUG
  // Set up serial port for debugging
  Serial.begin(115200); // set up serial port for debugging
  delay(10000); // don't spinlock waiting for serial in case it's not connected
  //while (!Serial);  // wait for serial access object to be ready
  //Serial.println("AsteriaAerospace.com 'Gideon Mk 2' Avionics Data Logger");
#endif

  // Set LED pins to output for debugging/status
  pinMode(RED_LED_PORT, OUTPUT);
  pinMode(GREEN_LED_PORT, OUTPUT);

  // Turn both LEDs off to start
  digitalWrite(GREEN_LED_PORT, LOW);
  digitalWrite(RED_LED_PORT, LOW);

  // early radio setup
  #ifdef ENABLE_RADIO
  setupRadioearly();
  #endif // ENABLE_RADIO

  // setup IMU/INS
  #ifdef ENABLE_INS
  setupINS();
  #endif // ENABLE_INS

  // setup Barometric Pressure/temp sensor
  #ifdef ENABLE_BMP
  setupBMP();
  #endif // ENABLE_BMP
  
  // set up data logging to SD card
  #ifdef ENABLE_SD
  setupSD();
  #endif // ENABLE_SD

  // setup GPS
  #ifdef ENABLE_GPS
  setupGPS();
  #endif // ENABLE_GPS

#ifdef SERIAL_DEBUG
  printCSVHeader(Serial);
#endif

#ifdef ENABLE_SD
  startSD();
#endif ENABLE_SD

}




void readAndReportTime(Stream& outputFile) {
  Time = millis() / 1000.0;
  outputFile.print(Time);
  outputFile.print(", ");
}


bool isRadioConfigured = false;

void loop() {

  #ifdef ENABLE_BATTERY
  batVoltage = checkBatteryVoltage();
  #endif // ENABLE_BATTERY

  #ifdef ENABLE_RADIO
  if(!isRadioConfigured)
  { // turn on and configure radio
    //Serial.println("Starting Radio"); // new line
    setupRadio();
    isRadioConfigured = true;
    //Serial.println("Radio Started"); // new line
  }
  #endif // ENABLE_RADIO

  unsigned char dataBuffer[11]; 
  #ifdef ENABLE_BMP
  loopBMPfirst();
  #endif // ENABLE_BMP
  #ifdef ENABLE_INS
  loopINSfirst();
  #endif // ENABLE_INS
  #ifdef ENABLE_GPS
  bool packetready = loopGPSfirst(dataBuffer, batVoltage);
  if(isGPSFixAcquired())
  {
      digitalWrite(GREEN_LED_PORT, HIGH);   // turn the GREEN LED on to indicate GPS fix (HIGH is the voltage level) 
  }
  #endif // ENABLE_GPS
  #ifdef ENABLE_RADIO
  loopRadiofirst();
  #endif // ENABLE_RADIO
  
  #ifdef ENABLE_SD
  // print a CSV row to file
  if (dataLogFile) {
    readAndReportTime(dataLogFile);
    #ifdef ENABLE_BMP
    loopBMPsecond();
    #endif // ENABLE_BMP
    #ifdef ENABLE_INS
    loopINSsecond();
    #endif // ENABLE_INS
    loopSDfirst();
  }
  #endif // ENABLE_SD


#ifdef SERIAL_DEBUG
  // print a CSV row to Serial
  //readAndReportTime(Serial);
  #ifdef ENABLE_BMP
  reportBMP(Serial);
  #endif // ENABLE_BMP
  #ifdef ENABLE_INS
  reportIMU(Serial);
  #endif // ENABLE_INS
  //Serial.println(""); // new line
#endif

#ifdef ENABLE_RADIO
if(isRadioConfigured)
{
  if(packetready)
    {
      bool useBoost = false;
      if(radioPacketCount % 100)
      {
        #ifdef USE_RADIO_BOOST
        useBoost = true;
        #endif // USE_RADIO_BOOST
      }
    loopRadiosecond(dataBuffer, useBoost);
    radioPacketCount++;
    //Serial.print("VBat: " ); Serial.println(batVoltage);
    } // if
}
#endif // ENABLE_RADIO

}
