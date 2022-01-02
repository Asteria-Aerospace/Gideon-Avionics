// Asteria Aerospace Gideon Mk2 Arduino Feather M0 Avionics stack

// enable this to stream telemetry data to serial/USB.
// This is blocking and will stall operations if no USB/serial
// receiver is listening.
#define SERIAL_DEBUG 0

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>

// Config for SD card
const int chipSelect = 4; // SD card SPI Chip Select is port 4, make sure this doesn't conflict with other SPI devices
unsigned long lastFlushTimeMilliseconds = 0, flushRateMilliseconds = (5 * 1000); // five seconds

// Config for Barometric pressure/temp sensor
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_H3LIS331 lis = Adafruit_H3LIS331();


// Config for diagnostic LEDs
#define GREEN_LED_PORT 8
#define RED_LED_PORT 13

// instantiate program object for BMP sensor on I2C bus
Adafruit_BMP3XX bmp;

// These variables accumulate the most current data as it arrives, for later reporting
double Time, Baro, TempC, TempF, BaroCal, AltiM, AltiF, AccelX, AccelY, AccelZ;

File dataLogFile;

// setup SD card IO
void setupSD(void) {
#ifdef SERIAL_DEBUG
  Serial.print("Initializing SD card...");
#endif

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
#ifdef SERIAL_DEBUG
    Serial.println("Card failed, or not present");
#endif
  }
  else {
    char newfilename[50];
    sprintf(newfilename, "%04d.csv", millis());
    dataLogFile = SD.open(newfilename, FILE_WRITE);
#ifdef SERIAL_DEBUG
    Serial.println("card initialized. Filename:");
    Serial.println(newfilename);
#endif
  }
}

// set up devices
void setup() {
#ifdef SERIAL_DEBUG
  // Set up serial port for debugging
  Serial.begin(115200); // set up serial port for debugging
  delay(10000); // don't spinlock waiting for serial in case it's not connected
  //while (!Serial);  // wait for serial access object to be ready
  Serial.println("AsteriaAerospace.com 'Gideon Mk 2' Avionics Data Logger");
#endif

  // Set LED pins to output for debugging/status
  pinMode(RED_LED_PORT, OUTPUT);
  pinMode(GREEN_LED_PORT, OUTPUT);

  // Turn both LEDs off to start
  digitalWrite(GREEN_LED_PORT, LOW);
  digitalWrite(RED_LED_PORT, LOW);

  // Initialize IMU
  if (!lis.begin_I2C()) {
#ifdef SERIAL_DEBUG
    Serial.println("Failed to find LIS331 chip");
#endif
    while (1) {
      delay(10);
    } // while
  }
  
#ifdef SERIAL_DEBUG
  Serial.println("LIS331 Found!");
#endif
  lis.setRange(H3LIS331_RANGE_100_G); // 100G range
  lis.setDataRate(LIS331_DATARATE_50_HZ); // 50Hz update rate

  // seup BMP pressure sensor
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
#ifdef SERIAL_DEBUG
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
#endif
    while (1);
  }

  // set up data logging to SD card
  setupSD();

  // Set up BMP with oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
#ifdef SERIAL_DEBUG
  printCSVHeader(Serial);
#endif
  if (dataLogFile) {
    printCSVHeader(dataLogFile);
    digitalWrite(GREEN_LED_PORT, HIGH); // indicate we're writing data
  }
}


void printCSVHeader (Stream& outputFile) {
  outputFile.print("Time, Baro, TempC, TempF, BaroCal, AltiM, AltiF, AccelX, AccelY, AccelZ,");
  outputFile.println(""); // new line
}

void reportIMU(Stream& outputFile) {
  outputFile.print(AccelX);
  outputFile.print(", ");
  outputFile.print(AccelY);
  outputFile.print(", ");
  outputFile.print(AccelZ);
  outputFile.print(", ");
}

void readIMU(void) {

  sensors_event_t event;
  lis.getEvent(&event);
  
  AccelX = event.acceleration.x;
  AccelY = event.acceleration.y;
  AccelZ = event.acceleration.z;

}

void readAndReportTime(Stream& outputFile) {
  Time = millis() / 1000.0;
  outputFile.print(Time);
  outputFile.print(", ");
}

void reportBMP(Stream&outputFile) {
  // Raw Pressure reading
  outputFile.print(Baro);
  outputFile.print(", ");

  // Celsius
  outputFile.print(TempC);
  outputFile.print(", ");

  // Farenheit
  outputFile.print(TempF);
  outputFile.print(", ");

  // Barometric Calibration value
  outputFile.print(BaroCal);
  outputFile.print(", ");

  // Altitude Meters
  outputFile.print(AltiM);
  outputFile.print(", ");

  // Altitude Feet
  outputFile.print(AltiF);
  outputFile.print(", ");

}

void readBMP(void) {
  if (! bmp.performReading()) {
#ifdef SERIAL_DEBUG
    Serial.println("Failed to perform reading.");
#endif
    return;
  }

  // Store data to temp variables

  // Raw Pressure reading
  Baro = bmp.pressure / 100.0;

  // Celsius
  TempC = bmp.temperature;

  // Farenheit
  TempF = ( bmp.temperature * 9 / 5) + 32;

  AltiM = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  AltiF = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084;
}

void loop() {
  readBMP();
  readIMU();

  // print a CSV row to file
  if (dataLogFile) {
    readAndReportTime(dataLogFile);
    reportBMP(dataLogFile);
    reportIMU(dataLogFile);
    dataLogFile.println(""); // new line
  }

#ifdef SERIAL_DEBUG
  // print a CSV row to Serial
  readAndReportTime(Serial);
  reportBMP(Serial);
  reportIMU(Serial);
  Serial.println(""); // new line
#endif

  // is it time to flush the writes to the SD card yet?
  unsigned long nowMillis = millis();
  if (nowMillis > lastFlushTimeMilliseconds + flushRateMilliseconds) {
    digitalWrite(RED_LED_PORT, HIGH);
    dataLogFile.flush();
    digitalWrite(RED_LED_PORT, LOW);
    lastFlushTimeMilliseconds = nowMillis;
  }

  delay(20); // 50Hz
}
