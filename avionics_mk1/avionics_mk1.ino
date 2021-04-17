/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <SD.h>

// Config for SD card
const int chipSelect = 4;
unsigned long lastFlushTimeMilliseconds = 0, flushRateMilliseconds = (5 * 1000); // five seconds

// Config for Barometric pressure/temp sensor
#define SEALEVELPRESSURE_HPA (1013.25)
// Config for BNO08X IMU
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;


// Config for diagnostic LEDs
#define GREEN_LED_PORT 8
#define RED_LED_PORT 13

// instantiate program object for BMP sensor on I2C bus
Adafruit_BMP3XX bmp;

// These variables accumulate the most current data as it arrives, for later reporting
double Time, Baro, TempC, TempF, BaroCal, AltiM, AltiF, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, LAccX, LAccY, LAccZ, GravX, GravY, GravZ, RotTheta, RotI, RotJ, RotK, GeoRotTheta, GeoRotI, GeoRotJ, GeoRotK, GameRotTheta, GameRotI, GameRotJ, GameRotK;

File dataLogFile;

// setup SD card IO
void setupSD(void) {
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  else {
    Serial.println("card initialized.");
    dataLogFile = SD.open("datalog.csv", FILE_WRITE);
  }
}

// set up devices
void setup() {
  // Set up serial port for debugging
  Serial.begin(115200); // set up serial port for debugging
  while (!Serial);  // wait for serial access object to be ready
  Serial.println("AsteriaAerospace.com 'Discovery' Avionics Data Logger");

  // Set LED pins to output for debugging/status
  pinMode(RED_LED_PORT, OUTPUT);
  pinMode(GREEN_LED_PORT, OUTPUT);

  // Turn both LEDs off to start
  digitalWrite(GREEN_LED_PORT, LOW);   
  digitalWrite(RED_LED_PORT, LOW);  

 // Initialize BNO08X!
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    } // while
  }
  Serial.println("BNO08x Found!");
  setReports(); // Chose what data we want to recieve from IMU
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // set up data logging to SD card
  setupSD();

  // Set up BMP with oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  printCSVHeader(Serial);
  if(dataLogFile) {
    printCSVHeader(dataLogFile);
  }
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
//  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
//    Serial.println("Could not enable gyroscope");
//  }
//  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
//    Serial.println("Could not enable magnetic field calibrated");
//  }
//  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
//    Serial.println("Could not enable linear acceleration");
//  }
//  if (!bno08x.enableReport(SH2_GRAVITY)) {
//    Serial.println("Could not enable gravity vector");
//  }
//  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
//    Serial.println("Could not enable rotation vector");
//  }
//  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
//    Serial.println("Could not enable geomagnetic rotation vector");
//  }
//  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
//    Serial.println("Could not enable game rotation vector");
//  }
//  // if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
//  //  Serial.println("Could not enable stability classifier");
//  // }
//  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
//    Serial.println("Could not enable raw accelerometer");
//  }
//  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
//    Serial.println("Could not enable raw gyroscope");
//  }
//  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
//    Serial.println("Could not enable raw magnetometer");
//  }
}

void printCSVHeader (Stream& outputFile){
  outputFile.print("Time, Baro, TempC, TempF, BaroCal, AltiM, AltiF, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, LAccX, LAccY, LAccZ, GravX, GravY, GravZ, RotTheta, RotI, RotJ, RotK, GeoRotTheta, GeoRotI, GeoRotJ, GeoRotK, GameRotTheta, GameRotI, GameRotJ, GameRotK");
}

void reportIMU(Stream& outputFile){
    outputFile.print(AccelX);
    outputFile.print(", ");
    outputFile.print(AccelY);
    outputFile.print(", ");
    outputFile.print(AccelZ);
    outputFile.print(", ");
    outputFile.print(GyroX);
    outputFile.print(", ");
    outputFile.print(GyroY);
    outputFile.print(", ");
    outputFile.print(GyroZ);
    outputFile.print(", ");
    outputFile.print(MagX);
    outputFile.print(", ");
    outputFile.print(MagY);
    outputFile.print(", ");
    outputFile.print(MagZ);
    outputFile.print(", ");
    outputFile.print(LAccX);
    outputFile.print(", ");
    outputFile.print(LAccY);
    outputFile.print(", ");
    outputFile.print(LAccZ);
    outputFile.print(", ");
    outputFile.print(GravX);
    outputFile.print(", ");
    outputFile.print(GravY);
    outputFile.print(", ");
    outputFile.print(GravZ);
    outputFile.print(", ");
    outputFile.print(RotTheta);
    outputFile.print(", ");
    outputFile.print(RotI);
    outputFile.print(", ");
    outputFile.print(RotJ);
    outputFile.print(", ");
    outputFile.print(RotK);
    outputFile.print(", ");
    outputFile.print(GeoRotTheta);
    outputFile.print(", ");
    outputFile.print(GeoRotI);
    outputFile.print(", ");
    outputFile.print(GeoRotJ);
    outputFile.print(", ");
    outputFile.print(GeoRotK);
    outputFile.print(", ");
    outputFile.print(GameRotTheta);
    outputFile.print(", ");
    outputFile.print(GameRotI);
    outputFile.print(", ");
    outputFile.print(GameRotJ);
    outputFile.print(", ");
    outputFile.print(GameRotK);
    outputFile.print(", ");
}

void readIMU(void) {

  if (bno08x.wasReset()) {
    //Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

// GameRotTheta, GameRotI, GameRotJ, GameRotK 
  switch (sensorValue.sensorId) {

  case SH2_ACCELEROMETER:
    AccelX = sensorValue.un.accelerometer.x;
    AccelY = sensorValue.un.accelerometer.y;
    AccelZ = sensorValue.un.accelerometer.z;
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    GyroX = sensorValue.un.gyroscope.x;
    GyroY = sensorValue.un.gyroscope.y;
    GyroZ = sensorValue.un.gyroscope.z;
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    MagX = sensorValue.un.magneticField.x;
    MagY = sensorValue.un.magneticField.y;
    MagZ = sensorValue.un.magneticField.z;
    break;
  case SH2_LINEAR_ACCELERATION:
    LAccX = sensorValue.un.linearAcceleration.x;
    LAccY = sensorValue.un.linearAcceleration.y;
    LAccZ = sensorValue.un.linearAcceleration.z;
    break;
  case SH2_GRAVITY:
    GravX = sensorValue.un.gravity.x;
    GravY = sensorValue.un.gravity.y;
    GravZ = sensorValue.un.gravity.z;
    break;
  case SH2_ROTATION_VECTOR:
    RotTheta = sensorValue.un.rotationVector.real;
    RotI = sensorValue.un.rotationVector.i;
    RotJ = sensorValue.un.rotationVector.j;
    RotK = sensorValue.un.rotationVector.k;
    break;
  case SH2_GEOMAGNETIC_ROTATION_VECTOR:
    GeoRotTheta = sensorValue.un.geoMagRotationVector.real;
    GeoRotI = sensorValue.un.geoMagRotationVector.i;
    GeoRotJ = sensorValue.un.geoMagRotationVector.j;
    GeoRotK = sensorValue.un.geoMagRotationVector.k;
    break;
  case SH2_GAME_ROTATION_VECTOR:
    GameRotTheta = sensorValue.un.gameRotationVector.real;
    GameRotI = sensorValue.un.gameRotationVector.i;
    GameRotJ = sensorValue.un.gameRotationVector.j;
    GameRotK = sensorValue.un.gameRotationVector.k;
    break;

//  case SH2_STABILITY_CLASSIFIER: {
//    Serial.print("Stability Classification: ");
//    sh2_StabilityClassifier_t stability = sensorValue.un.stabilityClassifier;
//    switch (stability.classification) {
//    case STABILITY_CLASSIFIER_UNKNOWN:
//      Serial.println("Unknown");
//      break;
//    case STABILITY_CLASSIFIER_ON_TABLE:
//      Serial.println("On Table");
//      break;
//    case STABILITY_CLASSIFIER_STATIONARY:
//      Serial.println("Stationary");
//      break;
//    case STABILITY_CLASSIFIER_STABLE:
//      Serial.println("Stable");
//      break;
//    case STABILITY_CLASSIFIER_MOTION:
//      Serial.println("In Motion");
//      break;
//    }
//    break;
//  }
//
//  case SH2_RAW_ACCELEROMETER:
//    Serial.print("Raw Accelerometer - x: ");
//    Serial.print(sensorValue.un.rawAccelerometer.x);
//    Serial.print(" y: ");
//    Serial.print(sensorValue.un.rawAccelerometer.y);
//    Serial.print(" z: ");
//    Serial.println(sensorValue.un.rawAccelerometer.z);
//    break;
//  case SH2_RAW_GYROSCOPE:
//    Serial.print("Raw Gyro - x: ");
//    Serial.print(sensorValue.un.rawGyroscope.x);
//    Serial.print(" y: ");
//    Serial.print(sensorValue.un.rawGyroscope.y);
//    Serial.print(" z: ");
//    Serial.println(sensorValue.un.rawGyroscope.z);
//    break;
//  case SH2_RAW_MAGNETOMETER:
//    Serial.print("Raw Magnetic Field - x: ");
//    Serial.print(sensorValue.un.rawMagnetometer.x);
//    Serial.print(" y: ");
//    Serial.print(sensorValue.un.rawMagnetometer.y);
//    Serial.print(" z: ");
//    Serial.println(sensorValue.un.rawMagnetometer.z);
//    break;
  }
}

void readAndReportTime(Stream& outputFile) {
  Time = millis() / 1000.0;
  outputFile.print(Time);
  outputFile.print(", ");
}

void reportBMP(Stream&outputFile){
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

void readBMP(void){
    if (! bmp.performReading()) {
    Serial.println("Failed to perform reading.");
    return;
  }

  // Store data to temp variables

  // Raw Pressure reading
  Baro = bmp.pressure / 100.0;

  // Celsius
  TempC = bmp.temperature;

  // Farenheit
  TempF = ( bmp.temperature * 9/5) + 32;

  AltiM = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  AltiF = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084;
}

void loop() {
  readBMP();
  readIMU();

  // print a CSV row to file
  if(dataLogFile) {
    readAndReportTime(dataLogFile);
    reportBMP(dataLogFile);
    reportIMU(dataLogFile);
    dataLogFile.println(""); // new line
  }
  
  // print a CSV row to Serial
  readAndReportTime(Serial);
  reportBMP(Serial);
  reportIMU(Serial);
  Serial.println(""); // new line

  // is it time to flush the writes to the SD card yet?
  unsigned long nowMillis = millis();
  if(nowMillis > lastFlushTimeMilliseconds + flushRateMilliseconds) {
    dataLogFile.flush();
    lastFlushTimeMilliseconds = nowMillis;
  }
  
  delay(20);
}
