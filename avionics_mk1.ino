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
//#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO08x.h>


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
    
  // Set up BMP with oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

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

void readAndReportIMU(void) {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

  case SH2_ACCELEROMETER:
    Serial.print("Accelerometer - x: ");
    Serial.print(sensorValue.un.accelerometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.accelerometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.accelerometer.z);
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    Serial.print("Gyro - x: ");
    Serial.print(sensorValue.un.gyroscope.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gyroscope.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gyroscope.z);
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    Serial.print("Magnetic Field - x: ");
    Serial.print(sensorValue.un.magneticField.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.magneticField.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.magneticField.z);
    break;
  case SH2_LINEAR_ACCELERATION:
    Serial.print("Linear Acceration - x: ");
    Serial.print(sensorValue.un.linearAcceleration.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.linearAcceleration.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.linearAcceleration.z);
    break;
  case SH2_GRAVITY:
    Serial.print("Gravity - x: ");
    Serial.print(sensorValue.un.gravity.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gravity.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gravity.z);
    break;
  case SH2_ROTATION_VECTOR:
    Serial.print("Rotation Vector - r: ");
    Serial.print(sensorValue.un.rotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.rotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.rotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.rotationVector.k);
    break;
  case SH2_GEOMAGNETIC_ROTATION_VECTOR:
    Serial.print("Geo-Magnetic Rotation Vector - r: ");
    Serial.print(sensorValue.un.geoMagRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.geoMagRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.geoMagRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.geoMagRotationVector.k);
    break;

  case SH2_GAME_ROTATION_VECTOR:
    Serial.print("Game Rotation Vector - r: ");
    Serial.print(sensorValue.un.gameRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.gameRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.gameRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.gameRotationVector.k);
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

  case SH2_RAW_ACCELEROMETER:
    Serial.print("Raw Accelerometer - x: ");
    Serial.print(sensorValue.un.rawAccelerometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.rawAccelerometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.rawAccelerometer.z);
    break;
  case SH2_RAW_GYROSCOPE:
    Serial.print("Raw Gyro - x: ");
    Serial.print(sensorValue.un.rawGyroscope.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.rawGyroscope.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.rawGyroscope.z);
    break;
  case SH2_RAW_MAGNETOMETER:
    Serial.print("Raw Magnetic Field - x: ");
    Serial.print(sensorValue.un.rawMagnetometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.rawMagnetometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.rawMagnetometer.z);
    break;
  }
}

void readAndReportBMP(void){
    if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Print data to serial port
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.print(" *C");
  Serial.print(" (");
  Serial.print((bmp.temperature * 9/5) + 32);
  Serial.println("F)");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print(" m");
  Serial.print(" (");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084);
  Serial.println("ft)");

  Serial.println();
}

void loop() {
  readAndReportIMU();
  //readAndReportBMP();
  
  delay(20);
}
