#include "Adafruit_BMP3XX.h"
#include <SD.h>

// Config for Barometric pressure/temp sensor
#define SEALEVELPRESSURE_HPA (1013.25)

// instantiate program object for BMP sensor on I2C bus
Adafruit_BMP3XX bmp;

extern File dataLogFile;

void setupBMP(void)
{
    // seup BMP pressure sensor
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
#ifdef SERIAL_DEBUG
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
#endif
    while (1);
  }

    // Set up BMP with oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


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


void loopBMPfirst(void)
{
  readBMP();

}

void loopBMPsecond(void)
{
  reportBMP(dataLogFile);
}
