#include <Adafruit_H3LIS331.h>

Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

void setupINS(void)
{
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
}

void loopINSfirst(void)
{
   readIMU();
}

void loopINSsecond(void)
{
   reportIMU(dataLogFile);
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
