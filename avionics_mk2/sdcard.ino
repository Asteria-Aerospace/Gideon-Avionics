#include <SD.h>

// Config for SD card
const int chipSelect = 4; // SD card SPI Chip Select is port 4, make sure this doesn't conflict with other SPI devices
unsigned long lastFlushTimeMilliseconds = 0, flushRateMilliseconds = (5 * 1000); // five seconds

File dataLogFile;

void printCSVHeader (Stream& outputFile) {
  outputFile.print("Time, Baro, TempC, TempF, BaroCal, AltiM, AltiF, AccelX, AccelY, AccelZ,");
  outputFile.println(""); // new line
}

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

void startSD(void)
{
    if (dataLogFile) {
    printCSVHeader(dataLogFile);
    digitalWrite(GREEN_LED_PORT, HIGH); // indicate we're writing data
  }
}

void loopSDfirst(void)
{
   dataLogFile.println(""); // new line
}

unsigned long nowMillis = millis();
void loopSDsecond(void)
{
    // is it time to flush the writes to the SD card yet?
  if (nowMillis > lastFlushTimeMilliseconds + flushRateMilliseconds) {
    digitalWrite(RED_LED_PORT, HIGH);
    dataLogFile.flush();
    digitalWrite(RED_LED_PORT, LOW);
    lastFlushTimeMilliseconds = nowMillis;
  }

}
