// rf95_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#include <SPI.h>
#include <RH_RF95.h>

#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95 

void setup() 
{
    pinMode(13, OUTPUT);
  //Serial.begin(9600);
  //while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    {
    Serial.println("init failed");
    }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

   // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(20, false);
  rf95.setPreambleLength(8);
  //rf95.setPayloadCRC(true);
  rf95.spiWrite(RH_RF95_REG_39_SYNC_WORD,0x12); // set RFM SyncWord 0x12
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); // Sf4096 = sf12



}

uint8_t counter(0);

void loop()
{
  //Serial.println("Sending to rf95_server");
  // Send a message
  uint8_t data[] = "0";
  data[0] = counter++;
    digitalWrite(13, HIGH);   // turn the RED LED on (HIGH is the voltage level) 

  rf95.send(data, sizeof(data));
  
  rf95.waitPacketSent();
     digitalWrite(13, LOW);    // turn the RED LED off by making the voltage LOW

  delay(1000);
}
