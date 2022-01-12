

#define RADIO_PULSE_LED_TRANSMIT

#include <SPI.h>
#include <LoRa.h>

// from https://blog.devmobile.co.nz/2018/09/23/adafruit-feather-m0-rfm95-lora-radio-payload-addressing-client/
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 3;         // change for your board; must be a hardware interrupt pin
const int ledPin = 13;

// format of 11 byte data packet time, x, y, altitude
//12345678901
//tttxxxyyyaa

void setupRadioearly(void)
{
  #ifdef RADIO_PULSE_LED_TRANSMIT
  pinMode(ledPin, OUTPUT); // to blink when transmitting
  #endif // RADIO_PULSE_LED_TRANSMIT

  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  
}


void setupRadio(void)
{
  LoRa.begin(915E6);
  LoRa.setSpreadingFactor(10); // should go to 10, 370.7ms
  LoRa.setPreambleLength(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  LoRa.setTxPower(19); // these can go to 20, but only for 1% duty cycle

}

uint8_t radioPacketCounter(0);

void loopRadiofirst(void)
{
  // nothing to do here
}

void loopRadiosecond(unsigned char dataBuffer[])
{
      //Serial.println("Starting send"); // new line

  #ifdef RADIO_PULSE_LED_TRANSMIT
  digitalWrite(ledPin, HIGH);   // turn the RED LED on (HIGH is the voltage level) 
  #endif // RADIO_PULSE_LED_TRANSMIT

  LoRa.beginPacket();
  //LoRa.write(radioPacketCounter++);
  LoRa.write(&dataBuffer[0], 11);

  LoRa.endPacket(true);
  #ifdef RADIO_PULSE_LED_TRANSMIT
  digitalWrite(ledPin, LOW);    // turn the RED LED off by making the voltage LOW
  #endif // RADIO_PULSE_LED_TRANSMIT

      //Serial.println("Sent"); // new line

  
}
