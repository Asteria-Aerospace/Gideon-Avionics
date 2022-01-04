

#define RADIO_PULSE_LED_TRANSMIT

#include <SPI.h>
#include <LoRa.h>

// from https://blog.devmobile.co.nz/2018/09/23/adafruit-feather-m0-rfm95-lora-radio-payload-addressing-client/
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 3;         // change for your board; must be a hardware interrupt pin
const int ledPin = 13;

void setupRadio(void)
{
  #ifdef RADIO_PULSE_LED_TRANSMIT
  pinMode(ledPin, OUTPUT); // to blink when transmitting
  #endif // RADIO_PULSE_LED_TRANSMIT

  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  LoRa.begin(915E6);
  LoRa.setSpreadingFactor(20); //
  LoRa.setPreambleLength(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();

}

uint8_t radioPacketCounter(0);

void loopRadiofirst(void)
{
  // nothing to do here
}

void loopRadiosecond(void)
{
  #ifdef RADIO_PULSE_LED_TRANSMIT
  digitalWrite(ledPin, HIGH);   // turn the RED LED on (HIGH is the voltage level) 
  #endif // RADIO_PULSE_LED_TRANSMIT

  LoRa.beginPacket();
  LoRa.write(radioPacketCounter++);

  LoRa.endPacket();
  #ifdef RADIO_PULSE_LED_TRANSMIT
  digitalWrite(ledPin, LOW);    // turn the RED LED off by making the voltage LOW
  #endif // RADIO_PULSE_LED_TRANSMIT

  
}
