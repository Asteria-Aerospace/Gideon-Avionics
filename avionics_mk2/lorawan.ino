

#define RADIO_PULSE_LED_TRANSMIT

#include <SPI.h>
#include <LoRa.h>

// from https://blog.devmobile.co.nz/2018/09/23/adafruit-feather-m0-rfm95-lora-radio-payload-addressing-client/
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 3;         // change for your board; must be a hardware interrupt pin
const int ledPin = 13;

// https://mylorawan.blogspot.com/2016/05/spread-factor-vs-payload-size-on-lora.html
// In essence, longer range is possible in Europe because of a higher permissible spreading factor. However, data throughput is generally higher in North America because of Europe’s duty cycle restriction, but keep in mind that using the highest spreading factor for North America (10) limits payload size to 11 bytes. Whereas in Europe the payload can be 51 bytes at the highest spreading factor (12).
// ISM Band 902-928 MHz
// TX Restriction 400ms tx time
// Payload sizes 11 – 242 bytes
// Spreading factors 7 – 10
// Data rates 1 – 12.5 kbps
// Max transmit power 21 dBm
// SF_10 125kHz 0.98 kbps 11 bytes

// https://medium.com/home-wireless/testing-lora-radios-with-the-limesdr-mini-part-2-37fa481217ff

// format of 11 byte data packet time, x, y, altitude
//12345678901
//STTXXXYYYAA

// S is composed of gps fix bit, 6 bits of battery level, one bit of time MSB

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

void loopRadiosecond(unsigned char dataBuffer[], bool boost)
{
      //Serial.println("Starting send"); // new line

if(boost)
{
  LoRa.setTxPower(20); // these can go to 20, but only for 1% duty cycle
}

  #ifdef RADIO_PULSE_LED_TRANSMIT
  digitalWrite(ledPin, HIGH);   // turn the RED LED on (HIGH is the voltage level) 
  #endif // RADIO_PULSE_LED_TRANSMIT

  LoRa.beginPacket();
  //LoRa.write(radioPacketCounter++);
  LoRa.write(&dataBuffer[0], 11);

  LoRa.endPacket(boost ? false : true); // wait for completion of boosted packets
  #ifdef RADIO_PULSE_LED_TRANSMIT
  digitalWrite(ledPin, LOW);    // turn the RED LED off by making the voltage LOW
  #endif // RADIO_PULSE_LED_TRANSMIT

if(boost)
{
  LoRa.setTxPower(19); // step back down to 19
}

      //Serial.println("Sent"); // new line

  
}
