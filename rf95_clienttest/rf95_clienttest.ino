// rf95_clienttest.ino

#include <SPI.h>
#include <LoRa.h>

// from https://blog.devmobile.co.nz/2018/09/23/adafruit-feather-m0-rfm95-lora-radio-payload-addressing-client/
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 3;         // change for your board; must be a hardware interrupt pin

void setup() 
{
    pinMode(13, OUTPUT);
//  Serial.begin(9600);
//  while (!Serial);

//  Serial.println("LoRa Sender");

  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {
//    Serial.println("Starting LoRa failed!");
    while (1);
  }

//    Serial.println("LoRa begin successful");


  LoRa.setSpreadingFactor(20); //
  LoRa.setPreambleLength(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();

//    Serial.println("LoRa parameters set");
}

uint8_t counter(0);



void loop()
{
//    Serial.println("LoRa sending");
  // Send a message
  LoRa.beginPacket();
  uint8_t data[] = "0";
  LoRa.write(counter++);
    digitalWrite(13, HIGH);   // turn the RED LED on (HIGH is the voltage level) 

  LoRa.endPacket();
//    Serial.println("LoRa sent");
  
     digitalWrite(13, LOW);    // turn the RED LED off by making the voltage LOW

  delay(1000);
}
