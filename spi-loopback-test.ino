/*
 * Example to do a loopback test for SPI
 * 
 * Simply hook up SDI to SDO
 *
 * Warning:
 * This has a workaround with SDA - SDO for the iLabs Challenger 2040 Wifi
 */
#include <SPI.h>

void spiByte() {
  const uint8_t txByte = 0x61;
  uint8_t rxByte = 0;
  
  rxByte = SPI.transfer(txByte);
  
  Serial.print("Byte received ");
  Serial.println(rxByte, DEC);

  if (rxByte == txByte) {
    Serial.println("Byte validated");
  } else {
    Serial.println("Byte failed");
  }
}

void spiShort() {
  const uint16_t txShort = 0x6162;
  uint16_t rxShort = 0;
  
  rxShort = SPI.transfer16(txShort);
  
  Serial.print("Short received ");
  Serial.println(rxShort, DEC);

  if (rxShort == txShort) {
    Serial.println("Short validated");
  } else {
    Serial.println("Short failed");
  }
}

void setup() {
  // cu -s 115200 -l /dev/ttyACM0
  Serial.begin(115200);

  delay(5000);
  
  Serial.println("SPI test");

  // Bug in Challenger2 2040 SDI becomes SDA
  SPI.setRX(PIN_WIRE0_SDA);

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  spiByte();
  spiShort();
  SPI.endTransaction();
  SPI.end();
}

void loop() {
}
