These are some basic Arduino examples for the iLabs Challenger 2040 Wifi.

This uses the Arduino Pico library for RP2040 boards:
https://arduino-pico.readthedocs.io/en/latest/index.html

Note: This board has an issue where SDI does not function in SPI mode.

Examples are:
- Simple SPI loopback test (SDA <-> SDO)
- Wifi connectivity using standard WPA-PSK
- Receive data from an Efergy Engage dongle using an RFM01
-- Uses SPI to set the RFM into Digital OOK mode
-- No use of FIFO
-- Reads edges on the DATA pin
![RFM01S + Challenger 2040 Breadboard Schematic](/rfm01s-challenger2040.png)
