/*
 * Interfacing the following to get power output:
 * - iLabs Challenger 2040 Wifi
 * - RFM01S RF Module (433MHz tuned variant)
 * - Efergy Engage sender (433.535Mhz)
 * 
 * Notes:
 * - Efergy data is 64 bits at non-standard 433.5Mhz
 * - Cannot use the 16-bit RFM FIFO: power data is 24bits
 * - RFM goes into non-FIFO mode using DATA/nFFS as output
 * - RFM digital filter is a digital output, OOK is analog
 * - SPI is broken on Challenger 2040 Wifi: SDI swapped for SDA
 * - No need for pull-up resistors
 */

#include <SPI.h>

// These need to be changed according to the setup
#define MAINS_VOLTAGE 230
#define nSEL_PIN D10
#define DATA_PIN D11

// Use SDA, as SDI is broken on Challenger 2040 Wifi
#define SDI_PIN  PIN_WIRE0_SDA

// RFM Definitions
#define RFM_CONFIG      0x897D // 433, batt/wake off, 12pF, 67Khz b/w, CLK off
#define RFM_FREQUENCY   0xA586 // 433.535MHz (efergy specific)
#define RFM_DATA_FILTER 0xC46C // Fast lock, Digital filter
#define RFM_RECEIVER    0xC049 // Rx:Enable, RSSI:-79dB
#define RFM_DATA_RATE   0xC822 // 10142bps 
#define RFM_AFC_MODE    0xC627 // +7/-8 range, hi-acc, output reg, on
#define RFM_LOW_DUTY    0xCC0E // DEFAULT (OFF)
#define RFM_BATT_CLOCK  0xC200 // DEFAULT (OFF)
#define RFM_OUTPUT_FIFO 0xCE84 // DEFAULT but with FIFO disabled

// Microsecond definitions
#define PREAMBLE_LOW_MAX 1950
#define PREAMBLE_LOW_MIN 1875
#define PREAMBLE_HIGH_MAX 525
#define PREAMBLE_HIGH_MIN 475
#define PULSE_MAX 175
#define PULSE_MID 100
#define PULSE_MIN 10
#define WAITTIME 10 // us between bit polls

// Millisecond definitions
#define WAKETIME 50 // ms between expected data packets

#define BITCOUNT 64

// Processing states
enum { LISTEN, PREAMBLE, PAYLOAD };
uint8_t phase = LISTEN;

// Variables that capture states
uint8_t bits, lastValue = 0;
uint32_t lastTime = 0;

// Holds the data received
struct packet { 
  // Align a raw stream of bytes with a structured data type
  union {
    // Push bits into this
    uint8_t raw[8]; 
    // Read data from this
    struct {
      // Static
      uint8_t hdr; // Byte 0
      uint16_t dev; // Byte 1+2

      // Byte 3 - Reverse order as RP2040 is little-endian
      unsigned padding:4; // 0x0F
      unsigned ival:2;    // 0x30
      unsigned batt:1;    // 0x40
      unsigned learn:1;   // 0x80

      // Ampere readings
      uint8_t amp_msb; // Byte 4 
      uint8_t amp_lsb; // Byte 5
      uint8_t amp_exp; // Byte 6

      // Checksum
      uint8_t crc;     // Byte 7
    }__attribute__((packed));
  };

};
struct packet pkt;

// An SPI command is two bytes long
uint16_t spi_command(uint16_t input) {
  uint16_t status;

  digitalWrite(nSEL, LOW);
  status = SPI.transfer16(input);
  digitalWrite(nSEL, HIGH);

  return status;
}

void rfm_init(void) {
  delay(1000);
  // Setup sequence
  spi_command(RFM_CONFIG);   
  spi_command(RFM_OUTPUT_FIFO);
  spi_command(RFM_FREQUENCY);
  spi_command(RFM_BATT_CLOCK);
  spi_command(RFM_DATA_RATE);
  spi_command(RFM_RECEIVER);
  spi_command(RFM_DATA_FILTER);
  spi_command(RFM_AFC_MODE);
  spi_command(RFM_LOW_DUTY);
}

void processPacket(void) {    
  uint8_t crc = 0;
  char msg[128];

  // Calculate the checksum
  for (uint8_t i=0;i<7; i++) {
    crc += pkt.raw[i];
  }

  // If the header starts 0000 and the checksum is good
  if (((pkt.hdr & 0xf0) == 0x00) && (crc == pkt.crc)) {
    // Interval is either 0=10 secs, 1=15 secs or 2=20 secs
    uint8_t ival = (pkt.ival + 2 ) * 5;
    
    // amps / current = value-bytes * exp-byte^2 * 32768
    // kilowatt hours = amps * voltage / 1000 
    float kwh = (float)(pkt.amp_msb << 8 | pkt.amplsb) * (1 << pkt.amp_exp) / (1 << 15) * MAINS_VOLTAGE / 1000;    
    sprintf(msg, "{\"uptime\":%.3f,\"dev\":%d,\"learn\":%d,\"battery\":%d,\"interval\":%d,\"kwh\":%.2f}", (float)millis() / 1000, pkt.dev, pkt.learn, pkt.batt, ival, kwh);

    Serial.println(msg);
    
    // Go to sleep until the next packet is expected
    delay((ival * 1000) - WAKETIME);
  } else {
    Serial.println("{ \"error\": \"Header or CRC error\"");
  }
}

void setup() {
  // cu -s 115200 -l /dev/ttyACM0
  Serial.begin(115200);
  delay(5000);
  Serial.println("Serial console online");

  // Setup pins
  pinMode(DATA, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(nSEL, OUTPUT);
  digitalWrite(nSEL, HIGH);
  
  // SPI Setup with RFM01S
  SPI.setRX(SDI_PIN);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  Serial.println("SPI connection open");
  
  rfm_init();
  Serial.println("RFM01S radio online");
}

void loop() {
  uint32_t time = micros();
  uint8_t value = digitalRead(DATA);

  // If there's a change on DATA
  if (value != lastValue) {
    uint32_t gap = time - lastTime;

    // Based on the phase, what do we do...
    switch (phase) {
      case LISTEN:
        // Preamble pt1 - sustained low then rising edge
        if (value && (gap > PREAMBLE_LOW_MIN) && (gap < PREAMBLE_LOW_MAX)) {
          // Could be preamble, so check next pulse
          phase = PREAMBLE;
        }
        break;
        
      case PREAMBLE:
        // Preamble pt2 - sustained high then falling edge
        if (!value && (gap > PREAMBLE_HIGH_MIN) && (gap < PREAMBLE_HIGH_MAX)) {
          // We are good to process incoming data
          phase = PAYLOAD;
          digitalWrite(PIN_LED, HIGH);
          bits = 0;
        } else {
          // False alarm, go back to listening
          phase = LISTEN;
        }
        break;
        
      case PAYLOAD:
        // Data collection phase  
        if((gap > PULSE_MIN) && (gap < PULSE_MAX)) {
          // Only look for a rising edge
          if(value) {
            if ((gap > PULSE_MID)) {
              // Long 0 = logical 0 = shift
              pkt.raw[bits/8] = (pkt.raw[bits/8] << 1);             
            } else {
              // Short 0 = logical 1 = shift + 1
              pkt.raw[bits/8] = (pkt.raw[bits/8] << 1) + 1;
            }
            bits++;
          }
          // Do nothing with a falling edge
        } else if ((bits >= BITCOUNT)) {
          // If we have enough bits, process it
          digitalWrite(PIN_LED, LOW);
          phase = LISTEN;
          processPacket();
        } else {
          // If nothing, do nothing
          digitalWrite(PIN_LED, LOW);
          phase = LISTEN;
        }
        break;
        
    }
    // Data changed, so record the state
    lastValue = value;
    lastTime = time;
    
    // Wait a short period
    delayMicroseconds(WAITTIME);
  }
}
