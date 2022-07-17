/*
 * Challenger 2040 Wifi + NeoPixel demo for Arduino IDE
 * - Connects to Wifi as a client, header creds
 * - Use the RGB of the NeoPixel to show status
 * 
 * References:
 * - WiFiEspAT Example: TemporaryWifiConnect.ino
 * - Adafruit NeoPixel Example: simple.ino
 */
 
// Both can be installed via Arduino Library Manager
#include <WiFiEspAT.h>
#include <Adafruit_NeoPixel.h>

// Need to define WIFI_HOST, WIFI_SSID, WIFI_PSK
#include "wifi_settings.h"

#define NEO_RED   0x0F0000
#define NEO_GREEN 0x000F00
#define NEO_BLUE  0x00000F

const char host[] = WIFI_HOST;
const char ssid[] = WIFI_SSID;
const char psk[] = WIFI_PSK;

Adafruit_NeoPixel pixels(1, NEOPIXEL, NEO_GRB + NEO_KHZ800);

void neopixelSet(uint32_t x) {
  uint32_t rgb = pixels.Color((x>>16) & 0xFF, (x>>8) & 0xFF, x & 0xFF);
  pixels.setPixelColor(0, rgb);
  pixels.show();
}

void setup() {
  Serial.begin(115200);

  // Use the NeoPixel to show the status
  pixels.begin();
  pixels.clear();
  
  // RED - Not ready
  neopixelSet(NEO_RED);

  delay(5000);

  // Wifi Setup
  Serial2.begin(115200);
  WiFi.init(Serial2, PIN_ESP_RST);
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFiEspAT: No module");
    while (1);
  }
  
  // Turn-off any on-boot wifi setup
  WiFi.disconnect();
  WiFi.setPersistent();
  WiFi.endAP();

  // Set the hostname
  WiFi.setHostname(host);
}

void loop() {
  // If the wireless isn't connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFiEspAt: Attempting connection");
    
    // BLUE - Ready,  not connected
    neopixelSet(NEO_BLUE);

    // Try a WPA connection
    WiFi.begin(ssid,psk);

    // If it's connected
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFiEspAt: Connection success");
      
      // GREEN - Ready, connected
      neopixelSet(NEO_GREEN);
    } else {
      Serial.println("WiFiEspAt: Connection failed");
    }
  } else {
    // If all is well, keep printing the IP
    Serial.print("WiFiEspAT: Connected on ");
    Serial.println(WiFi.localIP());
  }
  
  delay(10000);
}
