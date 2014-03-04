/****************************************************************
BoardTest.ino
CC3000 Board Test
Shawn Hymel @ SparkFun Electronics
November 26, 2014
https://github.com/sparkfun/SFE_CC3000_Library

Performs an initialization and reads the CC3000's MAC address and
firmware version. The MAC address and firmware version are
displayed to the Serial Monitor. No WiFi connections are made.

Hardware Connections:
 
 Uno Pin    CC3000 Board    Function
 
 +5V        VCC or +5V      5V
 GND        GND             GND
 2          INT             Interrupt
 7          EN              WiFi Enable
 10         CS              SPI Chip Select
 11         MOSI            SPI MOSI
 12         MISO            SPI MISO
 13         SCK             SPI Clock

Resources:
Include SPI.h and SFE_CC3000.h

Development environment specifics:
Written in Arduino 1.0.5
Tested with Arduino UNO R3

This code is beerware; if you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please
buy us a round!

Distributed as-is; no warranty is given.
****************************************************************/
 
#include <SPI.h>
#include <SFE_CC3000.h>

// Pins
#define CC3000_INT      2   // Needs to be an interrupt pin (D2/D3)
#define CC3000_EN       7   // Can be any digital pin
#define CC3000_CS       10  // Preferred is pin 10 on Uno

// Constants
#define FW_VER_LEN      2   // Length of firmware version in bytes
#define MAC_ADDR_LEN    6   // Length of MAC address in bytes

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);

void setup() {

  int i;
  unsigned char fw_ver[FW_VER_LEN];
  unsigned char mac_addr[MAC_ADDR_LEN];
  
  // Initialize Serial port
  Serial.begin(115200);
  Serial.println();
  Serial.println("----------------------------");
  Serial.println("SparkFun CC3000 - Board Test");
  Serial.println("----------------------------");
  
  // Initialize CC3000 (configure SPI communications)
  if ( wifi.init() ) {
    Serial.println("CC3000 initialization complete");
  } else {
    Serial.println("Something went wrong during CC3000 init!");
  }
  
  // Read and display CC3000 firmware version
  if ( wifi.getFirmwareVersion(fw_ver) ) {
    Serial.print("Firmware version: ");
    Serial.print(fw_ver[0], DEC);
    Serial.print(".");
    Serial.print(fw_ver[1], DEC);
    Serial.println();
  } else {
    Serial.println("Could not read firmware version from CC3000");
  }
  
  // Read and display CC3000 MAC address
  if ( wifi.getMacAddress(mac_addr) ) {
    Serial.print("MAC address: ");
    for ( i = 0; i < MAC_ADDR_LEN; i++ ) {
      if ( mac_addr[i] < 0x10 ) {
        Serial.print("0");
      }
      Serial.print(mac_addr[i], HEX);
      if ( i < MAC_ADDR_LEN - 1 ) {
        Serial.print(":");
      }
    }
    Serial.println();
  } else {
    Serial.println("Could not read MAC address from CC3000");
  } 
  
  // Done!
  Serial.println("Finished board test");
  
}

void loop() {
  
  // Do nothing
  delay(1000);
  
}