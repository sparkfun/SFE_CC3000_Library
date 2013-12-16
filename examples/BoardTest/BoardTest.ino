/* 
 11-26-14
 SparkFun Electronics 2013
 Shawn Hymel
 
 This code is public domain but you buy me a beer if you use this 
 and we meet someday (Beerware license).
 
 Description:
 
 Performs an initialization and reads the CC3000's MAC address and
 firmware version. The MAC address and firmware version are
 displayed to the Serial Monitor.
 
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
 
 */
 
//#include <SPI.h>
#include <SFE_CC3000.h>

// Pins
#define CC3000_INT  2  // Needs to be an interrupt pin
#define CC3000_EN   7  // Can be any digital pin
#define CC3000_CS   10 // Preferred is pin 10 on Uno

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);

void setup() {
  
  // Initialize Serial port
  Serial.begin(9600);
  Serial.println("SparkFun CC3000 - Board Test");
  
  // Initialize CC3000 (configure SPI communications)
  wifi.init();
  
  // Read and display CC3000 firmware version
  
  // Read and display CC3000 MAC address
  
}

void loop() {
  
  // Do nothing
  delay(1000);
}