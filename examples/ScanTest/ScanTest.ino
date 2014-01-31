/* 
 01-05-2014
 SparkFun Electronics 2014
 Shawn Hymel
 
 This code is public domain but you buy me a beer if you use this 
 and we meet someday (Beerware license).
 
 Description:
 
 Performs a scan of all SSIDs as seen by the CC3000 and prints the
 access points' information to the Serial Monitor. No WiFi
 connections are made.
 
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
 
#include <SPI.h>
#include <SFE_CC3000.h>

// Pins
#define CC3000_INT      2   // Needs to be an interrupt pin (D2/D3)
#define CC3000_EN       7   // Can be any digital pin
#define CC3000_CS       10  // Preferred is pin 10 on Uno

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);

void setup() {

  int i;
  AccessPointInfo ap_info;
  
  // Initialize Serial port
  Serial.begin(115200);
  Serial.println();
  Serial.println("---------------------------");
  Serial.println("SparkFun CC3000 - Scan Test");
  Serial.println("---------------------------");
  
  // Initialize CC3000 (configure SPI communications)
  if ( wifi.init() ) {
    Serial.println("CC3000 initialization complete");
  } else {
    Serial.println("Something went wrong during CC3000 init!");
  }
  
  // Perform scan of nearby WAPs
  Serial.println("Scanning APs. Waiting for scan to complete.");
  if ( wifi.scanAccessPoints(4000) != true ) {
    Serial.println("Error scanning APs");
  }
  
  // Iterate through available WAPs and print their information
  Serial.println("Access Points found:");
  Serial.println();
  while ( wifi.getNextAccessPoint(ap_info) ) {
    Serial.print("SSID: ");
    Serial.println(ap_info.ssid);
    Serial.print("MAC address: ");
    for ( i = 0; i < BSSID_LENGTH; i++ ) {
      if ( ap_info.bssid[i] < 0x10 ) {
        Serial.print("0");
      }
      Serial.print(ap_info.bssid[i], HEX);
      if ( i < BSSID_LENGTH - 1 ) {
        Serial.print(":");
      }
    }
    Serial.println();
    Serial.print("RSSI: ");
    Serial.println(ap_info.rssi, DEC);
    Serial.print("Security: ");
    switch(ap_info.security_mode) {
      case WLAN_SEC_UNSEC:
        Serial.println("Unsecured");
        break;
      case WLAN_SEC_WEP:
        Serial.println("WEP");
        break;
      case WLAN_SEC_WPA:
        Serial.println("WPA");
        break;
      case WLAN_SEC_WPA2:
        Serial.println("WPA2");
        break;
      default:
        break;
    }
    Serial.println();
  }

  // Done!
  Serial.println("Finished scan test");
  
}

void loop() {
  
  // Do nothing
  delay(1000);
  
}