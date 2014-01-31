/* 
 01-30-2014
 SparkFun Electronics 2014
 Shawn Hymel
 
 This code is public domain but you buy me a beer if you use this 
 and we meet someday (Beerware license).
 
 Description:
 
 Connects to the access point given by the SSID and password and
 waits for a DHCP-assigned IP address. To use a static IP
 address, change the #define USE_DHCP from 1 to 0 and assign an
 IP address to static_ip_addr in the Constants section.
 
 The security mode is defined by one of the following:
 WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA, WLAN_SEC_WPA2
 
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

// IP address assignment method
#define USE_DHCP        1   // 0 = static IP, 1 = DHCP

// Constants
char ap_ssid[] = "sparkfun";
char ap_password[] = "sparkfun6175";
unsigned int ap_security = WLAN_SEC_WPA2;
unsigned int timeout = 10000;         // Milliseconds
//const char static_ip_addr[] = "0.0.0.0";

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);

void setup() {
  
  // Initialize Serial port
  Serial.begin(115200);
  Serial.println();
  Serial.println("---------------------------------");
  Serial.println("SparkFun CC3000 - Connection Test");
  Serial.println("---------------------------------");
  
  // Initialize CC3000 (configure SPI communications)
  if ( wifi.init() ) {
    Serial.println("CC3000 initialization complete");
  } else {
    Serial.println("Something went wrong during CC3000 init!");
  }

  // Connect using DHCP
#if (USE_DHCP == 1)
  Serial.println("Connecting to AP");
  if(!wifi.connect(ap_ssid, ap_security, ap_password, timeout)) {
    Serial.println("Error connecting to AP");
  }
#endif

  // Connect using static IP
#if (USE_DHCP == 0)
  // ***TODO: Connect using static IP
#endif
  
  // Done!
  Serial.println("Finished connection test");
  
}

void loop() {
  
  // Do nothing
  delay(1000);
  
}