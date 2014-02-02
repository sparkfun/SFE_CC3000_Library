/* 
 02-01-2014
 SparkFun Electronics 2014
 Shawn Hymel
 
 This code is public domain but you buy me a beer if you use this 
 and we meet someday (Beerware license).
 
 Description:
 
 Connects to tha access point given by the SSID and password and
 waits for a DHCP-assigned IP address. Pings the give website or
 IP address and waits for a response.
 
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

// Connection info data lengths
#define IP_ADDR_LEN     4   // Length of IP address in bytes

// Constants
char ap_ssid[] = "TDG";
char ap_password[] = "97413quyrTTEW";
unsigned int ap_security = WLAN_SEC_WPA2;
unsigned int timeout = 30000;         // Milliseconds
char remote_host[] = "www.sparkfun.com";

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);

void setup() {
  
  ConnectionInfo connection_info;
  IPAddr ip_addr;
  int i;
  
  // Initialize Serial port
  Serial.begin(115200);
  Serial.println();
  Serial.println("---------------------------");
  Serial.println("SparkFun CC3000 - Ping Test");
  Serial.println("---------------------------");
  
  // Initialize CC3000 (configure SPI communications)
  if ( wifi.init() ) {
    Serial.println("CC3000 initialization complete");
  } else {
    Serial.println("Something went wrong during CC3000 init!");
  }

  // Connect and wait for DHCP-assigned IP address
  Serial.print("Connecting to: ");
  Serial.println(ap_ssid);
  if(!wifi.connect(ap_ssid, ap_security, ap_password, timeout)) {
    Serial.println("Error: Could not connect to AP");
  }
  
  // Gather connection details and print IP address
  /*if ( !wifi.getConnectionInfo(connection_info) ) {
    Serial.println("Error: Could not obtain connection details");
  } else {
    Serial.println("Connected!");
    Serial.print("IP Address: ");
    for (i = 0; i < IP_ADDR_LEN; i++) {
      Serial.print(connection_info.ip_address[i]);
      if ( i < IP_ADDR_LEN - 1 ) {
        Serial.print(".");
      }
    }
    Serial.println();
  }*/
  
  // Perform a DNS lookup to get the IP address of a host
  Serial.print("Looking up IP address of: ");
  Serial.println(remote_host);
  if ( !wifi.dnsLookup(remote_host, ip_addr) ) {
    Serial.println("Error: Could not lookup host by name");
  } else {
    Serial.print("IP address found: ");
    for (i = 0; i < IP_ADDR_LEN; i++) {
      Serial.print(ip_addr.address[i]);
      if ( i < IP_ADDR_LEN - 1 ) {
        Serial.print(".");
      }
    }
    Serial.println();
  }
  
  // Ping IP address of remote host
  
  // Disconnect
  if ( wifi.disconnect() ) {
    Serial.println("Disconnected");
  } else {
    Serial.println("Error: Could not disconnect from network");
  }
  
  // Done!
  Serial.println("Finished ping test");
  
}

void loop() {
  
  // Do nothing
  delay(1000);
  
}