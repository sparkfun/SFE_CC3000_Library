/****************************************************************
PingTest.ino
CC3000 Ping Test
Shawn Hymel @ SparkFun Electronics
February 1, 2014
https://github.com/sparkfun/SFE_CC3000_Library

Connects to the access point given by the SSID and password and
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

// Connection info data lengths
#define IP_ADDR_LEN     4   // Length of IP address in bytes

// Constants
char ap_ssid[] = "SSID";                  // SSID of network
char ap_password[] = "PASSWORD";          // Password of network
unsigned int ap_security = WLAN_SEC_WPA2; // Security of network
unsigned int timeout = 30000;             // Milliseconds
char remote_host[] = "www.sparkfun.com";  // Host to ping
unsigned int num_pings = 3;    // Number of times to ping

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);

void setup() {
  
  ConnectionInfo connection_info;
  IPAddress ip_addr;
  IPAddress remote_ip;
  PingReport ping_report = {0};
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
  if ( !wifi.getConnectionInfo(connection_info) ) {
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
  }
  
  // Perform a DNS lookup to get the IP address of a host
  Serial.print("Looking up IP address of: ");
  Serial.println(remote_host);
  if ( !wifi.dnsLookup(remote_host, &remote_ip) ) {
    Serial.println("Error: Could not lookup host by name");
  } else {
    Serial.print("IP address found: ");
    for (i = 0; i < IP_ADDR_LEN; i++) {
      Serial.print(remote_ip[i], DEC);
      if ( i < IP_ADDR_LEN - 1 ) {
        Serial.print(".");
      }
    }
    Serial.println();
  }
  
  // Ping IP address of remote host
  Serial.print("Pinging ");
  for (i = 0; i < IP_ADDR_LEN; i++) {
    Serial.print(remote_ip[i], DEC);
    if ( i < IP_ADDR_LEN - 1 ) {
      Serial.print(".");
    }
  }
  Serial.print(" ");
  Serial.print(num_pings, DEC);
  Serial.println(" times...");
  if ( !wifi.ping(remote_ip, ping_report, 3, 56, 1000) ) {
    Serial.println("Error: no ping response");
  } else {
    Serial.println("Pong!");
    Serial.println();
    Serial.print("Packets sent: ");
    Serial.println(ping_report.packets_sent);
    Serial.print("Packets received: ");
    Serial.println(ping_report.packets_received);
    Serial.print("Min round time (ms): ");
    Serial.println(ping_report.min_round_time);
    Serial.print("Max round time (ms): ");
    Serial.println(ping_report.max_round_time);
    Serial.print("Avg round time (ms): ");
    Serial.println(ping_report.avg_round_time);
    Serial.println();
  }
  
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