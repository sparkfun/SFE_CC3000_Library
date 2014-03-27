/****************************************************************
WebClientSD.ino
CC3000 WebClient with SD Card Test
Shawn Hymel @ SparkFun Electronics
March 27, 2014
https://github.com/sparkfun/SFE_CC3000_Library

Manually connects to a WiFi network and performs an HTTP GET
request on a web page. Saves the contents of the site to an SD
card.

IMPORTANT: SD cards use 3.3V logic, so a 5V - 3.3V level shifter
is required if coming from a 5V Arduino.

Hardware Connections:
 
 Uno Pin    CC3000 Board    SD Card     Function
 
 +5V        VCC or +5V                  5V
 +3.3V      -               4 (3.3V)    3.3V
 GND        GND             6 (GND)     GND
 2          INT             -           Interrupt
 7          EN              -           WiFi Enable
 8          -               2 (CS)      SD Chip Select
 10         CS              -           WiFi Chip Select
 11         MOSI            3 (MOSI)    SPI MOSI
 12         MISO            7 (MISO)    SPI MISO
 13         SCK             5 (SCK)     SPI Clock

Resources:
Include SD.h, SPI.h, SFE_CC3000.h, and SFE_CC3000_Client.h

Development environment specifics:
Written in Arduino 1.0.5
Tested with Arduino UNO R3

This code is beerware; if you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please
buy us a round!

Distributed as-is; no warranty is given.
****************************************************************/

#include <SD.h>
#include <SPI.h>
#include <SFE_CC3000.h>
#include <SFE_CC3000_Client.h>

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
char server[] = "www.example.com";        // Remote host site

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);
SFE_CC3000_Client client = SFE_CC3000_Client(wifi);

void setup() {
  
  ConnectionInfo connection_info;
  int i;
  
  // Initialize Serial port
  Serial.begin(115200);
  Serial.println();
  Serial.println("-----------------------------");
  Serial.println("SparkFun CC3000 - WebClientSD");
  Serial.println("-----------------------------");
  
  // Initialize CC3000 (configure SPI communications)
  if ( wifi.init() ) {
    Serial.println("CC3000 initialization complete");
  } else {
    Serial.println("Something went wrong during CC3000 init!");
  }
  
  // Connect using DHCP
  Serial.print("Connecting to SSID: ");
  Serial.println(ap_ssid);
  if(!wifi.connect(ap_ssid, ap_security, ap_password, timeout)) {
    Serial.println("Error: Could not connect to AP");
  }
  
  // Gather connection details and print IP address
  if ( !wifi.getConnectionInfo(connection_info) ) {
    Serial.println("Error: Could not obtain connection details");
  } else {
    Serial.print("IP Address: ");
    for (i = 0; i < IP_ADDR_LEN; i++) {
      Serial.print(connection_info.ip_address[i]);
      if ( i < IP_ADDR_LEN - 1 ) {
        Serial.print(".");
      }
    }
    Serial.println();
  }
  
  // Make a TCP connection to remote host
  Serial.print("Performing HTTP GET of: ");
  Serial.println(server);
  if ( !client.connect(server, 80, TCP) ) {
    Serial.println("Error: Could not make a TCP connection");
  }
  
  // Make a HTTP GET request
  client.println("GET /index.html HTTP/1.1");
  client.print("Host: ");
  client.println(server);
  client.println("Connection: close");
  client.println();
  Serial.println();
}

void loop() {
  
  // If there are incoming bytes, print them
  if ( client.available() ) {
    char c = client.read();
    Serial.print(c);
  }
  
  // If the server has disconnected, stop the client and wifi
  if ( !client.connected() ) {
    Serial.println();
    
    // Close socket
    if ( !client.close() ) {
      Serial.println("Error: Could not close socket");
    }
    
    // Disconnect WiFi
    if ( !wifi.disconnect() ) {
      Serial.println("Error: Could not disconnect from network");
    }
    
    // Do nothing
    Serial.println("Finished WebClient test");
    while(true){
      delay(1000);
    }
  }
}