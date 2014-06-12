/****************************************************************
 * WebClientSD.ino
 * CC3000 WebClient with SD Card Test
 * Shawn Hymel @ SparkFun Electronics
 * March 27, 2014
 * https://github.com/sparkfun/SFE_CC3000_Library
 * 
 * Manually connects to a WiFi network and performs an HTTP GET
 * request on a web page. Saves the contents of the site to an SD
 * card. Prints the page contents our from the SD card.
 * 
 * IMPORTANT: SD cards use 3.3V logic, so a 5V - 3.3V level shifter
 * is required if coming from a 5V Arduino.
 * 
 * The security mode is defined by one of the following:
 * WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA, WLAN_SEC_WPA2
 * 
 * Hardware Connections:
 * 
 * Uno Pin    CC3000 Board    SD Card     Function
 * 
 * +5V        VCC or +5V                  5V
 * +3.3V      -               4 (3.3V)    3.3V
 * GND        GND             6 (GND)     GND
 * 2          INT             -           Interrupt
 * 7          EN              -           WiFi Enable
 * 8          -               2 (CS)      SD Chip Select
 * 10         CS              -           WiFi Chip Select
 * 11         MOSI            3 (MOSI)    SPI MOSI
 * 12         MISO            7 (MISO)    SPI MISO
 * 13         SCK             5 (SCK)     SPI Clock
 * 
 * Resources:
 * Include SD.h, SPI.h, SFE_CC3000.h, and SFE_CC3000_Client.h
 * 
 * Development environment specifics:
 * Written in Arduino 1.0.5
 * Tested with Arduino UNO R3
 * 
 * This code is beerware; if you see me (or any other SparkFun 
 * employee) at the local, and you've found our code helpful, please
 * buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ****************************************************************/

#include <SD.h>
#include <SPI.h>
#include <SFE_CC3000.h>
#include <SFE_CC3000_Client.h>

// Pins
#define CC3000_INT      2   // Needs to be an interrupt pin (D2/D3)
#define CC3000_EN       7   // Can be any digital pin
#define CC3000_CS       10  // Preferred is pin 10 on Uno
#define SD_CS           8   // Chip select for SD card

// Connection info data lengths
#define IP_ADDR_LEN     4   // Length of IP address in bytes

// Constants
char ap_ssid[] = "SSID";                  // SSID of network
char ap_password[] = "PASSWORD";          // Password of network
unsigned int ap_security = WLAN_SEC_WPA2; // Security of network
unsigned int timeout = 30000;             // Milliseconds
char server[] = "www.example.com";        // Remote host site
char filename[] = "example.txt";          // File name on SD card

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);
SFE_CC3000_Client client = SFE_CC3000_Client(wifi);
File sd_file;

void setup() {

  ConnectionInfo connection_info;
  char c;
  int i;

  // Initialize Serial port
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("-----------------------------"));
  Serial.println(F("SparkFun CC3000 - WebClientSD"));
  Serial.println(F("-----------------------------"));

  // Initialize SD card
  pinMode(SD_CS, OUTPUT);
  if ( !SD.begin(SD_CS) ) {
    Serial.println(F("Error: Could not initialize SD card"));
    return;
  } 
  else {
    Serial.println(F("SD card initialization complete"));
  }

  // Initialize CC3000 (configure SPI communications)
  if ( wifi.init() ) {
    Serial.println(F("CC3000 initialization complete"));
  } 
  else {
    Serial.println(F("Something went wrong during CC3000 init!"));
  }

  // Connect using DHCP
  Serial.print(F("Connecting to SSID: "));
  Serial.println(ap_ssid);
  if(!wifi.connect(ap_ssid, ap_security, ap_password, timeout)) {
    Serial.println(F("Error: Could not connect to AP"));
  }

  // Gather connection details and print IP address
  if ( !wifi.getConnectionInfo(connection_info) ) {
    Serial.println(F("Error: Could not get connection details"));
  } 
  else {
    Serial.print(F("IP Address: "));
    for (i = 0; i < IP_ADDR_LEN; i++) {
      Serial.print(connection_info.ip_address[i]);
      if ( i < IP_ADDR_LEN - 1 ) {
        Serial.print(".");
      }
    }
    Serial.println();
  }

  // Make a TCP connection to remote host
  Serial.print(F("Performing HTTP GET of: "));
  Serial.println(server);
  if ( !client.connect(server, 80) ) {
    Serial.println(F("Error: Could not make a TCP connection"));
    return;
  }
  
  // If file on SD already exists, delete it
  if ( SD.exists(filename) ) {
    Serial.print(filename);
    Serial.println(F(" exists. Removing it."));
    if ( !SD.remove(filename) ) {
      Serial.println(F("Error: Could not remove file"));
    }
  }

  // Open a file on the SD card to write to
  sd_file = SD.open(filename, FILE_WRITE);
  if (sd_file) {
    Serial.print(F("Writing web page to "));
    Serial.println(filename);
  } 
  else {
    Serial.println(F("Error: Could not open file for writing"));
    return;
  }

  // Make a HTTP GET request
  client.println(F("GET /index.html HTTP/1.1"));
  client.print(F("Host: "));
  client.println(server);
  client.println(F("Connection: close"));
  client.println();

  // Wait for response to GET
  while ( !client.available() ) {
    if ( !client.connected() ) {
      Serial.println(F("Error: Server closed connection early"));
      return;
    }
  }

  // Print web page to file
  while ( client.available() ) {
    c = client.read();
    if (sd_file) {
      sd_file.print(c);
    }
  }

  // Close file
  if (sd_file) {
    sd_file.close();
  }

  // Wait for server to disconnect
  while ( client.connected() ) {
    delay(1);
  }

  // Close socket
  if ( !client.close() ) {
    Serial.println(F("Error: Could not close socket"));
  }

  // Disconnect WiFi
  if ( !wifi.disconnect() ) {
    Serial.println(F("Error: Could not disconnect from AP"));
  }

  // Re-open file and print web page from file to console
  sd_file = SD.open(filename, FILE_READ);
  if (sd_file) {
    Serial.print(F("Reading web page from "));
    Serial.println(filename);
    Serial.println();
    while ( sd_file.available() ) {
      Serial.write(sd_file.read());
    }
    sd_file.close();
    Serial.println();
  } 
  else {
    Serial.println(F("Error: Could not open file for reading"));
  }

  // Finished!
  Serial.println(F("Finished WebClientSD test"));
}

void loop() {

  // Do nothing
  delay(1000);  
}