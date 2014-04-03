SFE_CC3000_Library
==============

Arduino library for the TI CC3000 WiFi [Shield](https://www.sparkfun.com/products/12071) and [Breakout Board](https://www.sparkfun.com/products/12072).

Known Issues
------------

The following are known problems with the library and are being actively worked on:

* SD Card libraries causes the CC3000 library to hang (when used together).
* Static IP is not working at this time (only DHCP works).
* HTTP POST requests work but do not print the whole server response to the console.
* The library does not work with any Teensey boards. Only ATmega328-based Arduinos have been tested.

Getting Started
---------------

* Download the Git repository as a ZIP ("Download ZIP" button)
* Unzip
* Copy the entire directory (SFE_CC3000_Library) to <Arduino installation directory>/libraries
* Open the Arduino program
* Select File -> Examples -> SFE_CC3000_Library -> WebClient
* Change ap_ssid and ap_password to match your WiFi's SSID and password
* If your WiFi's security is anything other than WPA2, change ap_security to one of WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA, or WLAN_SEC_WPA2
* Plug in your Arduino and CC3000 board
* Go to Tools -> Board and select your Arduino board
* Go to Tools -> Serial Port and select the COM port of your Arduino board
* Click "Upload"
* Go to Tools -> Serial Monitor
* Ensure the baud rate is set at 115200 baud
* The program should connect and print out the HTML of [www.example.com](http://www.example.com)!

Version History
---------------

**v1.0**

* Initial release
* Manual connection implemented
* SmartConfig and FastConnect implemented
* Ping implemented
* TCP connections implemented
* Users can perform HTTP GET and HTTP POST

License
-------

This code is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.