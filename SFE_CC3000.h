/**
 * @file	SFE_CC3000.h
 * @brief 	Library for the SparkFun CC3000 shield and breakout boards
 * @author	Shawn Hymel (SparkFun Electronics)
 * 
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 * 
 * This library interfaces the TI CC3000 to Arduino over SPI. The library relies
 * on the Arduino built-in SPI commands. To use the library, instantiate an
 * SFE_CC3000 object, call the init() function, and then call connect() with the
 * necessary connection details.
 */

#ifndef SFE_CC3000_H
#define SFE_CC3000_H
 
#include <Arduino.h>
 
//#include "SFE_CC3000_SPI.h"
 
class SFE_CC3000 {
    // ***TODO: Put in struct?
    uint8_t pin_int;
    uint8_t cc3000_en;
    uint8_t cc3000_cs;
public:
    SFE_CC3000(uint8_t int_pin, uint8_t en_pin, uint8_t cs_pin);
    ~SFE_CC3000();
    bool init();
    bool connect(const char *ssid, const char *password, uint8_t sec);
private:
};