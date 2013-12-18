/**
 * @file	SFE_CC3000.cpp
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

#include <Arduino.h>
 
#include "SFE_CC3000.h"
#include "SFE_CC3000_SPI.h"
#include "debug.h"
 
 /**
  * @brief Constructor - Instantiates SFE_CC3000 object
  *
  * @param int_pin pin needed for MCU interrupt
  * @param en_pin pin used for CC3000 enable
  * @param cs_pin pin for SPI chip select
  */
SFE_CC3000::SFE_CC3000(uint8_t int_pin, uint8_t en_pin, uint8_t cs_pin)
{
    int_pin_ = int_pin;
    en_pin_ = en_pin;
    cs_pin_ = cs_pin;
    is_initialized_ = false;
}

/**
 * @brief Destructor
 */
SFE_CC3000::~SFE_CC3000()
{

}

/**
 * @brief Configure SPI for MCU to CC3000
 *
 * @return True if SPI initialization completed successfully. False otherwise.
 */
bool SFE_CC3000::init()
{

#if (DEBUG == 1)
    Serial.println("Initializing CC3000");
#endif

    /* Check if CC3000 SPI is already initialized */
    if (is_initialized_) {
        return true;
    }
    
    /* Set interrupt pin to input */
    
    /* Set CC3000 enable pin to output and initialize low */
    
    /* Set chip select pin to output and initialize high */
    
    /* Setup SPI */
    
    /* Initialize CC3000 library - provide callback definitions */
    
    /* Start CC3000 - asserts enable pin and blocks until init is complete */
    
    is_initialized_ = true;

    return true;
}

/**
 * @brief Connects to a WAP using the given SSID and password
 *
 * @param ssid the SSID for the wireless network
 * @param password ASCII password for the wireless network
 * @param sec type of security for the network
 * @return True if connected to wireless network. False otherwise.
 */
bool SFE_CC3000::connect(const char *ssid, const char *password, uint8_t sec)
{
    return true;
}