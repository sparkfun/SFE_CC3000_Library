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
#include <SPI.h>
 
#include "common.h"
#include "SFE_CC3000.h"
#include "SFE_CC3000_Callbacks.h"
#include "SFE_CC3000_SPI.h"
#include "utility/wlan.h"

#define SPI_CLK_DIV             SPI_CLOCK_DIV2        

/* Global variables */
uint8_t g_int_pin;
uint8_t g_int_num;
uint8_t g_en_pin;
 
 /**
  * @brief Constructor - Instantiates SFE_CC3000 object
  *
  * @param int_pin pin needed for MCU interrupt
  * @param en_pin pin used for CC3000 enable
  * @param cs_pin pin for SPI chip select
  */
SFE_CC3000::SFE_CC3000(uint8_t int_pin, uint8_t en_pin, uint8_t cs_pin)
{

    /* Set initialization state */
    is_initialized_ = false;

    /* Set pin definitions */
    g_int_pin = int_pin;
    g_en_pin = en_pin;
    cs_pin_ = cs_pin;

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
    
    /* Determine available interrupt pins on supported microcontrollers */
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
    switch (g_int_pin) {
        case 2:
            g_int_num = 0;
            break;
        case 3:
            g_int_num = 1;
            break;
        default:
# if (DEBUG == 1)
        Serial.println("ERROR: Interrupt line not attached to pin 2 or 3");
# endif
        return false;
    }
#else
# if (DEBUG == 1)
        Serial.println("ERROR: Microcontroller not supported");
# endif
        return false;
#endif
    
    /* Initialize interrupt, CS, and enable pins */
    pinMode(g_int_pin, INPUT);
    pinMode(g_en_pin, OUTPUT);
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(g_en_pin, LOW);
    digitalWrite(cs_pin_, LOW);
    
    /* Setup SPI */
    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLK_DIV);
    
    /* Initialize CC3000 library - provide callback definitions */
    wlan_init(  cc3000AsyncCallback,
                sendFirmwarePatch,
                sendDriverPatch,
                sendBootLoaderPatch,
                readWlanInterruptPin,
                enableWlanInterrupt,
                disableWlanInterrupt,
                writeWlanPin);
    
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
 *
 * @return True if connected to wireless network. False otherwise.
 */
bool SFE_CC3000::connect(const char *ssid, const char *password, uint8_t sec)
{
    return true;
}

