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
 
#include "SFE_CC3000.h"
#include "SFE_CC3000_SPI.h"
#include "debug.h"

#define SPI_CLK_DIV             SPI_CLOCK_DIV2
 
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
    
    /* Initialize interrupt, CS, and enable pins */
    pinMode(int_pin_, INPUT);
    pinMode(en_pin_, OUTPUT);
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(en_pin_, LOW);
    digitalWrite(cs_pin_, LOW);
    
    /* Setup SPI */
    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLK_DIV);
    
    /* Initialize CC3000 library - provide callback definitions */
    wlan_init(  cc3000AsyncCallback,
                sendFirmwarePatches,
                sendDriverPatches,
                sendBootLoaderPatches,
                readWlanInterruptPin,
                enableWlanInterrupt
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

/**
 * @brief Asynchronous callback from CC3000 library - handles events
 *
 * @param lEventType Event type
 * @param data Pointer to the data
 * @param length length of data
 *
 * @return none
 */
void SFE_CC3000:cc3000AsyncCallback(    long lEventType, 
                                        char *data, 
                                        unsigned char length)
{
	/*if (lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE)
	{
		ulSmartConfigFinished = 1;
		ucStopSmartConfig     = 1;  
	}
	
	if (lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT)
	{
		ulCC3000Connected = 1;
		
		// Turn on the LED1
		turnLedOn(1);
	}
	
	if (lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
	{		
		ulCC3000Connected = 0;
		ulCC3000DHCP      = 0;
		ulCC3000DHCP_configured = 0;
		printOnce = 1;
		// Turn off the LED1
		turnLedOff(1);                                     
	}
	
	if (lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
	{
		// Notes: 
		// 1) IP config parameters are received swapped
		// 2) IP config parameters are valid only if status is OK, i.e. ulCC3000DHCP becomes 1
		
		// only if status is OK, the flag is set to 1 and the addresses are valid
		if ( *(data + NETAPP_IPCONFIG_MAC_OFFSET) == 0)
		{
			char *ccPtr;
			unsigned short ccLen;

			pucCC3000_Rx_Buffer[0] = 'I';
			pucCC3000_Rx_Buffer[1] = 'P';
			pucCC3000_Rx_Buffer[2] = ':';

			ccPtr = (char*)&pucCC3000_Rx_Buffer[3];

			ccLen = itoa(data[3], ccPtr);
			ccPtr += ccLen;
			*ccPtr++ = '.';
			ccLen = itoa(data[2], ccPtr);
			ccPtr += ccLen;
			*ccPtr++ = '.';
			ccLen = itoa(data[1], ccPtr);
			ccPtr += ccLen;
			*ccPtr++ = '.';
			ccLen = itoa(data[0], ccPtr);
			ccPtr += ccLen;
			*ccPtr++ = '\f';
			*ccPtr++ = '\r';
			*ccPtr++ = '\0';

			ulCC3000DHCP = 1;
		}
		else
			ulCC3000DHCP = 0;
		
	}
	
	if (lEventType == HCI_EVENT_CC3000_CAN_SHUT_DOWN)
	{
		OkToDoShutDown = 1;
	}*/
	
}