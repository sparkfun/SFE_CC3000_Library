/**
 * @file	SFE_CC3000_Callbacks.cpp
 * @brief 	Callback functions for the CC3000 library
 * @author	Shawn Hymel (SparkFun Electronics)
 * 
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 * 
 * These functions implement the required callbacks for the provided TI CC3000 
 * library. The Arduino library provides the functions' addresses so the TI
 * library can call them.
 */

#include <Arduino.h>

#include "common.h"
#include "SFE_CC3000_Callbacks.h"
#include "SFE_CC3000_SPI.h"
#include "utility/evnt_handler.h"
#include "utility/hci.h"

/**
 * @brief   Asynchronous callback from CC3000 library - handles events
 *
 * @param   lEventType Event type
 * @param   data Pointer to the data
 * @param   length length of data
 */
void cc3000AsyncCallback(long lEventType, char * data, unsigned char length)
{
  
	if (lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE)
	{
		ulSmartConfigFinished = 1;
		ucStopSmartConfig     = 1;  
	}
	
	if (lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT)
	{
		ulCC3000Connected = 1;
	}
	
	if (lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
	{		
		ulCC3000Connected = 0;
		ulCC3000DHCP      = 0;
		ulCC3000DHCP_configured = 0;   
	}
	
	if (lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
	{
		// Notes: 
		// 1) IP config parameters are received swapped
		// 2) IP config parameters are valid only if status is OK, i.e. ulCC3000DHCP becomes 1
		
		// only if status is OK, the flag is set to 1 and the addresses are valid
		if ( *(data + NETAPP_IPCONFIG_MAC_OFFSET) == 0)
		{
			ulCC3000DHCP = 1;
		}
		else
		{
			ulCC3000DHCP = 0;
		}
	}
	
	if (lEventType == HCI_EVENT_CC3000_CAN_SHUT_DOWN)
	{
		OkToDoShutDown = 1;
	}
    
    if (lEventType == HCI_EVNT_WLAN_ASYNC_PING_REPORT) 
    {
#if (DEBUG == 1)
    Serial.println("Ping report received");
#endif
        memcpy(&g_ping_report, data, length);
    }
    if (lEventType == HCI_EVNT_BSD_TCP_CLOSE_WAIT)
    {
    
#if (DEBUG == 1)
        Serial.println("BSD TCP Close Wait event");
#endif

    }
}

/**
 * @brief   This function provides a pointer to the firmware patch
 *
 * Since there is no patch in the host, it returns NULL.
 *
 * @param   Length pointer to the length
 *
 * @return  a pointer to the firmware patch
 */
char *sendFirmwarePatch(unsigned long *Length)
{
	*Length = 0;
	return NULL;
}

/**
 * @brief   This function provides a pointer to the driver patch
 *
 * Since there is no patch in the host, it returns NULL.
 *
 * @param   Length pointer to the length
 *
 * @return  a pointer to the driver patch
 */
char *sendDriverPatch(unsigned long *Length)
{
	*Length = 0;
	return NULL;
}

/**
 * @brief   This function provides a pointer to the bootloader patch
 *
 * Since there is no patch in the host, it returns NULL.
 *
 * @param   Length pointer to the length
 *
 * @return  a pointer to the bootloader patch
 */
char *sendBootLoaderPatch(unsigned long *Length)
{
	*Length = 0;
	return NULL;
}

/**
 * @brief   Reads the state of the CC3000 interrupt pin
 *
 * @return  the state of the CC3000 interrupt pin
 */
long readWlanInterruptPin()
{
    return digitalRead(g_int_pin);
}

/**
 * @brief   Enables CC3000 interrupt pin interrupts
 */
void enableWlanInterrupt()
{
    attachInterrupt(g_int_num, cc3000_ISR, FALLING);
}

/**
 * @brief   Disables CC3000 interrupt pin interrupts
 */
void disableWlanInterrupt()
{
    detachInterrupt(g_int_num);
}

/**
 * @brief   The TI library calls this to enable or disable the CC3000 EN pin
 *
 * @param   val The value to write to the EN pin (high or low)
 */
void writeWlanPin(unsigned char val)
{
    if (val) {
        digitalWrite(g_en_pin, HIGH);
    } else {
        digitalWrite(g_en_pin, LOW);
    }
}