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
#include "utility/nvmem.h"
#include "utility/wlan.h"

/* Global variables */
uint8_t g_int_pin;
uint8_t g_int_num;
uint8_t g_en_pin;
uint8_t g_cs_pin;
#if (DEBUG == 1)
volatile unsigned int g_debug_interrupt;
#endif
volatile unsigned long ulSmartConfigFinished;
volatile unsigned long ucStopSmartConfig;
volatile unsigned long ulCC3000Connected;
volatile unsigned long ulCC3000DHCP;
volatile unsigned long ulCC3000DHCP_configured;
volatile unsigned long OkToDoShutDown;
 
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
    
    /* Initialize access point scan variables */
    num_access_points_ = 0;
    access_point_count_ = 0;

    /* Set pin definitions */
    g_int_pin = int_pin;
    g_en_pin = en_pin;
    g_cs_pin = cs_pin;
#if (DEBUG == 1)
    g_debug_interrupt = 0xFFFF;
#endif
    
    /* Initialize status global variables */
    ulSmartConfigFinished = 0;
    ucStopSmartConfig = 0;
    ulCC3000Connected = 0;
    ulCC3000DHCP = 0;
    ulCC3000DHCP_configured = 0;
    OkToDoShutDown = 0;

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
    pinMode(g_cs_pin, OUTPUT);
    digitalWrite(g_en_pin, LOW);
    digitalWrite(g_cs_pin, LOW);
    
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
    
    /* CC3000 needs a delay before starting WLAN or it gets stuck sometimes */
    delay(100);
    
    /* Start CC3000 - asserts enable pin and blocks until init is complete */
    wlan_start(0);
    
    is_initialized_ = true;

    return true;
}

/**
 * @brief Reads the firmware version from the CC3000
 *
 * @param fw_ver firmware version in 2 bytes. [0] is major and [1] is minor
 * @return True is firmware could be read from the CC3000. False otherwise.
 */
bool SFE_CC3000::getFirmwareVersion(unsigned char *fw_ver)
{
	/* If CC3000 is not initialized, return false. */
	if (!is_initialized_) {
        return false;
    }
	
	/* Read firmware version from the CC3000 */
	if (nvmem_read_sp_version(fw_ver) != CC3000_SUCCESS) {
        return false;
    }
	
	return true;
}

/**
 * @brief Reads the MAC address from the CC3000
 *
 * @param mac_addr six char buffer containing the MAC address as a return value
 * @return True if MAC address could be read from the CC3000. False otherwise.
 */
bool SFE_CC3000::getMacAddress(unsigned char *mac_addr)
{
	/* If CC3000 is not initialized, return false. */
	if (!is_initialized_) {
        return false;
    }
    
    /* Read MAC address from the CC3000 */
    if (nvmem_get_mac_address(mac_addr) != CC3000_SUCCESS) {
        return false;
    }
    
    return true;
}

/**
 * @brief Scans area for access points. Blocks operation to allow for scan.
 *
 * To scan for APs, first call scanAccessPoints() with an appropriate scan time
 * (recommended scan_time = 4000ms). Create an AccessPointInfo struct and pass
 * that to getNextAccessPoint(). Continue to call getNextAccessPoint() until it
 * returns false (there are no more APs to scan).
 *
 * @param scan_time time to scan networks in milliseconds
 * @return True if scan succeeded. False otherwise.
 */
bool SFE_CC3000::scanAccessPoints(unsigned int scan_time)
{
    int i;
    unsigned long channel_timeouts[SCAN_NUM_CHANNELS];
    
    /* If CC3000 is not initialized, return false. */
	if (!is_initialized_) {
        return false;
    }
    
    /* Create channel interval list for AP scanning */
    for (i = 0; i < SCAN_NUM_CHANNELS; i++) {
        channel_timeouts[0] = SCAN_CHANNEL_TIMEOUT;
    }
    
    /* Setup access point scan */
    if (wlan_ioctl_set_scan_params( scan_time, 
                                    SCAN_MIN_DWELL_TIME,
                                    SCAN_MAX_DWELL_TIME,
                                    SCAN_NUM_PROBE_REQS,
                                    SCAN_CHANNEL_MASK,
                                    SCAN_RSSI_THRESHOLD,
                                    SCAN_NSR_THRESHOLD,
                                    SCAN_DEFAULT_TX_POWER,
                                    channel_timeouts ) != CC3000_SUCCESS) {
        return false;
    }

    /* Wait for scan to complete */
    delay(scan_time + 500);
    
    /* Re-initialize AP counters */
    num_access_points_ = 0;
    access_point_count_ = 0;
    
    /* Get first scan result in order to obtain the total number of APs */
    if (wlan_ioctl_get_scan_results(0, (unsigned char *)&ap_scan_result_) != 
                                                            CC3000_SUCCESS ){
        return false;
    }
    num_access_points_ = ap_scan_result_.num_networks;
    
    /* Stop scan */
    if (wlan_ioctl_set_scan_params( 0, 
                                    SCAN_MIN_DWELL_TIME,
                                    SCAN_MAX_DWELL_TIME,
                                    SCAN_NUM_PROBE_REQS,
                                    SCAN_CHANNEL_MASK,
                                    SCAN_RSSI_THRESHOLD,
                                    SCAN_NSR_THRESHOLD,
                                    SCAN_DEFAULT_TX_POWER,
                                    channel_timeouts ) != CC3000_SUCCESS) {
        return false;
    }

    return true;
}

/**
 * @brief Fills out AP info struct with next access data. 
 *
 * To scan for APs, first call scanAccessPoints() with an appropriate scan time
 * (recommended scan_time = 4000ms). Create an AccessPointInfo struct and pass
 * that to getNextAccessPoint(). Continue to call getNextAccessPoint() until it
 * returns false (there are no more APs to scan).
 *
 * @param ap_info struct containing information about the next AP
 * @return True if next AP obtained. False if no more APs available.
 */
bool SFE_CC3000::getNextAccessPoint(AccessPointInfo &ap_info)
{
   /* If CC3000 is not initialized, return false. */
	if (!is_initialized_) {
        return false;
    }
    
    /* If results are invalid (e.g. no results), return false */
    if (!ap_scan_result_.is_valid) {
        return false;
    }
    
    /* If we have exhausted all of the networks to list, return false */
    if (access_point_count_ >= num_access_points_) {
        return false;
    }
    
    /* Fill out AP info with last AP surveyed */
    ap_info.rssi = ap_scan_result_.rssi;
    ap_info.security_mode = ap_scan_result_.security_mode;
    strncpy(ap_info.ssid, 
            (char *)ap_scan_result_.ssid, 
            ap_scan_result_.ssid_length);
    ap_info.ssid[ap_scan_result_.ssid_length] = '\0';
    memcpy(ap_info.bssid, ap_scan_result_.bssid, BSSID_LENGTH);
    
    /* Get next set of results */
    if (wlan_ioctl_get_scan_results(0, (unsigned char *)&ap_scan_result_) != 
                                                            CC3000_SUCCESS ) {
        return false;
    }
    
    /* Increment AP counter */
    access_point_count_++;
    
    return true;
}

/**
 * @brief Connects to a WAP using the given SSID and password
 *
 * @param ssid the SSID for the wireless network
 * @param security type of security for the network
 * @param password optional ASCII password if connecting to a secured AP
 * @param timeout optional argument to set the timeout in ms. 0 = no timeout.
 * @return True if connected to wireless network. False otherwise.
 */
bool SFE_CC3000::connect(   char *ssid, 
                            unsigned int security, 
                            char *password,
                            unsigned int timeout)
{

    unsigned long time;

    /* If CC3000 is not initialized, return false. */
	if (!is_initialized_) {
        return false;
    }
    
    /* If already connected, return false. */
    if (getDHCPStatus()) {
        return false;
    }
    
    /* If security mode is not a predefined type, return false. */
    if ( !( security == WLAN_SEC_UNSEC ||
            security == WLAN_SEC_WEP ||
            security == WLAN_SEC_WPA ||
            security == WLAN_SEC_WPA2) ) {
        return false;
    }
    
    /* Set connection profile to manual (no fast or auto connect) */
    if (wlan_ioctl_set_connection_policy(0, 0, 0) != CC3000_SUCCESS) {
        return false;
    }
    
    /* Connect to the given access point*/
    time = millis();
    while (getDHCPStatus() == false) {
    
        /* Attempt to connect to an AP */
        delay(10);
        if (security == WLAN_SEC_UNSEC) {
#if (DEBUG == 1)
            Serial.println("Connecting to unsecured WiFi");
#endif
            if (wlan_connect(   WLAN_SEC_UNSEC, 
                                ssid, 
                                strlen(ssid), 
                                0, 
                                0, 
                                0) == CC3000_SUCCESS) {
                break;
            }
        } else {
#if (DEBUG == 1)
            Serial.println("Connecting to secured WiFi");
#endif
            if (wlan_connect(   security, 
                                ssid, 
                                strlen(ssid), 
                                0, 
                                (unsigned char*)password, 
                                strlen(password)) == CC3000_SUCCESS) {
                break;
            }
        }
        
        /* Check against timeout. Return if out of time. */
        if (timeout != 0) {
            if ( (millis() - time) > timeout ) {
                return false;
            }
        }
    }
    
#if (DEBUG == 1)
    Serial.println("Waiting for DHCP");
#endif
    
    /* Wait for DHCP */
    while (getDHCPStatus() == false) {
        if (timeout != 0) {
            if ( (millis() - time) > timeout ) {
#if (DEBUG == 1)
                Serial.println("Error: Timed out (waiting for DHCP)");
#endif
                return false;
            }
        }
    }
    
#if (DEBUG == 1)
    Serial.println("DHCP Obtained");
#endif

    return true;
}

/**
 * @brief Returns the status of DHCP
 *
 * @return True if DHCP has assigned an IP address. False otherwise.
 */
bool SFE_CC3000::getDHCPStatus() {
    if (ulCC3000DHCP == 1) {
        return true;
    }
    
    return false;
}