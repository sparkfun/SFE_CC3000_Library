/**
 * @file	SFE_CC3000.cpp
 * @brief 	Library for the SparkFun CC3000 shield and breakout boards
 * @author	Shawn Hymel (SparkFun Electronics)
 * @author  Revisions by Jacob Rosenthal (https://github.com/jacobrosenthal)
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
#include "utility/hci.h"
#include "utility/netapp.h"
#include "utility/nvmem.h"
#include "utility/socket.h"
#include "utility/wlan.h"

/* Global variables */
uint8_t g_int_pin;
uint8_t g_int_num;
uint8_t g_en_pin;
uint8_t g_cs_pin;
bool g_socket_connected;
uint8_t g_saved_data_mode;
uint8_t g_saved_bit_order;
uint8_t g_saved_clock_div;
#if (DEBUG == 1)
volatile long g_debug_interrupt;
#endif
volatile unsigned long ulSmartConfigFinished;
volatile unsigned long ucStopSmartConfig;
volatile unsigned long ulCC3000Connected;
volatile unsigned long ulCC3000DHCP;
volatile unsigned long ulCC3000DHCP_configured;
volatile unsigned long OkToDoShutDown;
netapp_pingreport_args_t g_ping_report = {0};
 
 /**
  * @brief Constructor - Instantiates SFE_CC3000 object
  *
  * @param[in] int_pin pin needed for MCU interrupt
  * @param[in] en_pin pin used for CC3000 enable
  * @param[in] cs_pin pin for SPI chip select
  */
SFE_CC3000::SFE_CC3000(uint8_t int_pin, uint8_t en_pin, uint8_t cs_pin)
{
    /* Set initialization state */
    is_initialized_ = false;
    
    /* Initialize access point scan variables */
    num_access_points_ = 0;
    access_point_count_ = 0;
    
    /* Initialize status global variables */
    ulSmartConfigFinished = 0;
    ucStopSmartConfig = 0;
    ulCC3000Connected = 0;
    ulCC3000DHCP = 0;
    ulCC3000DHCP_configured = 0;
    OkToDoShutDown = 0;
    g_socket_connected = false;
#if (DEBUG == 1)
    g_debug_interrupt = 0;
#endif

    /* Set pin definitions */
    g_int_pin = int_pin;
    g_en_pin = en_pin;
    g_cs_pin = cs_pin;

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
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || \
    defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    
    switch (g_int_pin) {
        case 2:
            g_int_num = 0;
            break;
        case 3:
            g_int_num = 1;
            break;
        case 18:
            g_int_num = 5;
            break;
        case 19:
            g_int_num = 4;
            break;
        case 20:
            g_int_num = 3;
            break;
        case 21:
            g_int_num = 2;
            break;
        default:
# if (DEBUG == 1)
            Serial.println("ERROR: Interrupt not pin 2, 3, 18, 19, 20, or 21");
# endif
            return false;
    }
#elif defined(__AVR_ATmega32U4__)
    switch (g_int_pin) {
        case 3:
            g_int_num = 0;
            break;
        case 2:
            g_int_num = 1;
            break;
        case 0:
            g_int_num = 2;
            break;
        case 1:
            g_int_num = 3;
            break;
        case 7:
            g_int_num = 4;
            break;
        default:
# if (DEBUG == 1)
            Serial.println("ERROR: Interrupt not pin 0, 1, 2, 3, or 7");
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
    
    /* Set up SPI, save default SPI parameters */
    SPI.begin();
    //SPI.setDataMode(SPI_MODE1);
    //SPI.setBitOrder(MSBFIRST);
    //SPI.setClockDivider(SPI_CLK_DIV);
    
    /* De-assert CS */
    digitalWrite(g_cs_pin, HIGH);
    
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
    
    /* Mask out non-required events */
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT);
    
    is_initialized_ = true;
    
    return true;
}

/**
 * @brief Reads the firmware version from the CC3000
 *
 * @param[out] fw_ver firmware version in 2 bytes. [0] is major and [1] is minor
 * @return True is firmware could be read from the CC3000. False otherwise.
 */
bool SFE_CC3000::getFirmwareVersion(unsigned char *fw_ver)
{
	/* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
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
 * @param[out] mac_addr six char buffer containing the MAC address
 * @return True if MAC address could be read from the CC3000. False otherwise.
 */
bool SFE_CC3000::getMacAddress(unsigned char *mac_addr)
{
	/* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
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
 * @param[in] scan_time time to scan networks in milliseconds
 * @return True if scan succeeded. False otherwise.
 */
bool SFE_CC3000::scanAccessPoints(unsigned int scan_time)
{
    int i;
    unsigned long channel_timeouts[SCAN_NUM_CHANNELS];
    
    /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
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
 * @param[out] ap_info struct containing information about the next AP
 * @return True if next AP obtained. False if no more APs available.
 */
bool SFE_CC3000::getNextAccessPoint(AccessPointInfo &ap_info)
{
   /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
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
 * @param[in] ssid the SSID for the wireless network
 * @param[in] security type of security for the network
 * @param[in] password optional ASCII password if connecting to a secured AP
 * @param[in] timeout optional time (ms) to wait before stopping. 0 = no timeout
 * @return True if connected to wireless network. False otherwise.
 */
bool SFE_CC3000::connect(   char *ssid, 
                            unsigned int security, 
                            char *password,
                            unsigned int timeout)
{
    unsigned long time;

    /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
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
    if (wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE) != 
                                                            CC3000_SUCCESS) {
        return false;
    }
    
    /* Connect to the given access point*/
    time = millis();
    while (getConnectionStatus() == false) {
    
        /* Attempt to connect to an AP */
        delay(10);
        if (security == WLAN_SEC_UNSEC) {

            if (wlan_connect(   WLAN_SEC_UNSEC, 
                                ssid, 
                                strlen(ssid), 
                                0, 
                                0, 
                                0) == CC3000_SUCCESS) {
                break;
            }
        } else {
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
    
    /* Wait for DHCP */
    while (getDHCPStatus() == false) {
        if (timeout != 0) {
            if ( (millis() - time) > timeout ) {
                return false;
            }
        }
    }

    /* Get connection information */
    netapp_ipconfig(&connection_info_);

    return true;
}

/**
 * @brief Begins SmartConfig. The user needs to run the SmartConfig phone app.
 *
 * @param[in] timeout optional time (ms) to wait before stopping. 0 = no timeout
 * @return True if connected to wireless network. False otherwise.
 */
bool SFE_CC3000::startSmartConfig(unsigned int timeout)
{
    char cc3000_prefix[] = {'T', 'T', 'T'};
    unsigned long time;
    
    /* Reset all global connection variables */
    ulSmartConfigFinished = 0;
	ulCC3000Connected = 0;
	ulCC3000DHCP = 0;
	OkToDoShutDown=0;
    
    /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
        return false;
    }
    
    /* Set connection profile to manual (no fast or auto connect) */
    if (wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE) != 
                                                            CC3000_SUCCESS) {
        return false;
    }
    
    /* Delete old connection profiles */
    if (wlan_ioctl_del_profile(255) != CC3000_SUCCESS) {
        return false;
    }
    
    /* Wait until CC3000 is disconnected */
    while (getConnectionStatus()) {
        delay(1);
    }
    
    /* Sets the prefix for SmartConfig. Should always be "TTT" */
    if (wlan_smart_config_set_prefix((char*)cc3000_prefix) != CC3000_SUCCESS) {
        return false;
    }
    
    /* Start the SmartConfig process */
    if (wlan_smart_config_start(0) != CC3000_SUCCESS) {
        return false;
    }
    
    /* Wait for SmartConfig to complete */
    time = millis();
    while (ulSmartConfigFinished == 0) {
        if (timeout != 0) {
            if ( (millis() - time) > timeout ) {
                return false;
            }
        }
    }
    
    /* Configure to connect automatically to AP from SmartConfig process */
    if (wlan_ioctl_set_connection_policy(DISABLE, DISABLE, ENABLE) != 
                                                            CC3000_SUCCESS) {
        return false;
    }
    
    /* Reset CC3000 */
    wlan_stop();
    delay(400);
    wlan_start(0);
    
    /* Wait for connection and DHCP-assigned IP address */
    while (getDHCPStatus() == false) {
        if (timeout != 0) {
            if ( (millis() - time) > timeout ) {
                return false;
            }
        }
    }
    
    /* If we make it this far, we need to tell the SmartConfig app to stop */
    mdnsAdvertiser(1, DEVICE_NAME, strlen(DEVICE_NAME));
    
    /* Get connection information */
    netapp_ipconfig(&connection_info_);

    return true;
}

/**
 * @brief Attempts to connect to network stored in memory.
 *
 * FastConnect attempts to connect to the WiFi network (AP) stored in non-
 * volatile memory. The user needs to run SmartConfig first in order to create
 * a network profile in memory before running FastConnect.
 *
 * @param[in] timeout optional time (ms) to wait before stopping. 0 = no timeout
 * @return True if connected to wireless network. False otherwise.
 */
bool SFE_CC3000::fastConnect(unsigned int timeout)
{
    unsigned long time;
    
    /* Reset all global connection variables */
    ulSmartConfigFinished = 0;
	ulCC3000Connected = 0;
	ulCC3000DHCP = 0;
	OkToDoShutDown=0;
    
    /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
        return false;
    }
    
    /* Set connection profile to auto-connect */
    if (wlan_ioctl_set_connection_policy(DISABLE, DISABLE, ENABLE) != 
                                                            CC3000_SUCCESS) {
        return false;
    }

    /* Wait for connection and DHCP-assigned IP address */
    time = millis();
    while (getDHCPStatus() == false) {
        if (timeout != 0) {
            if ( (millis() - time) > timeout ) {
                return false;
            }
        }
    }
    
    /* Get connection information */
    netapp_ipconfig(&connection_info_);

    return true;
}

/**
 * @brief Disconnects from the AP
 *
 * @return True if disconnected successfully. False otherwise.
 */
bool SFE_CC3000::disconnect()
{
    /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
        return false;
    }
    
    /* Attempt to disconnect from the network */
    if (wlan_disconnect() == CC3000_SUCCESS) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Looks up the IP address of a given hostname
 *
 * @param[in] hostname the name of the host or website (e.g. www.google.com)
 * @param[out] ip_address returned IP address of the hostname
 * @return True if lookup completed successfully. False otherwise.
 */
bool SFE_CC3000::dnsLookup(char *hostname, IPAddress *ip_address)
{
    unsigned long ret_ip_addr = 0;

    /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
        return false;
    }
    
    /* If not connected, return false. */
    if (!getConnectionStatus()) {
        return false;
    }
    
    /* If DHCP has not been assigned, return false. */
    if (!getDHCPStatus()) {
        return false;
    }

    /* Attempt to get IP address by hostname */
    if (!gethostbyname(hostname, strlen(hostname), &ret_ip_addr)) {
        return false;
    }

    *ip_address = IPAddress(
        (uint8_t)(ret_ip_addr >> 24) & 0xFF,
        (uint8_t)(ret_ip_addr >> 16) & 0xFF,
        (uint8_t)(ret_ip_addr >> 8) & 0xFF,
        (uint8_t)ret_ip_addr & 0xFF);

    return true;
}

/**
 * @brief Pings IP address [attempts] times and returns a ping report
 *
 * @param[in] ip_address the IP address to ping
 * @param[out] ping_report returned ping report with statistics
 * @param[in] attempts optional number of times to ping the address
 * @param[in] size optional size of ping buffer (up to 1400 bytes)
 * @param[in] timeout optional time to wait for ping response (milliseconds)
 * @return True if ping command succeeded. False otherwise.
 */
bool SFE_CC3000::ping(  IPAddress ip_address, 
                        PingReport &ping_report,
                        unsigned int attempts, 
                        unsigned int size, 
                        unsigned int timeout)
{
    
    unsigned long ip_addr;
    
    /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
        return false;
    }
    
    /* If not connected, return false. */
    if (!getConnectionStatus()) {
        return false;
    }
    
    /* If DHCP has not been assigned, return false. */
    if (!getDHCPStatus()) {
        return false;
    }
    
    /* Create unsigned long IP address out of char array */
    ip_addr = (unsigned long)ip_address[0] | 
                ((unsigned long)ip_address[1] << 8) |
                ((unsigned long)ip_address[2] << 16) |
                ((unsigned long)ip_address[3] << 24);
                
    /* Send pings and wait for report */
    if (netapp_ping_send(&ip_addr, attempts, size, timeout) != CC3000_SUCCESS) {
        return false;
    }
    delay((timeout * attempts) * 2);
    
    /* Copy output of ping report to return sruct */
    memcpy(&ping_report, &g_ping_report, sizeof(PingReport));
    
    return true;
}

/**
 * @brief Returns the status of DHCP
 *
 * @return True if DHCP has assigned an IP address. False otherwise.
 */
bool SFE_CC3000::getDHCPStatus() 
{
    if (ulCC3000DHCP == 1) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Returns the status of connection to an access point
 *
 * @return True if connected. False otherwise.
 */
bool SFE_CC3000::getConnectionStatus() 
{
    if (ulCC3000Connected == 1) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Returns the initialization state of the CC3000
 *
 * @return True if initialized. False otherwise.
 */
bool SFE_CC3000::getInitStatus() 
{
    return is_initialized_;
}

/**
 * @brief Fills out ConnectionInfo struct with AP connection details
 *
 * @param[out] info struct containing information about the AP connection
 * @return True if connection is valid. False otherwise.
 */
bool SFE_CC3000::getConnectionInfo(ConnectionInfo &info) 
{
    uint8_t i;
    uint8_t max;
    
    /* If CC3000 is not initialized, return false. */
	if (!getInitStatus()) {
        return false;
    }
    
    /* If not connected, return false. */
    if (!getConnectionStatus()) {
        return false;
    }
    
    /* If DHCP has not been assigned, return false. */
    if (!getDHCPStatus()) {
        return false;
    }
    
    /* Copy IP address to return struct. Reverse byte order. */
    max = sizeof(connection_info_.aucIP);
    for (i = 0; i < max; i++) {
        info.ip_address[i] = connection_info_.aucIP[max - 1 - i];
    }

    /* Copy subnet mask to return struct. Reverse byte order. */
    max = sizeof(connection_info_.aucSubnetMask);
    for (i = 0; i < max; i++) {
        info.subnet_mask[i] = connection_info_.aucSubnetMask[max - 1 - i];
    }
    
    /* Copy default gateway to return struct. Reverse byte order. */
    max = sizeof(connection_info_.aucDefaultGateway);
    for (i = 0; i < max; i++) {
        info.default_gateway[i] = connection_info_.aucDefaultGateway[max - 
                                                                        1 - i];
    }
    
    /* Copy DHCP server address to return struct. Reverse byte order. */
    max = sizeof(connection_info_.aucDHCPServer);
    for (i = 0; i < max; i++) {
        info.dhcp_server[i] = connection_info_.aucDHCPServer[max - 1 - i];
    }
    
    /* Copy DNS server address to return struct. Reverse byte order. */
    max = sizeof(connection_info_.aucDNSServer);
    for (i = 0; i < max; i++) {
        info.dns_server[i] = connection_info_.aucDNSServer[max - 1 - i];
    }
    
    /* Copy MAC address to return struct. Reverse byte order. */
    max = sizeof(connection_info_.uaMacAddr);
    for (i = 0; i < max; i++) {
        info.mac_address[i] = connection_info_.uaMacAddr[max - 1 - i];
    }

    /* Copy SSID to return struct. Keep byte order. */
    memcpy(info.ssid, connection_info_.uaSSID, 32);
    
    return true;
}