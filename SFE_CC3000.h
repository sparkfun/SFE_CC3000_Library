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

/* Clock definition for SPI */
#define SPI_CLK_DIV             SPI_CLOCK_DIV2

/* Constants for AP scanning */
#define SCAN_MIN_DWELL_TIME     20              // Milliseconds
#define SCAN_MAX_DWELL_TIME     30              // Milliseconds
#define SCAN_NUM_PROBE_REQS     2
#define SCAN_CHANNEL_MASK       0x7FF
#define SCAN_RSSI_THRESHOLD     -80
#define SCAN_NSR_THRESHOLD      0
#define SCAN_DEFAULT_TX_POWER   205
#define SCAN_NUM_CHANNELS       16
#define SCAN_CHANNEL_TIMEOUT    2000            // Milliseconds
#define BSSID_LENGTH            6

/* WLAN security types */
#define SEC_UNSECURED           0
#define SEC_WEP                 1
#define SEC_WPA                 2
#define SEC_WPA2                3    

typedef struct ScanResult {
    uint32_t num_networks;
    uint32_t scan_status;
    unsigned is_valid : 1;
    unsigned rssi : 7;
    unsigned security_mode : 2;
    unsigned ssid_length : 6;
    uint16_t entry_time;
    unsigned char ssid[32];
    unsigned char bssid[6];
} ScanResult;

typedef struct AccessPointInfo {
    unsigned int rssi;
    unsigned int security_mode;
    char ssid[32];
    unsigned char bssid[6];
} AccessPointInfo;

class SFE_CC3000 {
public:
    SFE_CC3000(uint8_t int_pin, uint8_t en_pin, uint8_t cs_pin);
    ~SFE_CC3000();
    bool init();
	bool getFirmwareVersion(unsigned char *fw_ver);
    bool getMacAddress(unsigned char *mac_addr);
    bool scanAccessPoints(unsigned int scan_time);
    bool getNextAccessPoint(AccessPointInfo &ap_info);  
    bool connect(const char *ssid, const char *password, uint8_t sec);
private:
    bool is_initialized_;
    uint32_t num_access_points_;
    uint32_t access_point_count_;
    ScanResult ap_scan_result_;
};

#endif