/**
 * @file	common.h
 * @brief 	Global header file for the project (for constant/global definitions)
 * @author	Shawn Hymel (SparkFun Electronics)
 * 
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 */

#ifndef COMMON_H
#define COMMON_H
 
/* Sets the name of the device. Used by SmartConfig. */
#define DEVICE_NAME     "CC3000"
 
/* Debug setting. Set to 0 for no debug output. Set to 1 for debug output
 * If you enable debugging, make sure you call Serial.begin()! */
#define DEBUG           0

/* Define success and failure constants for CC3000 library functions. For
 * whatever reason, TI assigned 0 as success. */
#define CC3000_SUCCESS  0

/* Define other constants used by the CC3000 library */
#define DISABLE         0
#define ENABLE          1
#define IP_ADDR_LEN     4   // Length of IP address in bytes

/* Includes needed for defined types */
#include "utility/netapp.h"

/* Global variable declarations */
extern uint8_t g_int_pin;
extern uint8_t g_int_num;
extern uint8_t g_en_pin;
extern uint8_t g_cs_pin;
extern bool g_socket_connected;
extern volatile unsigned long ulSmartConfigFinished;
extern volatile unsigned long ucStopSmartConfig;
extern volatile unsigned long ulCC3000Connected;
extern volatile unsigned long ulCC3000DHCP;
extern volatile unsigned long ulCC3000DHCP_configured;
extern volatile unsigned long OkToDoShutDown;
extern netapp_pingreport_args_t g_ping_report;

#if (DEBUG == 1)
extern volatile long g_debug_interrupt;
#endif

#endif