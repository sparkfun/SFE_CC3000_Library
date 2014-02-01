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
 
/* Debug setting. Set to 0 for no debug output. Set to 1 for debug output
 * If you enable debugging, make sure you call Serial.begin()! */
#define DEBUG 1

/* Define success and failure constants for CC3000 library functions. For
 * whatever reason, TI assigned 0 as success. */
#define CC3000_SUCCESS 0

/* Global variable declarations */
extern uint8_t g_int_pin;
extern uint8_t g_int_num;
extern uint8_t g_en_pin;
extern uint8_t g_cs_pin;
extern volatile unsigned long ulSmartConfigFinished;
extern volatile unsigned long ucStopSmartConfig;
extern volatile unsigned long ulCC3000Connected;
extern volatile unsigned long ulCC3000DHCP;
extern volatile unsigned long ulCC3000DHCP_configured;
extern volatile unsigned long OkToDoShutDown;

#if (DEBUG == 1)
extern volatile long g_debug_interrupt;
#endif

#endif