/**
 * @file	SFE_CC3000_Callbacks.h
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

#ifndef SFE_CC3000_CALLBACKS_H
#define SFE_CC3000_CALLBACKS_H

void cc3000AsyncCallback(long lEventType, char * data, unsigned char length);
char *sendFirmwarePatch(unsigned long *Length);
char *sendDriverPatch(unsigned long *Length);
char *sendBootLoaderPatch(unsigned long *Length);
long readWlanInterruptPin();
void enableWlanInterrupt();
void disableWlanInterrupt();
void writeWlanPin(unsigned char val);

#endif