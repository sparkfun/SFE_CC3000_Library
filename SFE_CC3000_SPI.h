/**
 * @file	SFE_CC3000_SPI.h
 * @brief 	CC3000 library functions to handle SPI
 * @author	Texas Instruments
 * @author  Modified by Shawn Hymel (SparkFun Electronics)
 *
 * This code was originally written by TI to work with their microcontrollers.
 * Most of it has been altered to work with the Arduino. 
 */

/*****************************************************************************
*
*  spi.h  - CC3000 Host Driver Implementation.
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/


#ifndef SFE_CC3000_SPI_H
#define SFE_CC3000_SPI_H

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

typedef void (*gcSpiHandleRx)(void *p);
typedef void (*gcSpiHandleTx)(void);

extern unsigned char wlan_tx_buffer[];

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void SpiOpen(gcSpiHandleRx pfRxHandler);
extern void SpiClose(void);
extern long SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength);
extern long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength);
extern void SpiWriteDataSynchronous(unsigned char *data, unsigned short size);
extern void SpiReadDataSynchronous(unsigned char *data, unsigned short size);
extern void SpiReadHeader(void);
extern long SpiReadDataCont(void);
extern void SpiResumeSpi(void);
extern void SSIContReadOperation(void);
extern void SpiTriggerRxProcessing(void);
extern void cc3000_ISR(void);

#endif

