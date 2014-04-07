/**
 * @file	SFE_CC3000_SPI.cpp
 * @brief 	CC3000 library functions to handle SPI
 * @author	Texas Instruments
 * @author  Modified by Shawn Hymel (SparkFun Electronics)
 *
 * This code was originally written by TI to work with their microcontrollers.
 * Most of it has been altered to work with the Arduino. 
 */
 
/*****************************************************************************
*
*  spi.c - CC3000 Host Driver Implementation.
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

#include <Arduino.h>
#include <SPI.h>

#include "common.h"
#include "SFE_CC3000_SPI.h"
#include "utility/hci.h"
#include "utility/evnt_handler.h"


#define READ                    3
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define HEADERS_SIZE_EVNT       (SPI_HEADER_SIZE + 5)

#define SPI_HEADER_SIZE			(5)

#define 	eSPI_STATE_POWERUP 				 (0)
#define 	eSPI_STATE_INITIALIZED  		 (1)
#define 	eSPI_STATE_IDLE					 (2)
#define 	eSPI_STATE_WRITE_IRQ	   		 (3)
#define 	eSPI_STATE_WRITE_FIRST_PORTION   (4)
#define 	eSPI_STATE_WRITE_EOT			 (5)
#define 	eSPI_STATE_READ_IRQ				 (6)
#define 	eSPI_STATE_READ_FIRST_PORTION	 (7)
#define 	eSPI_STATE_READ_EOT				 (8)


typedef struct
{
	gcSpiHandleRx  SPIRxHandler;
	unsigned short usTxPacketLength;
	unsigned short usRxPacketLength;
	unsigned long  ulSpiState;
	unsigned char *pTxPacket;
	unsigned char *pRxPacket;

}tSpiInformation;


tSpiInformation sSpiInformation;


// buffer for 5 bytes of SPI HEADER
unsigned char tSpiReadHeader[] = {READ, 0, 0, 0, 0};

// The magic number that resides at the end of the TX/RX buffer (1 byte after
// the allocated size) for the purpose of detection of the overrun. The location
// of the memory where the magic number resides shall never be written. In case 
// it is written - the overrun occurred and either receive function or send
// function will stuck forever.
#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)

char spi_buffer[CC3000_RX_BUFFER_SIZE];
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];
 
//*****************************************************************************
//
//!  SpiClose
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Close Spi interface
//
//*****************************************************************************
void SpiClose(void)
{
	if (sSpiInformation.pRxPacket)
	{
		sSpiInformation.pRxPacket = 0;
	}
	
	//	Disable Interrupt in GPIOA module...
	tSLInformation.WlanInterruptDisable();
}


//*****************************************************************************
//
//!  SpiOpen
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Open Spi interface 
//
//*****************************************************************************
void SpiOpen(gcSpiHandleRx pfRxHandler)
{
    sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;
	sSpiInformation.SPIRxHandler = pfRxHandler;
	sSpiInformation.usTxPacketLength = 0;
	sSpiInformation.pTxPacket = NULL;
	sSpiInformation.pRxPacket = (unsigned char *)spi_buffer;
	sSpiInformation.usRxPacketLength = 0;
	spi_buffer[CC3000_RX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
	wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
	
	// Enable interrupt on the GPIOA pin of WLAN IRQ
	tSLInformation.WlanInterruptEnable();
    
}

//*****************************************************************************
//
//! SpiFirstWrite
//!
//!  @param  ucBuf     buffer to write
//!  @param  usLength  buffer's length
//!
//!  @return none
//!
//!  @brief  enter point for first write flow
//
//*****************************************************************************
long SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{
    // Save SPI settings
    save_spi_params();

	// workaround for first transaction
	digitalWrite(g_cs_pin, LOW);
	
	// Assuming we are running on 24 MHz ~50 micro delay is 1200 cycles;
	delayMicroseconds(50);
	
	// SPI writes first 4 bytes of data
	SpiWriteDataSynchronous(ucBuf, 4);
	
	delayMicroseconds(50);
	
	SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);
	
	// From this point on - operate in a regular way
	sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	
	digitalWrite(g_cs_pin, HIGH);
    
    // Restore SPI settings
    restore_spi_params();
	
	return(0);
}

//*****************************************************************************
//
//!  SpiWrite
//!
//!  @param  pUserBuffer  buffer to write
//!  @param  usLength     buffer's length
//!
//!  @return none
//!
//!  @brief  Spi write operation
//
//*****************************************************************************
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
	unsigned char ucPad = 0;
	
	// Figure out the total length of the packet in order to figure out if there 
	// is padding or not
	if(!(usLength & 0x0001))
	{
		ucPad++;
	}
	
	pUserBuffer[0] = WRITE;
	pUserBuffer[1] = HI(usLength + ucPad);
	pUserBuffer[2] = LO(usLength + ucPad);
	pUserBuffer[3] = 0;
	pUserBuffer[4] = 0;
	
	usLength += (SPI_HEADER_SIZE + ucPad);
	
	// The magic number that resides at the end of the TX/RX buffer (1 byte after 
	// the allocated size) for the purpose of detection of the overrun. If the 
	// magic number is overwritten - buffer overrun occurred - and we will stuck 
	// here forever!
	if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
	{
		while (1)
			;
	}
	
	if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
	{
		while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED)
			;
	}
	
	if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
	{
		// This is time for first TX/RX transactions over SPI: the IRQ is down - 
		// so need to send read buffer size command
		SpiFirstWrite(pUserBuffer, usLength);
	}
	else 
	{
		// We need to prevent here race that can occur in case 2 back to back 
		// packets are sent to the  device, so the state will move to IDLE and once 
		//again to not IDLE due to IRQ
		tSLInformation.WlanInterruptDisable();
		
		while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE)
		{
			;
		}
		
		sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
		sSpiInformation.pTxPacket = pUserBuffer;
		sSpiInformation.usTxPacketLength = usLength;
        
        // Save SPI settings
        save_spi_params();
		
		// Assert the CS line and wait till SSI IRQ line is active and then
		// initialize write operation
		digitalWrite(g_cs_pin, LOW);
		
		// Re-enable IRQ - if it was not disabled - this is not a problem...
		tSLInformation.WlanInterruptEnable();

		// check for a missing interrupt between the CS assertion and enabling back the interrupts
		if (tSLInformation.ReadWlanInterruptPin() == 0)
		{
                	SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

			sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

			digitalWrite(g_cs_pin, HIGH);
            
            // Restore SPI settings
            restore_spi_params();
		}
	}
	
	// Due to the fact that we are currently implementing a blocking situation
	// here we will wait till end of transaction
	while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState)
		;
    
	return(0);
}

//*****************************************************************************
//
//!  SpiWriteDataSynchronous
//!
//!  @param  data  buffer to write
//!  @param  size  buffer's size
//!
//!  @return none
//!
//!  @brief  Spi write operation
//
//*****************************************************************************
void SpiWriteDataSynchronous(unsigned char *data, unsigned short size)
{
	while (size)
	{
        SPI.transfer(*data);
		size --;
		data++;
	}
}

//*****************************************************************************
//
//! SpiReadDataSynchronous
//!
//!  @param  data  buffer to read
//!  @param  size  buffer's size
//!
//!  @return none
//!
//!  @brief  Spi read operation
//
//*****************************************************************************
void SpiReadDataSynchronous(unsigned char *data, unsigned short size)
{
	long i = 0;
	unsigned char *data_to_send = tSpiReadHeader;
	
	for (i = 0; i < size; i ++)
	{
		data[i] = SPI.transfer(0x03);
	}
}

//*****************************************************************************
//
//!  SpiReadHeader
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief   This function enter point for read flow: first we read minimal 5 
//!	          SPI header bytes and 5 Event Data bytes
//
//*****************************************************************************
void SpiReadHeader(void)
{
	SpiReadDataSynchronous(sSpiInformation.pRxPacket, 10);
}

//*****************************************************************************
//
//!  SpiReadDataCont
//!
//!  @param  None
//!
//!  @return None
//!
//!  @brief  This function processes received SPI Header and in accordance with 
//!	         it - continues reading the packet
//
//*****************************************************************************
long SpiReadDataCont(void)
{
	long data_to_recv;
	unsigned char *evnt_buff, type;
	
	//determine what type of packet we have
	evnt_buff =  sSpiInformation.pRxPacket;
	data_to_recv = 0;
	STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), 
                                            HCI_PACKET_TYPE_OFFSET, type);
	
	switch(type)
	{
        case HCI_TYPE_DATA:
		{
			// We need to read the rest of data..
			STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), 
										HCI_DATA_LENGTH_OFFSET, data_to_recv);
			if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
			{	
				data_to_recv++;
			}
			
			if (data_to_recv)
			{
				SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
			}
			break;
		}
        case HCI_TYPE_EVNT:
		{
			// Calculate the rest length of the data
			STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), 
									HCI_EVENT_LENGTH_OFFSET, data_to_recv);
			data_to_recv -= 1;
			
			// Add padding byte if needed
			if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
			{
				
				data_to_recv++;
			}
			
			if (data_to_recv)
			{
				SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
			}
			
			sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
			break;
		}
	}
	
	return (0);
}

//*****************************************************************************
//
//! SpiPauseSpi
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Spi pause operation
//
//*****************************************************************************
void SpiPauseSpi(void)
{
    detachInterrupt(g_int_num);
}

//*****************************************************************************
//
//! SpiResumeSpi
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Spi resume operation
//
//*****************************************************************************
void SpiResumeSpi(void)
{
    attachInterrupt(g_int_num, cc3000_ISR, FALLING);
}

//*****************************************************************************
//
//! SSIContReadOperation
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  SPI read operation
//
//*****************************************************************************
void SSIContReadOperation(void)
{
	// The header was read - continue with  the payload read
	if (!SpiReadDataCont())
	{
	
		// All the data was read - finalize handling by switching to the task
		//	and calling from task Event Handler
		SpiTriggerRxProcessing();
	}
}

//*****************************************************************************
//
//! SpiTriggerRxProcessing
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Spi RX processing 
//
//*****************************************************************************
void SpiTriggerRxProcessing(void)
{
	// Trigger Rx processing
	SpiPauseSpi();
	digitalWrite(g_cs_pin, HIGH);
    
    // Restore SPI settings
    restore_spi_params();
	
	// The magic number that resides at the end of the TX/RX buffer (1 byte 
    // after the allocated size) for the purpose of detection of the overrun. 
    // If the magic number is overwritten - buffer overrun occurred - and we 
    // will stuck here forever!
	if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != 
                                        CC3000_BUFFER_MAGIC_NUMBER)
	{
		while (1)
			;
	}
	
	sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
}

//*****************************************************************************
// Custom functions
//*****************************************************************************

/**
 * @brief Gets the SPI mode from the SPI control register
 *
 * Returns the SPI mode as given by:
 * 0x00 = MODE0
 * 0x04 = MODE1
 * 0x08 = MODE2
 * 0x0C = MODE3
 *
 * @return The SPI mode 
 */
uint8_t get_spi_data_mode(void) {
    return (SPCR & SPI_MODE_MASK);
}

/**
 * @brief Gets the bit order (MSB or LSB first) of SPI transactions
 *
 * @return 1 for MSB first, 0 for LSB first
 */
uint8_t get_spi_bit_order(void) {
    return bitRead(SPCR, DORD) ? 0 : 1;
}

/**
 * @brief Gets the clock divider for SPI
 *
 * Returns the clock divider for SPI based on the SPCR and SPSR registers.
 * 0x00 = DIV4
 * 0x01 = DIV16
 * 0x02 = DIV64
 * 0x03 = DIV128
 * 0x04 = DIV2
 * 0x05 = DIV8
 * 0x06 = DIV32
 * 0x07 = DIV64 (not implemented in Arduino)
 *
 * @return value of SPI2X, SPR1, and SPR0 bits as an unsigned 8-bit integer
 */
uint8_t get_spi_clock_div(void) {
    uint8_t clock_div;
    clock_div = (SPCR & SPI_CLOCK_MASK);
    clock_div = clock_div | ((SPSR & SPI_2XCLOCK_MASK) << 2);
    return clock_div;
}

/**
 * @brief Saves the current SPI parameters in global variables
 */
void save_spi_params(void) {

    /* Save current SPI settings */
    g_saved_data_mode = get_spi_data_mode();
    g_saved_bit_order = get_spi_bit_order();
    g_saved_clock_div = get_spi_clock_div();
    
    /* Set SPI settings for CC3000 */
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLK_DIV);
    
}

/**
 * @brief Restores the previously saved SPI parameters
 */
void restore_spi_params(void) {
    SPI.setDataMode(g_saved_data_mode);
    SPI.setBitOrder(g_saved_bit_order);
    SPI.setClockDivider(g_saved_clock_div);
}

/**
 * @brief Interrupt Service Routine for GPIO interrupt
 */
void cc3000_ISR(void)
{
    if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
    {
        //This means IRQ line was low call a callback of HCI Layer to inform 
        //on event 
        sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
    }
    else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
    {
        sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
        
        // Save SPI settings
        save_spi_params();
        
        //IRQ line goes down - we are start reception
        digitalWrite(g_cs_pin, LOW);
        
        // Wait for TX/RX Compete which will come as DMA interrupt
        SpiReadHeader();
        
        sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT; 
        
        SSIContReadOperation();
    }
    else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
    {
        SpiWriteDataSynchronous(sSpiInformation.pTxPacket,
                                            sSpiInformation.usTxPacketLength);
        
        sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
        
        digitalWrite(g_cs_pin, HIGH);
        
        // Restore SPI settings
        restore_spi_params();
    }
    
}