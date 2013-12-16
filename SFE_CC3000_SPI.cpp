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

#include "SFE_CC3000_SPI.h"
#include "utility/hci.h"
#include "utility/evnt_handler.h"


#define READ                    3
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define ASSERT_CS()          (P1OUT &= ~BIT3)

#define DEASSERT_CS()        (P1OUT |= BIT3)

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


void SpiWriteDataSynchronous(unsigned char *data, unsigned short size);
void SpiWriteAsync(const unsigned char *data, unsigned short size);
void SpiPauseSpi(void);
void SpiResumeSpi(void);
void SSIContReadOperation(void);

// The magic number that resides at the end of the TX/RX buffer (1 byte after
// the allocated size) for the purpose of detection of the overrun. The location
// of the memory where the magic number resides shall never be written. In case 
// it is written - the overrun occurred and either receive function or send
// function will stuck forever.
#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)

//*****************************************************************************
// 
//!  SpiCleanGPIOISR
//! 
//!  \param  none
//! 
//!  \return none
//! 
//!  \brief  This function get the reason for the GPIO interrupt and clear
//!          corresponding interrupt flag
// 
//*****************************************************************************
void
SpiCleanGPIOISR(void)
{
    
}
 
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
void
SpiClose(void)
{
    /*
	if (sSpiInformation.pRxPacket)
	{
		sSpiInformation.pRxPacket = 0;
	}
	
	//	Disable Interrupt in GPIOA module...
	tSLInformation.WlanInterruptDisable();
    */
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
void 
SpiOpen(gcSpiHandleRx pfRxHandler)
{
	/*
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
    */
}

//*****************************************************************************
//
//!  init_spi
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  initializes an SPI interface
//
//*****************************************************************************

int init_spi(void)
{

    /*
	// Select the SPI lines: MISO/MOSI on P1.6,7 
	MOSI_MISO_PORT_SEL |= (SPI_MISO_PIN + SPI_MOSI_PIN);
	MOSI_MISO_PORT_SEL2 &= (~(SPI_MISO_PIN + SPI_MOSI_PIN));
	//CLK on P2.2
	SPI_CLK_PORT_SEL |= (SPI_CLK_PIN);
	SPI_CLK_PORT_SEL2 &= ~SPI_CLK_PIN;
	
	UCB0CTLW0 |= UCSWRST;                     // Put state machine in reset
	UCB0CTLW0 |= (UCMST+UCSYNC+UCMSB);      	// 3-pin, 8-bit SPI master
	// Clock polarity high, MSB
	UCB0CTLW0 |= UCSSEL_2;                    // ACLK
	UCB0BR0 = 0x02;                           // /2 change to /1
	UCB0BR1 = 0;                              //
	
	UCB0CTLW0 &= ~UCSWRST;                    //Initialize USCI state machine
	*/
    
	return(ESUCCESS);
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
long
SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{
	/*
    // workaround for first transaction
	ASSERT_CS();
	
	// Assuming we are running on 24 MHz ~50 micro delay is 1200 cycles;
	__delay_cycles(1200);
	
	// SPI writes first 4 bytes of data
	SpiWriteDataSynchronous(ucBuf, 4);
	
	__delay_cycles(1200);
	
	SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);
	
	// From this point on - operate in a regular way
	sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	
	DEASSERT_CS();
    */
	
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
long
SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{

    /*
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
		
		// Assert the CS line and wait till SSI IRQ line is active and then
		// initialize write operation
		ASSERT_CS();
		
		// Re-enable IRQ - if it was not disabled - this is not a problem...
		tSLInformation.WlanInterruptEnable();

		// check for a missing interrupt between the CS assertion and enabling back the interrupts
		if (tSLInformation.ReadWlanInterruptPin() == 0)
		{
                	SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

			sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

			DEASSERT_CS();
		}
	}
	
	// Due to the fact that we are currently implementing a blocking situation
	// here we will wait till end of transaction
	while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState)
		;
	*/
    
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
void
SpiWriteDataSynchronous(unsigned char *data, unsigned short size)
{
    /* Use Arduino's SPI.transfer() function
	while (size)
	{   	
		while (!(TXBufferIsEmpty()));
		UCB0TXBUF = *data;
		while (!(RXBufferIsEmpty()));
		UCB0RXBUF;
		size --;
		data++;
	}
    */
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
void
SpiReadDataSynchronous(unsigned char *data, unsigned short size)
{
    /* Use Arduino's SPI.transfer() function
	long i = 0;
	unsigned char *data_to_send = tSpiReadHeader;
	
	for (i = 0; i < size; i ++)
	{
		while (!(TXBufferIsEmpty()));
		//Dummy write to trigger the clock
		UCB0TXBUF = data_to_send[0];
		while (!(RXBufferIsEmpty()));
		data[i] = UCB0RXBUF;
	}
    */
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
void
SpiReadHeader(void)
{
    /*
	SpiReadDataSynchronous(sSpiInformation.pRxPacket, 10);
    */
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
long
SpiReadDataCont(void)
{
    /*
	long data_to_recv;
	unsigned char *evnt_buff, type;
	
	//determine what type of packet we have
	evnt_buff =  sSpiInformation.pRxPacket;
	data_to_recv = 0;
	STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET,
									type);
	
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
	*/
    
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

void 
SpiPauseSpi(void)
{
    
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

void 
SpiResumeSpi(void)
{
    
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
void 
SpiTriggerRxProcessing(void)
{
	/*
	// Trigger Rx processing
	SpiPauseSpi();
	DEASSERT_CS();
	
	// The magic number that resides at the end of the TX/RX buffer (1 byte after 
	// the allocated size) for the purpose of detection of the overrun. If the 
	// magic number is overwritten - buffer overrun occurred - and we will stuck 
	// here forever!
	if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
	{
		while (1)
			;
	}
	
	sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
    */
}

//*****************************************************************************
// 
//!  IntSpiGPIOHandler
//! 
//!  @param  none
//! 
//!  @return none
//! 
//!  @brief  GPIO A interrupt handler. When the external SSI WLAN device is
//!          ready to interact with Host CPU it generates an interrupt signal.
//!          After that Host CPU has registered this interrupt request
//!          it set the corresponding /CS in active state.
// 
//*****************************************************************************
void IntSpiGPIOHandler(void)
{
    /*
	switch(__even_in_range(P2IV,P2IV_P2IFG3))
	{
	case P2IV_P2IFG3:
		if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
		{
			//This means IRQ line was low call a callback of HCI Layer to inform 
			//on event 
	 		sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
		}
		else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
		{
			sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
			
			//IRQ line goes down - we are start reception
			ASSERT_CS();
			
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
			
			DEASSERT_CS();
		}
		break;
	default:
		break;
	}
	*/
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

void
SSIContReadOperation(void)
{
    /*
	// The header was read - continue with  the payload read
	if (!SpiReadDataCont())
	{
		// All the data was read - finalize handling by switching to the task
		//	and calling from task Event Handler
		SpiTriggerRxProcessing();
	}
    */
}
