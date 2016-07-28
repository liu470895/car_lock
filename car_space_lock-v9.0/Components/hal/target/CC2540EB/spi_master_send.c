
/***********************************************************************************
  Filename:     spi_master_send.c

  Description:  This example uses a master to send data to a slave using SPI.

  Comments:     To execute this example, use the IAR project spi_slave_receive
                as the slave. The slave's code must be executed before executing
                the master's code, since the slave is clocked by the Master. The
                bytes sent are simply numbers 0x00 upto BUFFER_SIZE. Note that
                if BUFFER_SIZE is larger than 0xFF, then the element 0xFF in the
                array will have the value 0x00 and so on. When bytes up to
                BUFFER_SIZE are sent, the example will end and LED1 will be set
                on SmartRF05EB.

Configuration for each module:

CC2541EM, CC2543EM, CC2545EM: 
                Connect from each board:
                 - MISO:  P0_2  (PIN9 on Debug Connector P18)
                 - MOSI:  P0_3  (PIN11 on Debug Connector P18)
                 - SSN:   P0_4  (PIN13 on Debug Connector P18)
                 - SCK:   P0_5  (PIN15 on Debug Connector P18)
                 - GND:         (PIN20 on Debug Connector P18)

CC2544Dongle:
                No configuration for Dongle in this example. 
                The code illsutrate how to set up the interface for CC2544.
                There is not a CC2544 HW plattform available for running 
                this excact code.

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <hal_types.h>
// Include Name definitions of individual bits and bit-fields in the CC254x device registers.
#include <ioCC254x_bitdef.h>
#include "ioCC2541.h"

//#if (chip==2541)
//#include "ioCC2541.h"
//#elif (chip==2543)
//#include "ioCC2543.h"
//#elif (chip==2544)
//#include "ioCC2544.h"
//#warning "The CC2544Dongle is not supported for this software example."
//#warning "The definitions for CC2544 in the code illustrate how to set up an SPI interface."
//#elif (chip==2545)
//#include "ioCC2545.h"
//#else
//#error "Chip not supported!"
//#endif


/***********************************************************************************
* CONSTANTS
*/

// These values will give a baud rate of approx. 2.00 Mbps at 32 MHz system clock
#define SPI_BAUD_M  0
#define SPI_BAUD_E  16

// Define size of buffer and number of bytes to send
#define BUFFER_SIZE 252

/***********************************************************************************
* LOCAL VARIABLES
*/

// Masters's transmit buffer
static uint8 txBufferMaster[BUFFER_SIZE];

/***********************************************************************************
* LOCAL FUNCTIONS
*/


/***********************************************************************************
* @fn          main
*
* @brief       Send data to a single slave using SPI in Master mode
*
* @param       void
*
* @return      void
*/
void spi_master_send(void)
{
    /****************************************************************************
     * Clock setup
     * See basic software example "clk_xosc_cc254x"
     */
    
    // Set system clock source to HS XOSC, with no pre-scaling.
    CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M;
    while (CLKCONSTA & CLKCON_OSC);   // Wait until clock source has changed
    
    // Note the 32 kHz RCOSC starts calibrating, if not disabled.


    /***************************************************************************
     * Setup I/O ports
     *
     * Port and pins used by USART0 operating in SPI-mode are
     * MISO (MI): P0_2
     * MOSI (MO): P0_3
     * SSN (SS) : P0_4
     * SCK (C)  : P0_5
     *
     * These pins can be set to function as peripheral I/O to be be used by
     * USART0 SPI. Note however, when SPI is in master mode, only MOSI, MISO,
     * and SCK should be configured as peripheral I/O's. If the external
     * slave device requires a slave select signal (SSN), then the master
     * can control the external SSN by using one of its GPIO pin as output.
     */
    
#if (chip==2541 || chip==2543 || chip==2545)
    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0).
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U0CFG_ALT2;    
#endif
        
#if (chip==2541)
    // Give priority to USART 0 over Timer 1 for port 0 pins.
    P2DIR &= P2DIR_PRIP0_USART0;
#endif


#if (chip==2541 || chip==2543 || chip==2545)
    // Set pins 13, 14 and 15 as peripheral I/O and pin 12 as GPIO output.
    P1SEL = (P0SEL & ~BIT12) | BIT15 | BIT14 | BIT13;
    P1DIR |= BIT12;
#endif
    
    /***************************************************************************
     * Configure SPI
     */

    // Fill array with bytes to send.
    uint8 value = 0x00;
    int i;
    for (i = 0; i < BUFFER_SIZE; i++)
    {
        txBufferMaster[i] = value++;
    }

    // Set USART to SPI mode and Master mode.
    U0CSR &= ~(U0CSR_MODE | U0CSR_SLAVE);

    // Set:
    // - mantissa value
    // - exponent value
    // - clock phase to be centered on first edge of SCK period
    // - negative clock polarity (SCK low when idle)
    // - bit order for transfers to LSB first
    U0BAUD = SPI_BAUD_M;
    U0GCR = (U0GCR & ~(U0GCR_BAUD_E | U0GCR_CPOL | U0GCR_CPHA | U0GCR_ORDER))
        | SPI_BAUD_E;

    /***************************************************************************
     * Transfer data
     */

#if (chip==2541 || chip==2543 || chip==2545)
    // Clear SSN, the SPI slave is active when SSN is low.
    P1_2 = 0;
#endif

    for (i = 0; i < BUFFER_SIZE; i++)
    {
        // Write byte to USART0 buffer (transmit data).
        U0DBUF = txBufferMaster[i];

        // Check if byte is transmitted.
        while(!(U0CSR & U0CSR_TX_BYTE));

        // Clear transmit byte status.
        U0CSR &= ~U0CSR_TX_BYTE;
    }

#if (chip==2541 || chip==2543 || chip==2545)
    // Set SSN, the SPI slave is inactive when SSN is high.
    P1_2 = 1;
#endif

}


/***********************************************************************************
  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/

