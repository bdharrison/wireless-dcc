/**
 * \file uart.c
 * \author Chris Karaplis
 * \brief MSP430 hardware UART driver
 *
 * Copyright (c) 2015, simplyembedded.org
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "uart.h"
#include "ring_buffer.h"
#include <stdint.h>
#include <stddef.h>
#include <msp430.h>

#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))

struct baud_value
{
    uint32_t baud;
    uint16_t UCAxBR0;
    uint16_t UCAxBR1;
    uint16_t UCAxMCTL;
};

/* Table of baud rate register values from reference manual (SLAU144) */
static const struct baud_value _baud_tbl[] = {
    {9600, 65, 3, UCBRS_2},
    {19200, 160, 1, UCBRS_6},
    {38400, 208, 0, UCBRS_3},
    {56000, 142, 0, UCBRS_7},
    {115200, 69, 0, UCBRS_4},
    {256000, 31, 0, UCBRS_2}
};

/* RX ring bufer */
static rbd_t _rbd;
static char _rbmem[8];

/* TX ring bufer */
static rbd_t _rbdtx;
static char _rbtxmem[64];

/**
 * \brief Initialize the UART peripheral
 * \param[in] config - the UART configuration
 * \return 0 on success, -1 otherwise
 */
int uart_init(uart_config_t *config)
{
    int status = -1;

    /* USCI should be in reset before configuring - only configure once */
    if (UCA0CTL1 & UCSWRST) {
        size_t i;

        /* Set clock source to SMCLK */
        UCA0CTL1 |= UCSSEL_2;

        /* Find the settings from the baud rate table */
        for (i = 0; i < ARRAY_SIZE(_baud_tbl); i++) {
            if (_baud_tbl[i].baud == config->baud) {
                break;
            }
        }

        if (i < ARRAY_SIZE(_baud_tbl)) {
            rb_attr_t attr = {sizeof(_rbmem[0]), ARRAY_SIZE(_rbmem), _rbmem};

            /* Set the baud rate */
            UCA0BR0 = _baud_tbl[i].UCAxBR0;
            UCA0BR1 = _baud_tbl[i].UCAxBR1;
            UCA0MCTL = _baud_tbl[i].UCAxMCTL;

            /* Initialize the ring buffer */
            if (ring_buffer_init(&_rbd, &attr) == 0) {                 

            	/* Enable the USCI peripheral (take it out of reset) */
                UCA0CTL1 &= ~UCSWRST;

                /* Enable rx interrupts */
                IE2 |= UCA0RXIE;

                // Initialize the TX ring
                rb_attr_t attrtx = {sizeof(_rbtxmem[0]), ARRAY_SIZE(_rbtxmem), _rbtxmem};
                if (ring_buffer_init(&_rbdtx, &attrtx) == 0) {
                	status = 0;
                }
            }
        }
    }

    return status;
}

/**
 * \brief Read a character from UART
 * \return the character read on success, -1 if nothing was read
 */
int uart_getchar(void)
{
    int retval = -1;
    char c = 0xFF;
    
    if (ring_buffer_get(_rbd, &c) == 0) {
        retval = (int) c;
    }

    return retval;
}

/**
 * \brief Write a character to UART
 * \param[in] c - the character to write
 * \return 0 on sucess, -1 otherwise
 */
int uart_putchar(int c)
{
	ring_buffer_put(_rbdtx, &c);
    IE2 |= UCA0TXIE;		// Enable TX interrupts
    
    return 0;
}

/**
 * \brief Write a string to UART
 * \return 0 on sucesss, -1 otherwise
 */
int uart_puts(const char *str)
{
    int status = -1;

    if (str != NULL) {
        status = 0;

        while (*str != '\0') {
			ring_buffer_put(_rbdtx, str);

            /*  If there is a line-feed, add a carriage return */
            if (*str == '\n') {
            	const char r = '\r';
				ring_buffer_put(_rbdtx, &r);
            }
            str++;
        }
        IE2 |= UCA0TXIE;	// Enable TX interrupt
    }

    return status;
}
#pragma vector=USCIAB0RX_VECTOR
__interrupt void rx_isr (void)
{
    if (IFG2 & UCA0RXIFG) {
        const char c = UCA0RXBUF;
        
        /* Clear the interrupt flag */
        IFG2 &= ~UCA0RXIFG;

        ring_buffer_put(_rbd, &c);
    }
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void tx_isr (void)
{
    char c;

    if (IFG2 & UCA0TXIFG) {		// TX buffer empty?
        if (ring_buffer_get(_rbdtx, &c) == 0)
            UCA0TXBUF = c;		// Transmit character, which clears interrupt
        else
            IE2 &= ~UCA0TXIE;	// Turn off TX interrupts if nothing to transmit
   }
}
