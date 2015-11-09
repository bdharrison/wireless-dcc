/*
Copyright (c) 2014, Brian Harrison
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * main.c
 *
 * DCC Decoder application for MSP430G2553 with Anaren Radio
 * Note radio jumper set for GD0 on P1.1
 *
 ***********************************************************************************
 * Brian Harrison Aug 30, 2014 - briandharrison@gmail.com
 ***********************************************************************************
 * ACLK is at 32 kHz, 31us per cycle  (8192 cycles = 250ms)
 * SMCLK is at 8MHz, 125ns per cycle
 *
 * Watchdog timer will reset system every 250ms, unless it is reset
 * Timer0 is 125ns cycle, set to trigger on falling edge of the radio data
 * Timer1 is 250ns cycle used to wake up main loop every 4ms and for transmit timing
 *
 */

#include  "msp430g2553.h"
#include  "CC1101.h"

// P1 Connections
#define     LED1                  BIT0	// Steady when receiving, otherwise flashes
#define     GDO0                  BIT1
#define     UNUSED12              BIT2
#define     UNUSED13              BIT3
#define     UNUSED14              BIT4
#define     SCLK                  BIT5	// To radio
#define     MISO                  BIT6	// To radio
#define     MOSI                  BIT7	// To radio

// P2 connections
#define     RADIO0                BIT0	// 1 bit for radio channel - input
#define     RADIO1                BIT1 	// 2 bit for radio channel - output high
#define     RADIO2                BIT2 	// 4 bit for radio channel - input
#define     DCCENABLE             BIT3	// Output to Enable of power controller
#define     DCCA                  BIT4 	// Output to IN1 of power controller
#define     DCCB                  BIT5	// Output to IN2 of power controller
#define     UNUSED26              BIT6
#define     CSN                   BIT7 	// To radio

#define		TOGGLE_DCC			(P2OUT ^= (DCCA | DCCB))

struct buffer {
	bool ready;
	unsigned char data[8];
	unsigned int count;
} receiveBuf, transmitBuf;

unsigned char currentChannel=0xff;
unsigned int receiveData, receiveBit;
enum recState { preamble=0, getbyte=1, startendbit=2} receiveState;
unsigned int transmitData, transmitBit, transmitCount;
unsigned char ledCounter;
unsigned int timeoutCount;
bool bStop;

const unsigned char stopPacket[] = {0x00, 0x40, 0x40};	// Broadcast, stop, checksum

unsigned int receivedPackets, sentPackets, droppedPackets;	// Counters for debugging

void Initialize(void);
void InitializeTimers(void);
void InitializePorts(void);
void ProcessData(void);
void TransmitDCC(const unsigned char* data, unsigned int count);
void TransmitPacket(void);
void EnableDCC(void);
void SetRadioChannel(void);
void CheckTimeout(void);
void SetupRadioReceive(void);
void InitializeRadio(void);
inline void ClearBuffer(struct buffer* buf);
void SendBytes(const unsigned char* bytes, int byteCount);
void Send2Bytes(unsigned char b1, unsigned char b2);
inline void SendByte(unsigned char b);


/*
 * Main loop
 *
 * Initialize and loop waiting for interrupt every 4ms
 */
void main(void)
{

  Initialize();

  __enable_interrupt();                     // Enable interrupts.
  P1OUT |= LED1;
  
  /* Main Application Loop */
  while(1)
  {    
    __bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled
    										// Wait for timer 1 CCR1 to turn on CPU
    
    WDTCTL =  WDT_ARST_250;					// Reset Watchdog timer: ACLK 250ms

    ProcessData();
    SetRadioChannel();
    CheckTimeout();
  }  
}

/*
 * Initialize
 */
void Initialize(void)
{
	int i;
	for (i=6665; i>0; i--) { }		// Delay loop

	InitializeTimers();
	InitializePorts();

	timeoutCount = 0;
	ledCounter = 252;
	bStop = false;
	receivedPackets = 0;
	sentPackets = 0;
	droppedPackets = 0;

	SendByte(CC1101_SIDLE);		// Exit RX/TX, turn off frequency synthesizer
	InitializeRadio();
	SetupRadioReceive();
}

/*
 * InitializeTimers
 */
void InitializeTimers(void)
{
    __bis_SR_register(OSCOFF);        	// Oscillator off

    // System will reset if the watchdog is not reset every 250ms
    WDTCTL =  WDT_ARST_250;				// Reset Watchdog timer: ACLK 250ms
    
    // Basic system clock at 8MHz
    BCSCTL3 |= LFXT1S_2;				// VOCLK
    DCOCTL = 0;
    BCSCTL1 = CALBC1_8MHZ;
    DCOCTL  = CALDCO_8MHZ;

    // UCB is configured as I2C bus to radio
	UCB0CTL0 |= UCA10 | UCMM | UCMST | UCSYNC;
	UCB0CTL1 |= UCSSEL_2;						// SMCLK
	UCB0BR0 = 1;
	UCB0BR1 = 0;
	UCB0CTL1 &= ~(UCSWRST);						// Clear - released for operation

	// Timer 0 is used to detect bits received by radio
	TA0CTL = TASSEL_2 | MC_2;					// SMCLK, /1, Continuous up
	
	// Timer 1 is used for PCM to the motor and to wake up system every cycle
	TA1CTL = TASSEL_2 | ID_1 | MC_2 | TACLR;	// SMCLK, /2, Continuous, Clear
	TA1CCR1 = 16384;							// 4ms timer
	TA1CCTL1 = CM_0 | CCIS_0 | OUTMOD_0 | CCIE;	// Enable ints

}

/*
 * InitializePorts
 *
 * Set up MSP430 I/O ports
 * Several are used for peripheral I/O to connect to radio
 */
void InitializePorts(void)
{
	P1OUT = GDO0;						// GDO0 high
	P1REN = GDO0;						// Pull GDO0 up
	P1SEL  = MOSI | MISO | SCLK | GDO0;	// Primary peripheral GDO0,
	P1SEL2 = MOSI | MISO | SCLK;		// secondary for MOSI, MISO, SCLK
	P1DIR = LED1 | UNUSED12 | UNUSED13 | UNUSED14;	// Output ports

	P2OUT = CSN | RADIO1;				// Other ports low
	P2REN = RADIO2 | RADIO0;			// Pullup radio select inputs
	P2SEL = 0;							// All ports I/O
	P2DIR = 0xff ^ (RADIO0 | RADIO2);		// Other ports output
	P3REN = 0xff;						// Pull up all P3 ports (not used)
}

/*
 * ProcessData
 *
 * Check if a packet has been received and, if so, sent it on the DCC output.
 * LED is flashed if no packets are recevied
 */
void ProcessData(void)
{
	if (receiveBuf.ready) {
		timeoutCount = 0;
		bStop = false;				// Stop sending stop packets
		TransmitDCC(receiveBuf.data, receiveBuf.count);
		SetupRadioReceive();		// Prepare for the next packet

		ledCounter=0;				// Ensure LED stays on
	}

	if ((ledCounter&0xf0) == 0)		// Top bits are zero for 1/16 seconds
		P1OUT |= LED1;				// LED on
	else
		P1OUT &= ~LED1;				// LED off

	ledCounter++;
}

/*
 * TransmitDCC
 *
 * If the transmit buffer isn't busy, copy the data to the transmit buffer
 * and start sending the DCC packet
 */
void TransmitDCC(const unsigned char* data, unsigned int count)
{
	unsigned int i;

	if (transmitBuf.ready == false) {	// Throw away packet if transmitter busy
		for (i=0; i<count; i++)
			transmitBuf.data[i] = data[i];
		transmitBuf.count = count;
		TransmitPacket();
	} else
		droppedPackets++;
}

/*
 * TransmitPacket
 *
 * Prepare to send the transmit buffer as a DCC packet
 * Start with the DCC preamble and rely on the Timer1 interrupt to send the bits
 */
void TransmitPacket(void)
{
	transmitCount = 0;
	transmitData = 0xbfff;		// Preamble: 1011 1111 1111 1111
	EnableDCC();
	transmitBuf.ready = true;
}

/*
 * EnableDCC
 *
 * Enable the DCC output and set DCCA high and DCCB low for the first half of a bit
 * Timer1 will trigger very shortly to take over DCC transmission
 */
void EnableDCC(void)
{
	if ((P2IN & DCCENABLE) == 0) {
		P2OUT &= ~DCCB;					// DCCB low
		P2OUT |= (DCCA | DCCENABLE);	// DCCA high and enable output
		TA1CCR2 = TA1R + 225;			// Interrupt in 56us
		TA1CCTL2 = CCIE;				// Interrupts enabled
	}
}

/*
 * CheckTimeout
 *
 * Make sure trains don't run away if we lose contact with the transmitter.
 * If no packets received for 4 seconds then DCC broadcast a 'stop' command
 * every 4 seconds to bring trains to a clean stop.
 *
 * If no packets are recevied for 33s then cut the DCC power altogether,
 * but still listen for incoming packets.
 */
void CheckTimeout(void)
{
	if (P2OUT & DCCENABLE) {
		timeoutCount++;
		if (timeoutCount >= 8192)		// 8192*4ms = 33s after last packet
			P2OUT &= ~(DCCA | DCCB | DCCENABLE);	// Disable DCC
		else if (timeoutCount >= 1024)	// 1024*4ms = 4s
			bStop = true;

		if (bStop)
			TransmitDCC(stopPacket, sizeof(stopPacket));
	}
}


/*
 * SetRadioChannel
 *
 * Choose a readio channel based on the bits from signals RADIO0-2; default channel 2
 * If channel has changed then reconfigure radio
 *
 * The frequencies supported by the radio:
 *   0: 921.37MHz;	 1: 919.87MHz;	 2: 915.37MHz;	 3: 912.37MHz;
 *   4: 909.37MHz;	 5: 907.87MHz;	 6: 906.37MHz;	 7: 903.37MHz;
 *   8: 926.12MHz;	 9: 924.62MHz;	10: 923.12MHz;	11: 918.12MHz;
 *  12: 916.87MHz;	13: 913.62MHz;	14: 910.87MHz;	15: 904.87MHz;
 */
void SetRadioChannel(void)
{
	unsigned char channel;
	static const unsigned char channelSelect[] =
			{0x4B, 0x45, 0x33, 0x27, 0x1B, 0x15, 0x0F, 0x03};

	channel = P2IN & (RADIO0 | RADIO1 | RADIO2);

	if (channel != currentChannel) {
	    __bic_SR_register(GIE);        	// Disable interrupts
		SendByte(CC1101_SIDLE);			// Exit RX/TX, turn off frequency synthesizer
		Send2Bytes(CC1101_REG_CHANNR, channelSelect[channel]);	// Channel number
		SendByte(CC1101_SRX);			// Enable RX
		currentChannel = channel;
	    __bis_SR_register(GIE);        	// Enable interrupts
	}

}

/*
 * SetupRadioReceive
 *
 * Initialize receiver to start receiving a packet
 * Clears receive buffer and configures clock, ready for preamble
 */
void SetupRadioReceive(void)
{
	TA0CCTL0 = 0;			// Timer off
	receiveData = 0;
	receiveBit = 8;
	receiveState = preamble;

	ClearBuffer(&receiveBuf);

	TA0CCTL0 = CM_2 | SCS | CAP | CCIE;	// Trailing edge, Sync, Capture, Int. enabled
										// Wait for falling edge then 75us longer
}

/*
 * InitializeRadio
 *
 * Send initialization sequence to radio over I2C and prepare buffers
 */
void InitializeRadio(void)
{
	const static unsigned char initSequence[] = {
	0x40, 0x2E, 0x2E, 0x0D, 0x07, 0xD3, 0x91, 0xFF,
	0x04, 0x32, 0x00, 0x4B, 0x06, 0x00, 0x21, 0x6E,
	0x2C, 0xBA, 0x84, 0x00, 0x23, 0x2F, 0x47, 0x07,
	0x30, 0x18, 0x16, 0x6C, 0x03, 0x40, 0x91, 0x87,
	0x6B, 0xFB, 0x56, 0x10, 0xE9, 0x2A, 0x00, 0x1F,
	0x40, 0x00, 0x89, 0x7F, 0x63, 0x81, 0x35, 0x09 };

	SendBytes(initSequence, sizeof(initSequence));

	ClearBuffer(&receiveBuf);
	ClearBuffer(&transmitBuf);
}

/*
 * ClearBuffer
 */
inline void ClearBuffer(struct buffer* buf)
{
	unsigned int i;

	buf->ready = false;
	buf->count = 0;
	for (i=0; i<sizeof(buf->data); i++)
		buf->data[i] = 0;
}

/*
 * SendBytes
 *
 * Send commands to radio over I2C bus
 */
void SendBytes(const unsigned char* bytes, int byteCount)
{
	unsigned int i;

	P2OUT &= ~CSN;								// Set CSN low
	while (P1IN&MISO) {}						// Wait for MISO to go low

	for (i=0; i<byteCount; i++) {
		UCB0TXBUF = bytes[i];
		while (UCB0STAT & UCALIFG) {}			// Wait for int. pending
	}

	P2OUT |= CSN;								// Set CSN high
}

/*
 * SendByte
 */
inline void SendByte(unsigned char b)
{
	SendBytes(&b, 1);
}

/*
 * Send2Bytes
 */
void Send2Bytes(unsigned char b1, unsigned char b2)
{
	unsigned char bytes[2];
	bytes[0] = b1;
	bytes[1] = b2;
	SendBytes(bytes, 2);
}

/*
 *  Timer0 interrupt service routine
 *
 *  Timer0 is used for radio receive timing
 *  Configures to trigger on falling edge of signal, use CCR0 to wait another 75us and
 *  determine if a 0 or 1 bit has been received.
 *
 *  Operates in three stages:
 *    preamble: look for ten 1 bits followed by a 0 bit
 *    getbyte:  build a received data byte from 8 bits
 *    startendbit: between bytes, if zero received then receive another byte,
 *                 if 1 received then validate checksum (XOR of all bytes) and set
 *                 'ready' flag if a valid packet has been received
 *
 *  The main loop will pick up any received packets by checking the 'ready' flag
 *  every time around its loop (every 4ms)
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0 (void)
{
	unsigned char checksum;

	if (TA0CCTL0 & CAP) {			// Capture mode?
		if (!(P1IN & GDO0)) {
			TA0CCTL0 = CCIE;		// Enable interrupts
			TA0CCR0 += 600;			// Increase timer 75us
		}
	} else {
		TA0CCTL0 = CM_2 | SCS | CAP | CCIE;	// Falling edge, sync, capture, enable ints
		receiveData <<= 1;			// Shift bits left
		if (TA0CCTL0 & SCCI) {		// Capture input high?
			receiveData++;			// Set low bit
		}

		switch (receiveState) {

		case preamble:
			if ((receiveData & 0x07ff) == 0x07fe) {	// ten 1s followed by a 0?
				receiveState = getbyte;
			}
			break;

		case getbyte:
			if (--receiveBit == 0) {	// Got a full byte?
				receiveBuf.data[(receiveBuf.count)++] = (unsigned char)receiveData;
				receiveBit = 8;
				receiveState = startendbit;
			}
			break;

		case startendbit:
			if (receiveData & 0x0001) {	// 1 implies end of bytes sequence
				checksum = receiveBuf.data[5] ^ receiveBuf.data[4] ^ receiveBuf.data[3] ^ receiveBuf.data[2] ^ receiveBuf.data[1] ^ receiveBuf.data[0];
				if ((checksum != 0) || (receiveBuf.count < 3)) {	// Bad sequence
					SetupRadioReceive();		// Start over
				} else {
					TA0CCTL0 = 0;				// Stop receive for now
					receiveBuf.ready = true;	// Flag for data buffer
					receivedPackets++;
				}
			} else {
				if (receiveBuf.count > 6) {		// Too many bytes
					SetupRadioReceive();		// Start over
				} else {
					receiveState = getbyte;
				}
			}
			break;

		default:
			break;
		}
	}

}

/*
 * Timer1 Interrupt Service Routine
 *
 * Timer1 runs continually for two purposes:
 *  - CCR1 triggers every 4ms and wakes up the main loop
 *  - CCR2 is used to time the DCC transmit, triggers every 56us ('1') or 112 us ('0')
 *
 * Bits start with DCCA high, and the timer toggles the output on the first interrupt.
 * Next time around DCA is toggled high again and the loop moves on to the next bit.
 * Once all bits are sent, move to next byte; send a 0 bit at the start of each byte.
 * Once all bytes are sent then send 'end packet' - three 1 bits are sent.
 * Finally the buffer is cleared allowing the main loop to transmit more packets.
 *
 * Note that DCC wil continue to toggle every 56us (1 bits) even when there is no
 * data to transmit.
 */
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1 (void)
{
	switch (TA1IV) {

	case TA1IV_TACCR1:		// CCR1 triggers every 16384 clocks, 4.096ms
		TA1CCR1 += 16384;
		__bic_SR_register_on_exit(CPUOFF);        // Return to active mode
		break;

	case TA1IV_TACCR2:		// CCR2 used to control bit transmission
		TA1CCR2 += 225;		// Add 56us to timer
		if (transmitBit && (transmitData&transmitBit) == 0)	// Zero bit?
			TA1CCR2 += 225;	// Add another 56us

		TOGGLE_DCC;

		if (transmitBuf.ready == false)	// Do nothing more if idle
			break;

		if (P2OUT & DCCA)		// Sent both halves of bit?
			transmitBit >>= 1;	// Shift right for next bit

		if (transmitBit == 0) {	// Send all bits?
			if (transmitCount < transmitBuf.count) {	// More bytes to send
				transmitBit = 0x100;	// Send bit 8 (zero) followed by bits 7-0
				transmitData = transmitBuf.data[transmitCount++];
			} else {
				if (transmitCount == transmitBuf.count) {	// Send ending '1' bits
					transmitData = 0xffff;
					transmitBit  = 0x0004;	// End of data - send 3 1's
					transmitCount++;
				} else {		// No more data to send
					sentPackets++;
					ClearBuffer(&transmitBuf);
				}
			}
		}
		break;
	}
}
