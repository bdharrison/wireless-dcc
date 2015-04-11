/*
Copyright (c) 2014, Brian Harrison
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list
   of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this
   list of conditions and the following disclaimer in the documentation and/or other
   materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 DAMAGE.
*/

/*
 * main.c
 *
 * DCC Transmitter application for MSP430G2553 with Anaren Radio
 *
 ***********************************************************************************
 * Brian Harrison March, 2015 - briandharrison@gmail.com
 ***********************************************************************************
 * ACLK is at 32 kHz, 31us per cycle  (8192 cycles = 250ms)
 * SMCLK is at 8MHz, 125ns per cycle
 *
 * Watchdog timer will reset system every 250ms, unless it is reset
 * Timer0 is 125ns cycle, set to trigger on falling edge of the radio data
 * Timer1 is 31us cycle using ACLK on low power, used to check the push button
 * for power on/off
 *
 */

#include  "msp430g2553.h"
#include  "CC1101.h"

// P1 Connections
#define     LED1                  BIT0	// out: Green LED and GDO2
#define     POTH                  BIT1	// out: Pot high
#define     POTIN	              BIT2	// in:  Pot tap
#define     POTL	              BIT3	// out: Pot low
#define     UNUSED14              BIT4
#define     SCLK                  BIT5	// out: To radio
#define     MISO                  BIT6	// in:  I2C from radio UCB0
#define     MOSI                  BIT7	// out: I2C to radio UCB0

// P2 connections
#define     RADIO0                BIT0	// in:  1 bit for radio channel (high for 3)
#define     RADIO1                BIT1 	// out: 2 bit for radio channel - high (=2)
#define     RADIO2                BIT2 	// in:  4 bit for radio channel (high for 10)
#define     PUSH0	              BIT3	// in:  Pushbutton 0
#define     PUSH1                 BIT4 	// in:  Pushbutton 1
#define     PUSH2                 BIT5	// in:  Pushbutton 2
#define     GDO0	              BIT6	// in:  GDO0 = TA0.1 Timer0_A
#define     CSN                   BIT7 	// out: To radio

// DCC Standard bits
#define		DCCFL				BIT4
#define		DCCF1				BIT0
#define		DCCF2				BIT1

#define		SET_BIT(var, bits)		var |= (bits)
#define		CLEAR_BIT(var, bits)	var &= ~(bits)
#define		TOGGLE_BIT(var, bits)	var ^= (bits)

unsigned int locoAddress;
unsigned char newPBStatus, PBStatus, dccPBStatus;
unsigned char ledCounter;
bool bForwards;
unsigned int nCounter;
unsigned char currentChannel;
unsigned char powerOffCount;
bool bPowerOff;
unsigned int speed;

unsigned int dccTransmitData;
unsigned int dccTransmitBit;
unsigned int dccBufferIndex;
unsigned int dccBufferSize;
struct {
	unsigned char dccAddress0;
	unsigned char dccAddress1;
	unsigned char dccInstruction;
	unsigned char dccChecksum;
	unsigned char dccEnd;
} dccBuffer;

void Initialize(void);
void InitializeTimers(void);
void InitializePorts(void);
void SetRadioChannel(void);
void InitializeRadio(void);

void GetSpeed(void);
void GetPBState(void);
void CheckPushbuttons(void);
void FlashLED(void);
void SetupDCCBuffer(void);
void TransmitData(void);
void WaitForPB(void);

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

  /* Main Application Loop */
  while(1)
  {    
    __bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled
    										// Wait for timer 1 CCR1 to turn on CPU
    
    WDTCTL =  WDT_ARST_250;					// Reset Watchdog timer: ACLK 250ms

    SendByte(CC1101_STX);					// In IDLE state: enable TX

    GetSpeed();
    CheckPushbuttons();
    SetRadioChannel();
    FlashLED();
    SetupDCCBuffer();
    TransmitData();
    WaitForPB();
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

	locoAddress = 11;				// Change this as needed!
	currentChannel = 0;
	newPBStatus = 0;
	PBStatus = 0;
	ledCounter = 0;
	powerOffCount = 0;
	bPowerOff = false;

	InitializeRadio();
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
    SET_BIT(BCSCTL3, LFXT1S_2);				// VOCLK
    DCOCTL = 0;
    BCSCTL1 = CALBC1_8MHZ;
    DCOCTL  = CALDCO_8MHZ;

    // UCB is configured as I2C bus to radio
	SET_BIT(UCB0CTL0, UCA10 | UCMM | UCMST | UCSYNC);
	SET_BIT(UCB0CTL1, UCSSEL_2);						// SMCLK
	UCB0BR0 = 1;
	UCB0BR1 = 0;
	CLEAR_BIT(UCB0CTL1, UCSWRST);						// Clear - released for operation

}

/*
 * InitializePorts
 *
 * Set up MSP430 I/O ports
 * Several are used for peripheral I/O to connect to radio
 */
void InitializePorts(void)
{
	P1OUT = 0;							// Outputs low
	P1SEL = MOSI | MISO | SCLK;			// Secondary mode for MOSI, MISO, SCLK
	P1SEL2 = MOSI | MISO | SCLK;		// secondary for MOSI, MISO, SCLK
	P1DIR = MOSI | SCLK | UNUSED14 | POTL | POTH | LED1;	// Outputs
	ADC10AE0 = POTIN;					// Analog enable

	P2SEL = GDO0;
	P2OUT = CSN | RADIO1;				// High
	P2REN = RADIO0 | RADIO2 | PUSH0 | PUSH1 | PUSH2;	// Pull inputs up
	P2DIR = CSN | RADIO1;				// Outputs

	P3REN = 0xff;						// Pull up all P3 ports (not used)
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
		SendByte(CC1101_STX);			// In IDLE state: enable TX
		currentChannel = channel;
	    __bis_SR_register(GIE);        	// Enable interrupts
	}

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

	dccPBStatus = 0;
	bForwards = true;
	nCounter = 0;

	// Timer 0 is used for transmit timing
	TA0CTL = TASSEL_2 | ID_1 | MC_2 | TACLR | TAIE;	// SMCLK, /2, Continuous up, clear, enable

	SendByte(CC1101_SIDLE);					// Exit RX/TX, turn off frequency synthesizer

	SendBytes(initSequence, sizeof(initSequence));

	Send2Bytes(CC1101_PATABLE, 0x8E);		// RF output power level

	SetRadioChannel();
}

/*
 * GetSpeed
 *
 * Get the current speed setting from the potentiometer using the A2D converter
 * Middle is off, higher is forwards and lower is reverse
 * Set up speed and direction
 */
void GetSpeed(void)
{
	const static unsigned char speedTable[] = {
			0x00, 0x02, 0x12, 0x03, 0x13, 0x04, 0x14, 0x05,
			0x15, 0x06, 0x16, 0x07, 0x17, 0x08, 0x18, 0x18,
			0x09, 0x19, 0x0A, 0x1A, 0x0B, 0x1B, 0x0C, 0x0C,
			0x1C, 0x0D, 0x1D, 0x0E, 0x1E, 0x0F, 0x1F, 0x1F
	};
	unsigned int setting;

	CLEAR_BIT(P1DIR, LED1);					// Set P1.0 to input mode
	SET_BIT(P1OUT, POTL);					// Set ref output high
	ADC10CTL1 = INCH_2;						// Channel A2, input on P1.2
	ADC10CTL0 = ADC10SHT_3 | ADC10ON;		// 64xADC10 clk + ADC10 enabled
	SET_BIT(ADC10CTL0, ADC10SC | ENC);		// Enable conversion and start

	while (ADC10CTL1 & ADC10BUSY) {
		// Loop while busy
	}
	CLEAR_BIT(ADC10CTL0, ENC);				// Disable conversion
	CLEAR_BIT(P1OUT, POTL);
	SET_BIT(P1DIR, LED1);
	setting = ADC10MEM;
	if (setting < 0x200) {					// In reverse?
		bForwards = false;
		setting = 0x200 - setting;
	} else {
		bForwards = true;
		setting -= 0x200;
	}
	setting >>= 4;					// Reduce to 5 bits
	speed = speedTable[setting];

}

void GetPBState(void){
	newPBStatus = P2IN & (PUSH0 | PUSH1 | PUSH2);	// Read current PB state
	if (newPBStatus == PBStatus)
		newPBStatus = 0;
	else
		PBStatus = newPBStatus;
}
/*
 * CheckPushbuttons
 *
 * Read the current state of the pushbuttons and set the corresponding state
 * in the DCC function group instruction
 */
void CheckPushbuttons(void)
{
	GetPBState();

	if (PBStatus & PUSH2) {			// PB2 is held down
		if (--powerOffCount == 0) {		// Counter expired - need to power down
			bPowerOff = true;
		}
	} else {
		bPowerOff = false;
		powerOffCount = 0;
	}

	if (PBStatus & PUSH0)			// PB0 - F2 (horn)
		SET_BIT(dccPBStatus, DCCF2);
	else
		CLEAR_BIT(dccPBStatus, DCCF2);

	if (PBStatus & PUSH1)			// PB1 - F0 (bell)
		SET_BIT(dccPBStatus, DCCF1);
	else
		CLEAR_BIT(dccPBStatus, DCCF1);
//		TOGGLE_BIT(dccPBStatus, DCCF1);

	if (newPBStatus & PUSH2)			// PB2 - FL (light)
		TOGGLE_BIT(dccPBStatus, DCCFL);

}

/*
 * FlashLED
 *
 * Flash the LED based on the speed & direction
 */
void FlashLED(void)
{
	CLEAR_BIT(P1OUT, LED1);				// LED off
	ledCounter++;
	if (speed == 0) {			// Steady on if stationary
		SET_BIT(P1OUT, LED1);
	} else if (bForwards) {		// Forwards: Flash slowly
		if (ledCounter & 0x10) {
			SET_BIT(P1OUT, LED1);
		}
	} else {					// Backwards: Flash quickly
		if (ledCounter & 0x08) {
			SET_BIT(P1OUT, LED1);
		}
	}
}

/*
 * SetupDCCBuffer
 *
 * Prepare the dccBuffer structure for transmitting standard DCC commands.
 * The transmit packet includes one or two bytes for the address, an instruction byte,
 * and a checksum.  The instruction is usually speed and direction, but every 8th
 * packet the pushbitton status is sent instead.
 */
void SetupDCCBuffer(void)
{
	nCounter++;
	nCounter &= 0x03;	// Counts between 0 & 7

	dccBuffer.dccAddress0 = 0;
	dccBuffer.dccAddress1 = locoAddress;
	dccBufferIndex = 1;			// Start transmission at dccAddress1
	if (locoAddress >= 0x64) {	// Loco has a 14-bit address?
		dccBuffer.dccAddress0 = locoAddress>>8 | 0xC0;	// Set top two bits of most significant byte
		dccBufferIndex = 0;		// Start transmission at dccAddress0 for 2 address bytes
	}

	if (nCounter == 0) {
		dccBuffer.dccInstruction = dccPBStatus | 0x80;
	} else {
		dccBuffer.dccInstruction = speed | 0x40;
		if (bForwards)
			SET_BIT(dccBuffer.dccInstruction, 0x20);
	}
	dccBuffer.dccChecksum = dccBuffer.dccAddress0 ^ dccBuffer.dccAddress1 ^ dccBuffer.dccInstruction;
	dccBufferSize = 5;
}

/*
 * TransmitData
 *
 * Send no-op, wait for ack, then start timer to initiate transmitting
 */
void TransmitData(void)
{
	do {			// Send no-op and wait for ACK
		SendByte(CC1101_SNOP);
	} while (UCB0RXBUF != 0x2F);

	TA0CCTL1 = OUT;	// Timer A output high, disable ints
	SET_BIT(P2DIR, GDO0);

	dccTransmitData = 0xafff;	// Preamble bits: 1010 1111 1111 1111
	dccTransmitBit = 0x8000;	// Start with bit 15
	TA0CCR1 = TA0R + 0x0010;	// Count 16 cycles (16x200ns = 3.2 us)
	TA0CCTL1 = OUTMOD_5 | CCIE;	// Reset and enable interrupts
}

/*
 * WaitForPB
 *
 * If waitPBCounter reaches zero then power down and wait for PB2 to be pressed
 */
void WaitForPB(void)
{
	if (bPowerOff) {
		CLEAR_BIT(P1OUT, LED1);
		CLEAR_BIT(P2DIR, GDO0);
		TA0CCTL1 = 0;
		SendByte(CC1101_SIDLE);		// Exit TX/RX, trun off freq. syth
		SendByte(CC1101_SPWD);		// Power down when CSN high
		TA0CTL = TASSEL_1 | MC_2 | TACLR;	// ACLK, continuous up, reset
		TA0CCR2 = 8;				// 8 cycles of ACLK = 250us
		TA0CCTL2 = CCIE;			// Enable interrupts

		do {
			__bis_SR_register(LPM3_bits + GIE);	// LPM3 with interrupts enabled
			WDTCTL = WDT_ARST_250;		// Watchdog to 250ms
			GetPBState();
		} while (!(newPBStatus & PUSH2)); // Repeat until PB2 is pressed

		bPowerOff = false;
		TA0CCTL2 = 0;
		InitializeRadio();
	}
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
 * Timer0 Interrupt Service Routine
 *
 * Timer0 runs continually for two purposes:
 *  - CCR1 is used to time the DCC transmit, triggers every 56us ('1') or 112 us ('0')
 *  - CCR2 triggers every 4ms and wakes up the main loop
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
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0 (void)
{
	switch (TA0IV) {

	case TA0IV_TACCR1:				// CCR1 used to control DCC bit transmission
		TA0CCR1 += 225;				// Add 56us to timer
		if ((dccTransmitData & dccTransmitBit) == 0)
			TA0CCR1 += 225;			// Double timer length for zero bits

		if (TA0CCTL1 & OUTMOD2) {			// Currently reset?
			TA0CCTL1 = OUTMOD_1 | CCIE;		// Set mode
			break;
		}
		TA0CCTL1 = OUTMOD_5 | CCIE;			// Reset mode

		dccTransmitBit = dccTransmitBit>>1;
		if (dccTransmitBit == 0) {			// End of byte - look for next byte
			dccTransmitBit = 0x100;			// Send bit 8 (zero) followed by bits 7-0
			if (dccBufferIndex == dccBufferSize) {	// Reached end of buffer?
				TA0CCTL1 = 0;
				CLEAR_BIT(P2DIR, GDO0);
				SendByte(CC1101_SIDLE);		// Exit RX/TX, turn off frequency synthesizer
			} else {
				dccTransmitData = ((unsigned char*)&dccBuffer)[dccBufferIndex++];
				if (dccBufferIndex == dccBufferSize) {
					dccTransmitData = 0xffff;
					dccTransmitBit = 0x0004;	// End of data - send 3 1's
				}
			}
		}

		break;

	case TA0IV_TACCR2:				// CCR2
		TA0CCR2 += 768;				// Add ??? to timer
		__bic_SR_register_on_exit(LPM3_bits);        // Return to active mode
		break;

	case TA0IV_TAIFG:	// Timer overflow, triggers after 65536 * 200ns = 13ms
		__bic_SR_register_on_exit(CPUOFF);        // Return to active mode
		break;
	}
}


