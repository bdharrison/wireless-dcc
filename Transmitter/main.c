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
 * Timer1 is 125ns cycle, set to trigger on falling edge of the radio data
 * Timer2 is 31us cycle using ACLK on low power, used to check the push button
 * for power on
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


// Following constants can be modified by changing the values in the HEX file
#pragma SET_DATA_SECTION(".infoC")
const unsigned int locoAddress = 3;	// Change this to desired value
const bool push0Toggle = false;			// PB0 is press & hold, for whistle
const bool push1Toggle = false;			// PB1 is press & hold, for bell
const bool push2Toggle = true;			// PB2 toggles off and on, for light
#pragma SET_DATA_SECTION()

unsigned int dccTransmitData;
unsigned int dccTransmitBit;
unsigned int dccBufferIndex;
struct {
	unsigned char dccAddress0;
	unsigned char dccAddress1;
	unsigned char dccInstruction;
	unsigned char dccChecksum;
	unsigned char dccEnd;
} dccBuffer;

static unsigned int powerOffCounter = 0;

void Initialize(void);
void InitializeTimers(void);
void InitializePorts(void);
void SetRadioChannel(bool bReset);
void InitializeRadio(void);

int  GetSpeed(void);
unsigned char GetPushButtonState(void);
void SetDCCFunction(unsigned char* pDccFunction, unsigned char pbState, unsigned char pbBit, unsigned char dccBit, bool bToggle);
unsigned char DCCFunction(unsigned char pbState);
unsigned char DCCSpeed(int speed);
void FlashLED(int speed);
void SetupDCCBuffer(int speed, unsigned char pbState);
void TransmitData(void);
void CheckPowerOff(void);

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
	int speed;
	unsigned char pbState, dccFunction;

	Initialize();

	/* Main Application Loop */
	while(1) {
		__bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled
    											// Wait for timer 1 CCR1 to turn on CPU
    
		WDTCTL =  WDT_ARST_250;					// Reset Watchdog timer: ACLK 250ms

		SendByte(CC1101_STX);					// In IDLE state: enable TX

		speed = GetSpeed();
		pbState = GetPushButtonState();
		dccFunction = DCCFunction(pbState);
		SetRadioChannel(false);
		FlashLED(speed);
		SetupDCCBuffer(speed, dccFunction);
		TransmitData();
		CheckPowerOff();
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
void SetRadioChannel(bool bReset)
{
	static unsigned char currentChannel=0;
	unsigned char channel;
	static const unsigned char channelSelect[] =
			{0x4B, 0x45, 0x33, 0x27, 0x1B, 0x15, 0x0F, 0x03};

	if (bReset)
		currentChannel = 0;

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

	// Timer 0 is used for transmit timing
	TA0CTL = TASSEL_2 | ID_1 | MC_2 | TACLR | TAIE;	// SMCLK, /2, Continuous up, clear, enable

	SendByte(CC1101_SIDLE);					// Exit RX/TX, turn off frequency synthesizer

	SendBytes(initSequence, sizeof(initSequence));

	Send2Bytes(CC1101_PATABLE, 0x8E);		// RF output power level

	SetRadioChannel(true);
}

/*
 * GetSpeed
 *
 * Get the current speed setting from the potentiometer using the A2D converter
 * Middle is off, higher is forwards and lower is reverse
 * Set up speed and direction
 */
int GetSpeed(void)
{
	int speed;

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
	speed = ADC10MEM>>4;					// 10-bit value reduced to 0-64
	speed -= 32;							// Change to signed -32 to +32
	return speed;
}

/*
 * GetPushButtonState
 *
 * Returns the current state of the pushbuttons.
 * Bit 7 is set if the state has changed since the previous call
 * Resets the 15-minute power off timer if the buttons have been pressed
 */
unsigned char GetPushButtonState(void){
	static unsigned char pbStatus = 0;
	unsigned char newPBStatus;

	CLEAR_BIT(pbStatus, 0x80);
	newPBStatus = P2IN & (PUSH0 | PUSH1 | PUSH2);	// Read current PB state
	if (newPBStatus != pbStatus) {		// Pushbutton state changed
		SET_BIT(newPBStatus, 0x80);
		if (newPBStatus & PUSH2)		// PB2 is pressed
			powerOffCounter = 0x00ff;		// Short time to power off
		else
			powerOffCounter = 0;			// Reset 15 minute timer for power off
	}
	pbStatus = newPBStatus;
	return pbStatus;
}

void SetDCCFunction(unsigned char* pDccFunction, unsigned char pbState, unsigned char pbBit, unsigned char dccBit, bool bToggle)
{
	if (bToggle) {
		if ((pbState & 0x80) && (pbState & pbBit))	// State changed AND button pressed
			TOGGLE_BIT(*pDccFunction, dccBit);
	} else {
		if (pbState & pbBit)
			SET_BIT(*pDccFunction, dccBit);
		else
			CLEAR_BIT(*pDccFunction, dccBit);
	}

}

/*
 * DCCFunction
 *
 * Read the current state of the pushbuttons and return the corresponding state
 * in the DCC function group instruction:
 * Bit 7-5: 100 for function group one
 * Bit 4:   FL - Light
 * Bit 3:	F4
 * Bit 2:	F3
 * Bit 1:	F2 - Horn
 * Bit 0:	F1 - Bell
 */
unsigned char DCCFunction(unsigned char pbState)
{
	static unsigned char dccFunction=0x80 | DCCFL;	// DCC Function Group One command, light on

	SetDCCFunction(&dccFunction, pbState, PUSH0, DCCF2, push0Toggle);		// PB0 - F2 (horn)
	SetDCCFunction(&dccFunction, pbState, PUSH1, DCCF1, push1Toggle);		// PB1 - F1 (bell)
	SetDCCFunction(&dccFunction, pbState, PUSH2, DCCFL, push2Toggle);		// PB2 - FL (light)

	return dccFunction;
}


/*
 * FlashLED
 *
 * Flash the LED based on the speed & direction
 */
void FlashLED(int speed)
{
	static unsigned char ledCounter=0;

	CLEAR_BIT(P1OUT, LED1);				// LED off
	ledCounter++;
	if (speed == 0) {			// Steady on if stationary
		SET_BIT(P1OUT, LED1);
	} else if (speed>0) {		// Forwards: Flash slowly
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
void SetupDCCBuffer(int speed, unsigned char dccFunction)
{
	static int nCounter = 0;
//	static unsigned int locoAddress=11;		// Change this as needed

	dccBuffer.dccAddress0 = 0;
	dccBuffer.dccAddress1 = locoAddress;
	dccBufferIndex = 1;			// Start transmission at dccAddress1
	if (locoAddress >= 0x64) {	// Loco has a 14-bit address?
		dccBuffer.dccAddress0 = locoAddress>>8 | 0xC0;	// Set top two bits of most significant byte
		dccBufferIndex = 0;		// Start transmission at dccAddress0 for 2 address bytes
	}

	if (nCounter++ > 6)	{		// Send pushbutton every 8th packet
		dccBuffer.dccInstruction = dccFunction;
		nCounter = 0;
	} else
		dccBuffer.dccInstruction = DCCSpeed(speed);

	dccBuffer.dccChecksum = dccBuffer.dccAddress0 ^ dccBuffer.dccAddress1 ^ dccBuffer.dccInstruction;
}

/*
 * DCCSpeed(speed)
 *
 * Convert speed from range -32 to +32 into DCC speed format:
 * Bit 7-6: 01 for speed instruction
 * Bit 5:   direction, set for forwards
 * Bit 4:   least significant bit for 32-step speed
 * Bit 3-0: remaining speed bits
 */
unsigned char DCCSpeed(int speed)
{
	unsigned char dccSpeed=0;

	if (speed == -1)
		speed = 0;

	if (speed >= 0)
		SET_BIT(dccSpeed, 0x20);	// Set forwards direction bit
	else
		speed = -speed;

	if (speed > 31)
		speed = 31;

	if (speed & 0x01)
		SET_BIT (dccSpeed, 0x10);	// Set least significant speed bit

	speed >>= 1;					// Divide by 2 for 4-bit speed
	if (speed == 1)					// Speed 1 is an emergency stop
		speed = 2;
	dccSpeed |= speed | 0x40;		// Set remaining speed bits and command bit

	return dccSpeed;
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
 * CheckPowerOff
 *
 * If powerOffCount reaches zero then power down and wait for PB2 to be pressed
 */
void CheckPowerOff(void)
{
	unsigned char pbState;

	if (--powerOffCounter == 0) {	// Counter expired - need to power down

		CLEAR_BIT(P1OUT, LED1);
		CLEAR_BIT(P2DIR, GDO0);
		TA0CCTL1 = 0;
		SendByte(CC1101_SIDLE);		// Exit TX/RX, trun off freq. syth
		SendByte(CC1101_SPWD);		// Power down when CSN high
		TA0CTL = TASSEL_1 | MC_2 | TACLR;	// ACLK, continuous up, reset
		TA0CCR2 = 8;				// 8 cycles of ACLK = 250us
		TA0CCTL2 = CCIE;			// Enable interrupts

		do {
			__bis_SR_register(LPM3_bits + GIE);		// LPM3 with interrupts enabled
			WDTCTL = WDT_ARST_250;					// Watchdog to 250ms
			pbState = GetPushButtonState();
		} while (!(pbState & PUSH2) || !(pbState & 0x80));	// Repeat until PB2 is pressed

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
			if (dccBufferIndex == sizeof(dccBuffer)) {	// Reached end of buffer?
				TA0CCTL1 = 0;
				CLEAR_BIT(P2DIR, GDO0);
				SendByte(CC1101_SIDLE);		// Exit RX/TX, turn off frequency synthesizer
			} else {
				dccTransmitData = ((unsigned char*)&dccBuffer)[dccBufferIndex++];
				if (dccBufferIndex == sizeof(dccBuffer)) {
					dccTransmitData = 0xffff;
					dccTransmitBit = 0x0004;	// End of data - send 3 1's
				}
			}
		}

		break;

	case TA0IV_TACCR2:				// CCR2
		TA0CCR2 += 768;				// Add 24ms to timer
		__bic_SR_register_on_exit(LPM3_bits);        // Return to active mode
		break;

	case TA0IV_TAIFG:	// Timer overflow, triggers after 65536 * 200ns = 13ms
		__bic_SR_register_on_exit(CPUOFF);        // Return to active mode
		break;
	}
}


