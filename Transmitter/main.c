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

#define byte	unsigned char
#define uint	unsigned int

#define		memcpy		__builtin_memcpy

// P1 Connections
#define     LED1                  BIT0	// out: Green LED and GDO2
#define     POTH                  BIT1	// out: Pot high
#define     POTIN	              BIT2	// in:  Pot tap
#define     POTL	              BIT3	// out: Pot low
#define     PUSH6		          BIT4	// in:  Pushbutton 6 - accessory
#define     SCLK                  BIT5	// out: To radio
#define     MISO                  BIT6	// in:  I2C from radio UCB0
#define     MOSI                  BIT7	// out: I2C to radio UCB0

// P2 connections
#define     PUSH5                 BIT0	// in:  Pushbutton 5
#define     PUSH4                 BIT1	// in:  Pushbutton 4
#define     PUSH3                 BIT2	// in:  Pushbutton 3
#define     PUSH2	              BIT3	// in:  Pushbutton 2
#define     PUSH1                 BIT4 	// in:  Pushbutton 1
#define     PUSH0                 BIT5	// in:  Pushbutton 0
#define     GDO0	              BIT6	// in:  GDO0 = TA0.1 Timer0_A
#define     CSN                   BIT7 	// out: To radio

// DCC Commands
#define		DCCSPEED			0x40
#define		DCCFORWARDS			BIT5
#define		DCCSPEEDLSB			BIT4
#define		DCCFGROUP1			0x80
#define		DCCFGROUP2A			0xB0
#define		DCCFGROUP2B			0xA0


// DCC bits for Function Group 1
#define		DCCF0				BIT4
#define		DCCF1				BIT0
#define		DCCF2				BIT1
#define		DCCF3				BIT2
#define		DCCF4				BIT3

// DCC bits for Function Group 2a
#define		DCCF5				BIT0
#define		DCCF6				BIT1
#define		DCCF7				BIT2
#define		DCCF8				BIT3

// DCC bits for Function Group 2b
#define		DCCF9				BIT0
#define		DCCF10				BIT1
#define		DCCF11				BIT2
#define		DCCF12				BIT3

#define		SET_BIT(var, bits)		var |= (bits)
#define		CLEAR_BIT(var, bits)	var &= ~(bits)
#define		TOGGLE_BIT(var, bits)	var ^= (bits)

#define		FLASH_ON			0	// LED steady on
#define		FLASH_SLOW			1	// LED slow flashing
#define		FLASH_FAST			2	// LED quick flashing
#define		FLASH_BLINK			3	// LED blinking
#define		FLASH_LATCH			4	// LED on for one cycle


struct configVariables {
	unsigned int locoAddress;	// Change this to desired value
	unsigned char radioChannel;	// Set to desired radio channel

	// Select toggle or press-and-hold mode	// FWD/REV				NEUTRAL
	bool f0Toggle;			// headlight			headlight
	bool f1Toggle;			// bell					bell
	bool f2Toggle;			// whistle				whistle
	bool f3Toggle;			// coupler				coupler
	bool f4Toggle;			// blower hiss			blower hiss
	bool f5Toggle;			// dynamic brake		dynamic brake
	bool f6Toggle;			// doppler on/off		start-up
	bool f7Toggle;			// squeal brakes		cylinder cocks arm
	bool f8Toggle;			// audio mute			audio mute
	bool f9Toggle;			// very heavy load		disconnect/standby/shutdown
	bool f10Toggle;			// speed report			status report
	bool f11Toggle;			// horn/whistle toggle	horn/whistle toggle
	bool f12Toggle;			// cab lights			cab lights

	bool bCenterOff;		// Speed control has center-off position
};

// Following constants can be modified by changing the values in the HEX file
// Or by entering the 'settings' mode
#pragma SET_DATA_SECTION(".infoC")

const struct configVariables flashcv = {
		3,	 							// Loco Address
		2,								// Radio Channel
		true, true, false, false,		// Toggle flags F0-F3
		true, true, false, false,		// Toggle flags F4-F7
		true, true, false, true, true,	// Toggle flags F8-F12
		true							// Center-off speed control
};

#pragma SET_DATA_SECTION(".infoB")

// Device address = (accAddress-1)*4 + accDevice + 1
// For example, switch 3 has accAddress=1 and accDevice=2
const uint accAddress[6] = { 1, 1, 1, 1, 1, 1 };	// DCC address for PB0-5
const char accDevice[6]  = { 2, 1, 0, 2, 1, 0 };	// DCC device for PB0-5
const bool accClear[6]   = { 0, 0, 0, 1, 1, 1 };	// DCC clear flag for PB0-5

#pragma SET_DATA_SECTION()

struct configVariables cv;

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

struct dccFunctions {
	unsigned char group1;	// DCC Function Group One F0-F4
	unsigned char group2a;	// DCC Function Group Two, F5-F8
	unsigned char group2b;	// DCC Function Group Two, F9-F12
};

static unsigned int powerOffCounter = 0;
static int sentAccessory = 0;	// Counter for debugging

void Initialize(void);
void InitializeTimers(void);
void InitializePorts(void);
void SetRadioChannel(bool bReset);
void InitializeRadio(void);

void GetAccessoryCommand(void);
void TransmitAccessoryPacket(unsigned int address, bool activate, byte device);

int  GetSpeed(void);
bool InAccessoryMode(void);
unsigned char GetPushButtonState(void);
void SetDCCFunction(unsigned char* pDccFunction, unsigned char pbState, unsigned char pbBit, unsigned char dccBit, bool bToggle);
void DCCFunction(unsigned char pbState, struct dccFunctions *  pDccFn);
unsigned char DCCSpeed(int speed);
void FlashLED(char rate);
void SetupDCCBuffer(int speed, struct dccFunctions * pDccFn);
void TransmitData(void);
void CheckPowerOff(void);
void ProgramSettings(void);
unsigned char GetDigit (unsigned char pbState);
void UpdateFlash(void);

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
	unsigned char pbState;
	static struct dccFunctions dccFn = {
			DCCFGROUP1 | DCCF0,
			DCCFGROUP2A,
			DCCFGROUP2B
	};

	Initialize();

	/* Main Application Loop */
	while(1) {
		__bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled
    											// Wait for timer 1 CCR1 to turn on CPU

		WDTCTL =  WDT_ARST_250;					// Reset Watchdog timer: ACLK 250ms

		SendByte(CC1101_STX);					// In IDLE state: enable TX

		if (InAccessoryMode()) {
			GetAccessoryCommand();

		} else {
			speed = GetSpeed();
			pbState = GetPushButtonState();
			DCCFunction(pbState, &dccFn);			// Set up the DCC function settings
			SetRadioChannel(false);
			FlashLED(speed? (speed>0? FLASH_SLOW : FLASH_FAST) : FLASH_ON);
			SetupDCCBuffer(speed, &dccFn);
			TransmitData();
		}
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

	// Set up config variables from flash memory
	memcpy(&cv, &flashcv, sizeof(struct configVariables));

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
	P1REN = PUSH6;						// Pull input up
	P1SEL = MOSI | MISO | SCLK;			// Secondary mode for MOSI, MISO, SCLK
	P1SEL2 = MOSI | MISO | SCLK;		// secondary for MOSI, MISO, SCLK
	P1DIR = MOSI | SCLK | POTL | POTH | LED1;	// Outputs
	ADC10AE0 = POTIN;					// Analog enable

	P2SEL = GDO0;
	P2OUT = CSN;				// High
	P2REN = PUSH0 | PUSH1 | PUSH2 | PUSH3 | PUSH4 | PUSH5;	// Pull inputs up
	P2DIR = CSN;				// Outputs

	P3REN = 0xff;						// Pull up all P3 ports (not used)
}

/*
 * SetRadioChannel
 *
 * Choose a radio channel based on the constant radioChannel; default channel 2
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
	static unsigned char currentChannel=0xff;
	static const unsigned char channelSelect[] =
			{0x4B, 0x45, 0x33, 0x27, 0x1B, 0x15, 0x0F, 0x03};

	if (bReset)
		currentChannel = 0xff;

	if (cv.radioChannel >= sizeof(channelSelect))
		cv.radioChannel = 0;

	if (cv.radioChannel != currentChannel) {
	    __bic_SR_register(GIE);        	// Disable interrupts
		SendByte(CC1101_SIDLE);			// Exit RX/TX, turn off frequency synthesizer
		Send2Bytes(CC1101_REG_CHANNR, channelSelect[cv.radioChannel]);	// Channel number
		SendByte(CC1101_STX);			// In IDLE state: enable TX
		currentChannel = cv.radioChannel;
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
 * InAccessoryMode
 */
bool InAccessoryMode(void) {
	if (cv.bCenterOff)		// Accessory mode only supported with center-off control
		return (P1IN & PUSH6);
	return false;
}

/*
 * GetAccessoryCommand
 *
 * If any pushbutton is pressed then transmit the appropriate DCC command
 */
void GetAccessoryCommand(void)
{
	byte PBState;
	int pb;

	PBState = P2IN & (PUSH0 | PUSH1 | PUSH2 | PUSH3 | PUSH4 | PUSH5);

	for (pb=0; pb<6; pb++) {
		if (PBState & 0x01) {
			TransmitAccessoryPacket(accAddress[pb], accClear[pb], accDevice[pb]);
			break;
		}
		PBState >>= 1;
	}
}

/*
 * TransmitAccessoryPacket
 *
 * Accessory packets have the format:
 * {preamble} 0 10AAAAAA 0 1AAACDDD 0 EEEEEEEE 1
 * Where C activate or deactivates device
 *       DDD specifies which device at this address
 *       Least signficant 6 bits of address are in byte 1
 *       Most significant 3 bits of address are in byte 2, ones complement
 *       EEEEEEEE is the checksum
 */
void TransmitAccessoryPacket(unsigned int address, bool clear, byte device)
{
	dccBufferIndex = 1;			// Start transmission at dccAddress1
	dccBuffer.dccAddress0 = 0;
	dccBuffer.dccAddress1 = 0x80 | (byte)(address & 0x003F);	// Bottom 6 address bits

	dccBuffer.dccInstruction = ~(address>>2);					// Top three bits of address in upper nibble
	dccBuffer.dccInstruction &= 0x70;
	dccBuffer.dccInstruction |= 0x88;					// Set bit 3 to activate
	dccBuffer.dccInstruction |= ((device<<1) & 0x06);	// Device in bits 1 & 2
	if (clear)
		dccBuffer.dccInstruction |= 0x01;				// Set bit 0 to clear

	dccBuffer.dccChecksum = dccBuffer.dccAddress0 ^ dccBuffer.dccAddress1 ^ dccBuffer.dccInstruction;

	TransmitData();
	sentAccessory++;
}


/*
 * GetSpeed
 *
 * Get the current speed setting from the potentiometer using the A2D converter
 * In center-off mode, middle is off, higher is forwards and lower is reverse
 * Otherwise PUSH6 controls direction
 *
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
	if (cv.bCenterOff) {
		speed -= 32;						// Change to signed -32 to +32
		if ((speed > -3) && (speed < 2))	// Reduce sensitivity around the mid-point
			speed = 0;

	} else {
		speed >>=2;							// Change to 0-32
		if (P1IN & PUSH6)
			speed = -speed;					// Change to reverse
	}

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
	newPBStatus = P2IN & (PUSH0 | PUSH1 | PUSH2 | PUSH3 | PUSH4 | PUSH5);	// Read current PB state
	if (newPBStatus != pbStatus) {			// Pushbutton state changed
		SET_BIT(newPBStatus, 0x80);
		if (newPBStatus & PUSH0)			// PB0 is pressed
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
 * Read the current state of the pushbuttons and set the corresponding state
 * in the DCC function group instructions.
 *
 * If PUSH5 is pressed: PUSH0=F8, PUSH1=F9, PUSH2=F10, PUSH3=F11, PUSH4=F12
 * Else if PUSH4 is pressed: PUSH0=F4, PUSH1=F5, PUSH2=F6, PUSH3=F7
 * Else: PUSH0=F0, PUSH1=F1, PUSH2=F2, PUSH3=F3
 */
void DCCFunction(unsigned char pbState, struct dccFunctions * pDccFn)
{
	if (pbState & PUSH5) {			// Set F8-12
		SetDCCFunction(&pDccFn->group2a, pbState, PUSH0, DCCF8,  cv.f8Toggle);	// PB5 + PB0 - F8
		SetDCCFunction(&pDccFn->group2b, pbState, PUSH1, DCCF9,  cv.f9Toggle);	// PB5 + PB1 - F9
		SetDCCFunction(&pDccFn->group2b, pbState, PUSH2, DCCF10, cv.f10Toggle);	// PB5 + PB2 - F10
		SetDCCFunction(&pDccFn->group2b, pbState, PUSH3, DCCF11, cv.f11Toggle);	// PB5 + PB3 - F11
		SetDCCFunction(&pDccFn->group2b, pbState, PUSH4, DCCF12, cv.f12Toggle);	// PB5 + PB4 - F12

	} else if (pbState & PUSH4) {	// Set F4-7
		SetDCCFunction(&pDccFn->group1,  pbState, PUSH0, DCCF4, cv.f4Toggle);	// PB4 + PB0 - F4
		SetDCCFunction(&pDccFn->group2a, pbState, PUSH1, DCCF5, cv.f5Toggle);	// PB4 + PB1 - F5
		SetDCCFunction(&pDccFn->group2a, pbState, PUSH2, DCCF6, cv.f6Toggle);	// PB4 + PB2 - F6
		SetDCCFunction(&pDccFn->group2a, pbState, PUSH3, DCCF7, cv.f7Toggle);	// PB4 + PB3 - F7

	} else {						// Set F0-F3
		SetDCCFunction(&pDccFn->group1, pbState, PUSH0, DCCF0, cv.f0Toggle);		// PB0 - F0 (headlight)
		SetDCCFunction(&pDccFn->group1, pbState, PUSH1, DCCF1, cv.f1Toggle);		// PB1 - F1 (bell)
		SetDCCFunction(&pDccFn->group1, pbState, PUSH2, DCCF2, cv.f2Toggle);		// PB2 - F2 (horn)
		SetDCCFunction(&pDccFn->group1, pbState, PUSH3, DCCF3, cv.f3Toggle);		// PB3 - F3
	}
}

/*
 * FlashLED
 *
 * Flash the LED based on the speed & direction
 */
void FlashLED(char rate)
{
	static unsigned char ledCounter=0;
	static bool bLatch = false;

	CLEAR_BIT(P1OUT, LED1);				// LED off
	ledCounter++;

	switch (rate) {
	case FLASH_ON:
		SET_BIT(P1OUT, LED1);
		break;

	case FLASH_SLOW:
		if (ledCounter & 0x10)
			SET_BIT(P1OUT, LED1);
		break;

	case FLASH_FAST:
		if (ledCounter & 0x08)
			SET_BIT(P1OUT, LED1);
		break;

	case FLASH_BLINK:
		if (bLatch || ((ledCounter & 0x0f) < 2))
			SET_BIT(P1OUT, LED1);
		break;

	case FLASH_LATCH:
		bLatch = true;
		ledCounter = 0;
		break;
	}

	if (bLatch && (ledCounter & 0x08))
		bLatch = false;
}

/*
 * SetupDCCBuffer
 *
 * Prepare the dccBuffer structure for transmitting standard DCC commands.
 * The transmit packet includes one or two bytes for the address, an instruction byte,
 * and a checksum.  The instruction is usually speed and direction, but every 3rd
 * packet the function settings from pushbuttons are sent instead, rotating between
 * the three function group formats.
 */
void SetupDCCBuffer(int speed, struct dccFunctions * pDccFn)
{
	static int nCounter = 0;

	dccBuffer.dccAddress0 = 0;
	dccBuffer.dccAddress1 = cv.locoAddress;
	dccBufferIndex = 1;			// Start transmission at dccAddress1
	if (cv.locoAddress >= 0x64) {	// Loco has a 14-bit address?
		dccBuffer.dccAddress0 = cv.locoAddress>>8 | 0xC0;	// Set top two bits of most significant byte
		dccBufferIndex = 0;		// Start transmission at dccAddress0 for 2 address bytes
	}

	nCounter++;
	if (nCounter == 3)			// Send function group 1, F0-F4
		dccBuffer.dccInstruction = pDccFn->group1;
	else if (nCounter == 6)		// Send function group 2a, F5-F8
		dccBuffer.dccInstruction = pDccFn->group2a;
	else if (nCounter >= 9) {	// Send function group 2b, F9-F12
		dccBuffer.dccInstruction = pDccFn->group2b;
		nCounter = 0;			// Repeat cycle

	} else						// Send speed
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
	unsigned char dccSpeed=DCCSPEED;

	if (speed >= 0)
		SET_BIT(dccSpeed, DCCFORWARDS);	// Set forwards direction bit
	else
		speed = -speed;

	if (speed > 31)
		speed = 31;

	if (speed & 0x01)
		SET_BIT (dccSpeed, DCCSPEEDLSB);	// Set least significant speed bit

	speed >>= 1;					// Divide by 2 for 4-bit speed
	if (speed == 1)					// Speed 1 is an emergency stop
		speed = 2;
	dccSpeed |= speed;				// Set remaining speed bits

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
		} while (!(pbState & PUSH0) || !(pbState & 0x80));	// Repeat until PB0 is pressed

		if (pbState & PUSH5)		// If PB5 is held down when controller woken
			ProgramSettings();		// then we enter programming mode

		TA0CCTL2 = 0;
		InitializeRadio();
	}
}

/*
 * ProgramSettings
 *
 * Wait until 5 decimal digits are entered via the pushbuttons
 * Digits 1 & 2 set the radio channel
 * Digits 3, 4 & 5 set the loco address
 */
void ProgramSettings(void)
{
	unsigned char pbState;
	unsigned int nCount = 0;
	unsigned char  i;

	// Loop waiting for the radio channel and loco address to be set
	do {
		__bis_SR_register(LPM3_bits + GIE);	// LPM0 with interrupts enabled

		WDTCTL =  WDT_ARST_250;				// Reset Watchdog timer: ACLK 250ms

		pbState = GetPushButtonState();
		FlashLED(FLASH_BLINK);

		if (nCount == 0) {					// Wait for all PBs to be released
			if (pbState == 0)
				nCount = 1;
		}

		if (nCount && (pbState & 0x80)) {	// PB state changed
			i = GetDigit(pbState);

			if (i == 0xfe){					// Set to defaults and return to normal
				cv.radioChannel = flashcv.radioChannel;
				cv.locoAddress = flashcv.locoAddress;
				break;
			}

			if (i != 0xff) {				// Digit entered, store appropriately
				switch (nCount) {
				case 1:
					cv.radioChannel = i * 10;
					break;
				case 2:
					cv.radioChannel += i;
					break;
				case 3:
					cv.locoAddress = (unsigned int)(i * 100);
					break;
				case 4:
					cv.locoAddress += (unsigned int)(i * 10);
					break;
				case 5:
					cv.locoAddress += (unsigned int)i;
					break;
				}
				nCount++;
				FlashLED(FLASH_LATCH);
			}
		}
	} while(nCount < 6);

	if (nCount == 6) {	// Entered 6 digits, so save these to flash
		UpdateFlash();
	}
}

/*
 * GetDigit
 *
 * Returns 0-9 based on push buttons press, or 0xff if nothing valid set
 * Return 0xfe if both PB4 & PB5 pressed - abort and return to defaults
 *
 * 		Alone	+PB4	+PB5
 * PB0	0		4		8
 * PB1	1		5		9
 * PB2	2		6		-1
 * PB3	3		7		-1
 */
unsigned char GetDigit (unsigned char pbState)
{
	unsigned char i = 0xff;

	if ((pbState & PUSH4) && (pbState & PUSH5))
		return 0xfe;

	if (pbState & PUSH0)
		i = 0;
	else if (pbState & PUSH1)
		i = 1;
	else if (pbState & PUSH2)
		i = 2;
	else if (pbState & PUSH3)
		i = 3;

	if (i != 0xff) {
		if (pbState & PUSH4)
			i += 4;
		else if (pbState & PUSH5)
			i += 8;
	}

	if (i>9)
		i = 0xff;

	return i;
}

/*
 * UpdateFlash
 *
 * Write the current settings from cv to the flash memory flashcv
 */
void UpdateFlash(void)
{
	__bic_SR_register(GIE);        	// interrupts disabled
	WDTCTL = WDTPW + WDTHOLD;		// Turn off watchdog

	FCTL1 = FWKEY | ERASE;			// Erase individual 64-byte segment
	FCTL2 = FWKEY | FSSEL_1 | 16;	// MCLK(8MHz) / (16+1) = 470kHz flash clock
	FCTL3 = FWKEY;					// Unlock segment

	*((unsigned char *)&flashcv) = 0; // Dummy write to initiate segment erase

	FCTL1 = FWKEY | WRT;			// Write

	memcpy((void*)&flashcv, &cv, sizeof(struct configVariables));

	FCTL1 = FWKEY;					// End writing
	FCTL3 = FWKEY | LOCK;			// Lock segment

    WDTCTL =  WDT_ARST_250;			// Reset Watchdog timer: ACLK 250ms
	__bis_SR_register(GIE);    		// interrupts enabled
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


