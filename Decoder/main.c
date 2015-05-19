/*
Copyright (c) 2015, Brian Harrison
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
 * DCC Decoder application for MSP430G2553 with Anaren Radio
 * Note radio jumper set for GD0 on P1.1
 *
 ***********************************************************************************
 * Brian Harrison May, 2015 - briandharrison@gmail.com
 ***********************************************************************************
 * ACLK is at 32 kHz, 31us per cycle  (8192 cycles = 250ms)
 * SMCLK is at 8MHz, 125ns per cycle
 *
 * Watchdog timer will reset system every 250ms, unless it is reset
 * Timer0 is 125ns cycle, set to trigger on falling edge of the radio data
 * Timer1 is 250ns cycle used to wake up main loop every 4ms
 *
 */

#include  "msp430g2553.h"
#include  "CC1101.h"

#define byte	unsigned char
#define uint	unsigned int

// P1 Connections
#define     LED1                BIT0		// Steady when receiving, otherwise flashes
#define     GDO0                BIT1
#define     AUDIO	            BIT2
#define     FWD_LIGHT           BIT3
#define     REV_LIGHT         	BIT4
#define     SCLK                BIT5		// To radio
#define     MISO                BIT6		// To radio
#define     MOSI                BIT7		// To radio

// P2 connections
#define     CAB_LIGHT			BIT0		//
#define     BELL                BIT1 		//
#define     HORN                BIT2 		//
#define     SENSOR              BIT3		// May be used as input or...
#define		EXTRA_OUT			BIT3		// output instead of sensor
#define     MOTORA              BIT4 		// Output to IN1 of power controller
#define     MOTORB              BIT5		// Output to IN2 of power controller
#define     ALTRADIO            BIT6		// Ground to select alternative radio channel
#define     CSN                 BIT7 		// To radio

// DCC definitions
#define		DCC_BROADCAST_ADDR	0			// DCC broadcast address
#define		DCC_INST_TYPE_MASK	0xE0		// Mask for top three bits containing instruciton type
#define		DCC_DECODER			0x00
#define		DCC_ADV_OP			0x20
#define		DCC_REV_SPEED		0x40
#define		DCC_FWD_SPEED		0x60
#define		DCC_FN_GRP_1		0x80
#define		DCC_FN_GRP_2		0xA0
#define		DCC_FUTURE			0xC0
#define		DCC_CV_ACCESS		0xE0

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

// Constants for motor control
#define	START_SPEED	50
#define STOP_SPEED	40
#define	ACCEL_RATE	4
#define	DECEL_RATE	4
#define	HEAVY_ACCEL_RATE	1
#define	HEAVY_DECEL_RATE	2
#define FORWARDS	0
#define	REVERSE		1

#define	SECONDS		250			// 250 * 4ms (loop time) = 1 second


// Following constants can be modified by changing the values in the HEX file
#pragma SET_DATA_SECTION(".infoC")

const uint defLocoAddress = 3;	// Change this to desired value
const byte defRadioChannel1=2;	// Set to desired radio channel
const byte defRadioChannel2=4;	// Alternatve radio channel

// Map the decoder capabilities to DCC functions
const byte fLights = 0;			// DCC F0 for front/back headlight on/off
const byte fBell = 1;			// DCC F1 for bell
const byte fHorn = 2;			// DCC F2 for horn
const byte fExtra = 0xff;		// Change to 3 to use as output on DCC F3
const byte fAuto = 5;			// DCC F5 enables 'automatic' mode
const byte fShutdown = 6;		// DCC F6 to initiate a system shutdown
const byte fMute = 8;			// DCC F8 turns off sound
const byte fHeavy = 9;			// DCC F9 enables 'heavy' momentum
const byte fCabLight = 12;		// DCC F12 turns on/off cab light


#pragma SET_DATA_SECTION()



uint locoAddress;
int desiredSpeed=0;
int speed=0;
int autoSpeed;				// Speed when on automatic

bool dccFunctions[13];

enum recState { preamble=0, getbyte=1, startendbit=2} receiveState;
struct buffer {
	bool ready;
	byte data[8];
	uint count;
} receiveBuf;
uint receiveData, receiveBit;
uint receivedPackets;	// Counters for debugging

byte ledCounter;
uint timeoutCount;
bool bPower = true;
bool bOutputs = false;

enum timerEnum {
	timerNone = 0,
	timerClicking = 1,
	timerDelaying = 2,
	timerStarting = 3
} timerReason;
uint tCount = 0;	// Counter for timer
byte clickCount = 0;


void Initialize(void);
void InitializeTimers(void);
void InitializePorts(void);
void ProcessData(void);
void ProcessCommand(byte instruction);
void SetRadioChannel(bool bReset);
void CheckTimeout(void);
void CheckPowerOff(void);
void OutputsOff(void);
void SetTimer(uint t, enum timerEnum mode);
void CheckTimer(void);
void SetupRadioReceive(void);
void InitializeRadio(void);
inline void ClearBuffer(struct buffer* buf);
void SendBytes(const byte* bytes, int byteCount);
void Send2Bytes(byte b1, byte b2);
inline void SendByte(byte b);
void SetOutputs(void);
void SetSpeed(void);
int  GetSpeed(byte data);


/*
 * Main loop
 *
 * Initialize and loop waiting for interrupt every 4ms
 * Loop determines if any data received from radio and, if so,
 * processes the command.  It then checks for timed events
 * used in automatic mode, and finally checks the timer
 * that will shut down the decoder if no packets are received.
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
												// Wait for timer 1 CCR0 to turn on CPU

		WDTCTL =  WDT_ARST_250;					// Reset Watchdog timer: ACLK 250ms

		ProcessData();
		SetOutputs();
		SetSpeed();
		SetRadioChannel(false);
		CheckTimer();
		CheckTimeout();
		CheckPowerOff();
	}
}

/*
 * Initialize
 *
 * One time system initialization after power up
 */
void Initialize(void)
{
	int i;
	for (i=6665; i>0; i--) { }		// Delay loop

	InitializeTimers();
	InitializePorts();

	locoAddress = defLocoAddress;

	timeoutCount = 0;
	ledCounter = 252;
	receivedPackets = 0;

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

    // Basic system clock at 8MHz, i.e. 125ns per cycle
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
	TA1CTL = TASSEL_2 | MC_1 | ID_1;			// SMCLK, /2, Up to CCR0
	TA1CCR0 = 256;								// 250ns*256 = 64us (16kHz)
	TA1CCTL0 = OUTMOD_0 | CCIE;					// Output low
	TA1CCTL2 = OUTMOD_2;						// PWM mode toggle reset

}

/*
 * InitializePorts
 *
 * Set up MSP430 I/O ports
 * Several are used for peripheral I/O to connect to radio
 */
void InitializePorts(void)
{
	P1OUT = GDO0;						// GDO0 high, all other outputs low
	P1REN = GDO0;						// Pull GDO0 up
	P1SEL  = MOSI | MISO | SCLK | GDO0;	// Primary peripheral GDO0,
	P1SEL2 = MOSI | MISO | SCLK;		// secondary for MOSI, MISO, SCLK
	P1DIR = LED1 | AUDIO | FWD_LIGHT | REV_LIGHT;	// Output ports

	P2SEL = 0;							// All ports I/O
	if (fExtra == 0xff) {				// Use SENSOR as an input
		P2OUT = CSN | ALTRADIO;	// Other ports low
		P2REN = ALTRADIO | SENSOR;			// Pullup radio select input
		P2DIR = CAB_LIGHT | BELL | HORN | MOTORA | MOTORB | CSN;
		P2IE  = SENSOR | ALTRADIO;			// Enable interrupts for sensor & alt radio
		P2IES = SENSOR | ALTRADIO;			// Trigger sensor interrupt on falling edge
	} else {							// Use EXTRA_OUT as an output
		P2OUT = CSN | ALTRADIO;				// Other ports low
		P2REN = ALTRADIO;					// Pullup radio select input
		P2DIR = CAB_LIGHT | BELL | HORN | EXTRA_OUT | MOTORA | MOTORB | CSN;
	}

	P3REN = 0xff;						// Pull up all P3 ports (not used)
}

/*
 * unit GetPacketAddress(byte1, byte2)
 * Returns the loco address that is the target of the received command
 *
 * Determine the DCC loco address received in the radio packet.
 * Usually the loco address is in the first byte received (address < 192),
 * but some addresses (192-231) are spread over 2 bytes
 */
uint GetPacketAddress(byte byte1, byte byte2)
{
	uint address;

	address = (uint)byte1;
	if ((address >= 0x00C0) && (address < 0x00E8)) {	// Extended loco address 192-231
		address = (address & 0x003F)<<8 + (uint)receiveBuf.data[1];
	}
	return address;
}

/*
 * ProcessData
 *
 * Check if a packet has been received and, if so, process the DCC command
 * LED is flashed if no packets are recevied
 */
void ProcessData(void)
{
	uint address;
	byte instruction;

	if (receiveBuf.ready) {

		// Get loco address and, if matches, execute DCC command
		if (receiveBuf.count > 2) {
			address = GetPacketAddress(receiveBuf.data[0], receiveBuf.data[1]);
			if (address < 192)
				instruction = receiveBuf.data[1];
			else
				instruction = receiveBuf.data[2];

			if ((address == locoAddress) || (address == DCC_BROADCAST_ADDR)) {
				timeoutCount = 0;
				bOutputs = true;
				ProcessCommand(instruction);
			}
		}

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
 * ProcessCommand(instructions)
 *
 * Take action based on the received command
 */
void ProcessCommand(byte instruction)
{
	byte data = instruction & ~DCC_INST_TYPE_MASK;

	switch (instruction & DCC_INST_TYPE_MASK) {

	case DCC_DECODER:
		break;

	case DCC_ADV_OP:
		break;

	case DCC_REV_SPEED:
		if (!dccFunctions[fAuto])
			desiredSpeed = -GetSpeed(data);
		break;

	case DCC_FWD_SPEED:
		if (!dccFunctions[fAuto])
			desiredSpeed = GetSpeed(data);
		break;

	case DCC_FN_GRP_1:
		dccFunctions[0] = data&DCCF0;
		dccFunctions[1] = data&DCCF1;
		dccFunctions[2] = data&DCCF2;
		dccFunctions[3] = data&DCCF3;
		dccFunctions[4] = data&DCCF4;
		break;

	case DCC_FN_GRP_2:
		if (data&BIT4) {
			dccFunctions[5] = data&DCCF5;
			dccFunctions[6] = data&DCCF6;
			dccFunctions[7] = data&DCCF7;
			dccFunctions[8] = data&DCCF8;
		} else {
			dccFunctions[9]  = data&DCCF9;
			dccFunctions[10] = data&DCCF10;
			dccFunctions[11] = data&DCCF11;
			dccFunctions[12] = data&DCCF12;
		}
		break;

	case DCC_CV_ACCESS:
		break;

	}

}

/*
 * GetSpeed
 *
 * Interpret DCC speed command, which returns a value 0-31
 * Map this speed through the speed table to get a value 0-255
 * which is then used by timer to generate PWM for motor
 *
 * Note speed is multiplied by 8 to support acceleration and
 * deceleration; the clock divdes by 8 before use
 */
int GetSpeed(byte data)
{
	const static int speedTable[32] = {
			0, 8, 16, 24, 32, 40, 48, 56, 64,
			72, 80, 88, 96, 104, 112, 120, 128,
			136, 144, 152, 160, 168, 176, 184, 192,
			200, 208, 216, 224, 232, 240, 248
	};
	byte speed;

	speed = (data & 0x0F)*2;
	if (data & 0x10)
		speed++;
	return speedTable[speed]<<3;
}

/*
 * SetOutputs()
 *
 * Set the MSP output pins based on the received commands
 *
 */
void SetOutputs(void)
{
	if (!bOutputs)
		return;

	// Set forward & reverse lights
	CLEAR_BIT(P1OUT, FWD_LIGHT);
	CLEAR_BIT(P1OUT, REV_LIGHT);
	if (fLights < sizeof(dccFunctions)) {
		if (dccFunctions[fLights]) {			// Lights not turned off
			if (speed > 1)
				SET_BIT(P1OUT, FWD_LIGHT);
			else if ((ledCounter&0x03) == 0)	// Make forward light dim
				SET_BIT(P1OUT, FWD_LIGHT);

			if (speed < 0)
				SET_BIT(P1OUT, REV_LIGHT);
		}
	}

	// Set audio mute
	if (fMute < sizeof(dccFunctions)) {
		if (dccFunctions[fMute])
			CLEAR_BIT(P1OUT, AUDIO);
		else
			SET_BIT(P1OUT, AUDIO);
	}

	// Set cab light
	if (fCabLight < sizeof(dccFunctions)) {
		CLEAR_BIT(P2OUT, CAB_LIGHT);
		if ((dccFunctions[fCabLight])
		&&  (!dccFunctions[fAuto] || ((speed<2)&&(speed>-2))))	// Cab light on when halted in auto
			SET_BIT(P2OUT, CAB_LIGHT);
	}

	// Set bell
	if (fBell < sizeof(dccFunctions)) {
		if (dccFunctions[fBell])
			SET_BIT(P2OUT, BELL);
		else
			CLEAR_BIT(P2OUT, BELL);
	}

	// Set horn
	if (fHorn < sizeof(dccFunctions)) {
		if (dccFunctions[fHorn])
			SET_BIT(P2OUT, HORN);
		else
			CLEAR_BIT(P2OUT, HORN);
	}

	// Set extra output
	if (fExtra < sizeof(dccFunctions)) {
		if (dccFunctions[fExtra])
			SET_BIT(P2OUT, EXTRA_OUT);
		else
			CLEAR_BIT(P2OUT, EXTRA_OUT);
	}
}

/*
 * SetSpeed
 *
 * Compare the current speed with desiredSpeed and accelerate or decelerate to match
 * Note that speed jumps from zero to START_SPEED to start at level where motor will
 * turn, and when decelerating will drop from STOP_SPEED to zero to prevent motor stall
 *
 * If stopping, when speed reaches zero the bStopped flag is set and, if the bChangeDir
 * flag is set, then direction is toggled.
 */
void SetSpeed(void)
{
	if (!bOutputs)
		return;

	if (speed != desiredSpeed) {
		if (speed < desiredSpeed) {
			if (speed == 0)
				speed += START_SPEED;
			if (dccFunctions[fHeavy])		// Heavy load
				speed += HEAVY_ACCEL_RATE;
			else
				speed += ACCEL_RATE;
			if (speed > desiredSpeed)
				speed = desiredSpeed;
		} else {
			if (dccFunctions[fHeavy])		// Heavy load
				speed -= HEAVY_DECEL_RATE;
			else
				speed -= DECEL_RATE;
			if ((desiredSpeed == 0) && (speed < STOP_SPEED))
				speed = 0;
			if (speed < desiredSpeed)
				speed = desiredSpeed;
		}
	}

	if (speed>=0) {		// Forwards
		CLEAR_BIT(P2OUT, MOTORA);
		SET_BIT(P2OUT, MOTORB);
		SET_BIT(P2SEL, MOTORA);
		CLEAR_BIT(P2SEL, MOTORB);
		TA1CCR2 = speed>>3 & 0xff;			// Set PWM to current speed
	} else {
		CLEAR_BIT(P2OUT, MOTORB);
		SET_BIT(P2OUT, MOTORA);
		SET_BIT(P2SEL, MOTORB);
		CLEAR_BIT(P2SEL, MOTORA);
		TA1CCR2 = (-speed)>>3 & 0xff;		// Set PWM to current speed
	}
}


/*
 * CheckTimeout
 *
 * Make sure trains don't run away if we lose contact with the transmitter.
 * Ignore timeout if we are in automatic mode
 *
 * If no packets received for 4 seconds then stop.
 * If no packets received for 34 seconds then turn off all outputs
 * If no packets are recevied for 4 minutes then trigger complete shutdown,
 * disabling radio.  Only the sensor interrupt can wake system from this state.
 */
void CheckTimeout(void)
{
	if (!dccFunctions[fAuto]) {
		timeoutCount++;
		if (timeoutCount >= 60000)		// 60,000 * 4ms = 4 minutes
			dccFunctions[fShutdown] = true;	// Shut down the power

		else if (timeoutCount >= 8192)	// 8192*4ms = 33s after last packet
			OutputsOff();

		else if (timeoutCount >= 1024)	// 1024*4ms = 4s
			desiredSpeed = 0;
	}
}

/*
 * CheckPowerOff()
 *
 * If shutdown flag is set then power off all outputs, turn off watchdog timer and
 * stop system clock, and enter loop waiting for sensor interrupt.
 */
void CheckPowerOff(void)
{
	if (dccFunctions[fShutdown]) {
		bPower = false;					// Immediate stop and turn off all outputs
		TA0CCTL1 = 0;					// Timers off
		TA1CCTL1 = 0;
		dccFunctions[fShutdown] = 0;	// Don't shutdown again after revival
		speed = 0;
		OutputsOff();
		CLEAR_BIT(P1OUT, LED1);
		CLEAR_BIT(P1DIR, GDO0);
		SendByte(CC1101_SIDLE);			// Exit TX/RX, turn off freq. syth
		SendByte(CC1101_SPWD);			// Power down when CSN high
		WDTCTL = WDTPW + WDTHOLD;		// Turn off watchdog and wait for interrupt

		do {
			__bis_SR_register(LPM3_bits + GIE);		// LPM3 with interrupts enabled
		} while (!bPower);				// Repeat until sensor enables power

		WDTCTL = WDT_ARST_250;			// Resume watchdog
		timeoutCount = 0;
		InitializeTimers();
		InitializeRadio();
		SetupRadioReceive();
		SetRadioChannel(true);

	}
}

/*
 * OutputsOff()
 *
 * Turn off all outputs (conserves power)
 */
void OutputsOff(void)
{
	if (bOutputs) {
		bOutputs = false;
		CLEAR_BIT(P1OUT, AUDIO | FWD_LIGHT | REV_LIGHT);
		CLEAR_BIT(P2OUT, CAB_LIGHT | BELL | HORN | MOTORA | MOTORB);
		CLEAR_BIT(P2SEL, MOTORA | MOTORB);
	}
}

/*
 * setTimer (t, mode)
 *
 * t is used to count the number of 4ms main loop cycles have elapsed
 * mode determines what action to take when t reaches zero
 */
void SetTimer(uint t, enum timerEnum mode)
{
	tCount = t;
	timerReason = mode;
}

/*
 * CheckTimer()
 *
 * If auto mode is enabled and the timer reaches zero then take action based
 * on the reason for the timer.
 */
void CheckTimer(void)
{
	if (dccFunctions[fAuto] && tCount && (--tCount == 0)) {
		switch (timerReason) {

		case timerClicking:  				// Did we pick up a motor control click?
			if (speed) {    				// Clicks while running
				if (clickCount > 1)	  		// Multiple click - reverse
					autoSpeed = -speed;
				else
					autoSpeed = speed;	// Save the speed we were running at
				desiredSpeed = (autoSpeed>0)? 1 : -1; // Ensure appropriate headlight on
				SetTimer(10*SECONDS, timerDelaying);  // Delay then continue
			}
			clickCount = 0;
			break;

		case timerDelaying:
			desiredSpeed = autoSpeed;
			SetTimer(5*SECONDS, timerStarting);
			break;

		case timerStarting:
			SetTimer(0,timerNone);
			clickCount = 0;				// Ignore any clicks while starting up
			break;

		default:
			break;
		}
	}
}

/*
 * SetRadioChannel
 *
 * Choose a readio channel; default channel 2, or channel 4 if alt radio pin
 * is grounded.
 *
 * If channel has changed then reconfigure radio
 *
 * The frequencies supported by the radio:
 *   0: 921.37MHz;	 1: 919.87MHz;	 2: 915.37MHz;	 3: 912.37MHz;
 *   4: 909.37MHz;	 5: 907.87MHz;	 6: 906.37MHz;	 7: 903.37MHz;
 */
void SetRadioChannel(bool bReset)
{
	static byte currentChannel;
	static const byte channelSelect[] =
			{0x4B, 0x45, 0x33, 0x27, 0x1B, 0x15, 0x0F, 0x03};
	byte channel;

	if (bReset)
		currentChannel = 0xff;

	channel = (P2IN & ALTRADIO)? defRadioChannel1 : defRadioChannel2;

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
	const static byte initSequence[] = {
	0x40, 0x2E, 0x2E, 0x0D, 0x07, 0xD3, 0x91, 0xFF,
	0x04, 0x32, 0x00, 0x4B, 0x06, 0x00, 0x21, 0x6E,
	0x2C, 0xBA, 0x84, 0x00, 0x23, 0x2F, 0x47, 0x07,
	0x30, 0x18, 0x16, 0x6C, 0x03, 0x40, 0x91, 0x87,
	0x6B, 0xFB, 0x56, 0x10, 0xE9, 0x2A, 0x00, 0x1F,
	0x40, 0x00, 0x89, 0x7F, 0x63, 0x81, 0x35, 0x09 };

	SendBytes(initSequence, sizeof(initSequence));

	ClearBuffer(&receiveBuf);

}

/*
 * ClearBuffer
 */
inline void ClearBuffer(struct buffer* buf)
{
	uint i;

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
void SendBytes(const byte* bytes, int byteCount)
{
	uint i;

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
inline void SendByte(byte b)
{
	SendBytes(&b, 1);
}

/*
 * Send2Bytes
 */
void Send2Bytes(byte b1, byte b2)
{
	byte bytes[2];
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
	byte checksum;

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
				receiveBuf.data[(receiveBuf.count)++] = (byte)receiveData;
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
 * Timer1A0
 *
 * Timer1 CCR0 overflows every 64us (16kHz)
 * Divide this by 64 and awaken main loop every 4ms
 *
 * This timer is also used for the PWM output to the motor
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1A0 (void)
{
	static char counter = 0;

	counter -= 4;
	if (counter == 0)
		__bic_SR_register_on_exit(CPUOFF);        // Return to active mode
}


/*
 * PORT2_ISR() - Handle I/O port interrupt for input sensor
 */
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR (void)
{
	if (bPower) {
		if (P2IFG & SENSOR) {
			if (timerReason != timerStarting) {  // Ignore sensor while starting up
				++clickCount;
				SetTimer(1*SECONDS, timerClicking);
			}
		}

	} else {		// Powered off
		if (P2IFG & (SENSOR | ALTRADIO)) {
			bPower = true;
			__bic_SR_register_on_exit(LPM3_bits);        // Return to active mode
		}
	}
	P2IFG=0;
}
