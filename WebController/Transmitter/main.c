/*
Copyright (c) 2016, Brian Harrison
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
 * Recieves commands from serial port to manage table of DCC commands to transmit
 *
 ***********************************************************************************
 * Brian Harrison March, 2016 - briandharrison@gmail.com
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
#include "uart.h"

#define byte	unsigned char
#define uint	unsigned int

#define		memcpy		__builtin_memcpy

// P1 Connections
#define     LED1				BIT0	// out: Green LED and GDO2
#define     UART_RX				BIT1	// in:  UART receive
#define     UART_TX				BIT2	// out: UART transmit
#define     UNUSED_13			BIT3	//
#define     UNUSED_14			BIT4	//
#define     SCLK				BIT5	// out: To radio
#define     MISO				BIT6	// in:  I2C from radio UCB0
#define     MOSI				BIT7	// out: I2C to radio UCB0

// P2 connections
#define     UNUSED_20			BIT0	//
#define     UNUSED_21			BIT1	//
#define     UNUSED_22			BIT2	//
#define     UNUSED_23			BIT3	//
#define     UNUSED_24			BIT4 	//
#define     UNUSED_25			BIT5	//
#define     GDO0				BIT6	// in:  GDO0 = TA0.1 Timer0_A
#define     CSN					BIT7 	// out: To radio

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
#define		FLASH_OFF			5	// LED off after latch


struct configVariables {
	unsigned char radioChannel;	// Set to desired radio channel

};

// Following constants can be modified by changing the values in the HEX file
// Or by entering the 'settings' mode
#pragma SET_DATA_SECTION(".infoC")

const struct configVariables flashcv = {
		2								// Radio Channel
};

#pragma SET_DATA_SECTION()

struct configVariables cv;

#define LOCO_ENTRIES			10
struct locoEntry {
	unsigned int address;
	int speed;
	unsigned int functions;
	unsigned int count;
} locos[LOCO_ENTRIES];
#define UNUSED_ADDR			0xFFFF
#define ACCESSORY_OFFSET	256
#define	UNSPECIFIED			0x7FFF
#define MAX_SPEED			32


#define COMMAND_BUFFER_SIZE	20

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

unsigned int timerCount;

void Initialize(void);
void InitializeTimers(void);
void InitializePorts(void);
void SetRadioChannel(bool bReset);
void InitializeRadio(void);

int GetSerialCommand(char* pCmd);
void ProcessSerialCommand(const char* cmdBuffer);
void ParseCommand(const char* cmdBuffer, char* pCommand, int* pAddress, int* pSpeed, int* pFunctions);
int GetInt(const char* pBuffer, int* pCount);
void UpdateLocos(void);
void PrintLocos(void);
void PrintHex(const unsigned char c);
void PrintInt(int i);

unsigned char DCCSpeed(int speed);
unsigned char DCCFunction(int functions, int group);
void FlashLED(char rate);
void SetupAccessoryPacket(unsigned int address, bool clear, byte device);
void SetupDCCBuffer();
void TransmitData(void);

void UpdateFlash(void);
void SendBytes(const unsigned char* bytes, int byteCount);
void Send2Bytes(unsigned char b1, unsigned char b2);
inline void SendByte(unsigned char b);

const char strCommands[] = "\nCommands: a c e f l p r s x\n";
//		"a<addr>A<activate>D<device> Update accessory\n"
//		"c<addr> Clear loco\n"
//		"e<addr> Clear accessory\n"
//		"f Update Flash\n"
//		"l<addr>S<speed>F<func> Update loco\n"
//		"p Display locos\n"
//		"r<channel> Set radio channel\n"
//		"s Emergency Stop\n"
//		"x Clear all\n";

/*
 * Main loop
 *
 * Initialize and loop waiting for interrupt every 4ms
 */
void main(void)
{
	char cmdBuffer[COMMAND_BUFFER_SIZE];

	Initialize();
	__enable_interrupt();                     // Enable interrupts.

	uart_puts(strCommands);

	/* Main Application Loop */
	while(1) {
		__bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled
    											// Wait for timer 1 CCR1 to turn on CPU

		WDTCTL =  WDT_ARST_250;					// Reset Watchdog timer: ACLK 250ms

		// Check if a serial command has been recevied, and update table if so
		FlashLED(FLASH_OFF);
		if (GetSerialCommand(cmdBuffer)) {
			FlashLED(FLASH_LATCH);
			uart_puts(cmdBuffer);		// Echo back the received command
			uart_puts("\n");
			ProcessSerialCommand(cmdBuffer);
		}

		SendByte(CC1101_STX);					// In IDLE state: enable TX

		SetRadioChannel(false);
		SetupDCCBuffer();
		TransmitData();

//	    UpdateLocos();

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

	timerCount = 0;

	InitializeTimers();
	InitializePorts();

	InitializeRadio();

    uart_config_t config;
    /* Initialize UART to 9600 baud */
    config.baud = 9600;

    if (uart_init(&config) != 0) {
        while (1);
    }

    // Initialize loco table
	for (i=0;i<LOCO_ENTRIES;i++)
		locos[i].address = UNUSED_ADDR;

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
	P1SEL = MOSI | MISO | SCLK | UART_RX | UART_TX;	// Secondary mode for MOSI, MISO, SCLK, UART
	P1SEL2 = MOSI | MISO | SCLK | UART_RX | UART_TX;// secondary for MOSI, MISO, SCLK, UART
	P1DIR = MOSI | SCLK | LED1;	// Outputs

	P2SEL = GDO0;
	P2OUT = CSN;				// High
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
 * GetSerialCommand
 *
 * Read characters from the UART
 * When a newline is recieved, copy received command to buffer and return true
 * Otherwise save command so far and return false
 */
char tmpBuffer[COMMAND_BUFFER_SIZE];

int GetSerialCommand(char* cmdBuffer)
{
	static int count = 0;
	int c;

	c = uart_getchar();
	while (c != -1) {
		tmpBuffer[count++] = (char)c;
		if ((c == '\n') || (c == '\r') || (count >= sizeof(tmpBuffer))) {
			tmpBuffer[count-1] = '\0';
			memcpy(cmdBuffer, tmpBuffer, count);
			count = 0;
			return true;
		}
		c = uart_getchar();
	}
	return false;
}

/*
 * ProcessSerialCommand
 *
 * Execute command based on the first character received in the buffer
 */
void ProcessSerialCommand(const char* cmdBuffer)
{
	char command;
	int address, speed, functions;
	struct locoEntry *pLoco, *pEmpty=NULL;

	ParseCommand(cmdBuffer, &command, &address, &speed, &functions);

	switch (command) {

	case 'x':	// "x"  Clear loco & accessory table
		for (pLoco=locos; pLoco<&locos[LOCO_ENTRIES]; pLoco++) {
			pLoco->address = UNUSED_ADDR;
		}
		break;

	case 'e':	// "e<address>"  Clear accessory entry
		address += ACCESSORY_OFFSET;	// Make address unique and process like a loco entry
	case 'c':	// "c<address>"  Clear loco entry
		for (pLoco=locos; pLoco<&locos[LOCO_ENTRIES]; pLoco++) {
			if (pLoco->address == address) {
				pLoco->address = UNUSED_ADDR;
				break;
			}
		}
		break;

	case 'a':	// "a<address>A<activate>D<device>"  Add or update accessory entry
		address += ACCESSORY_OFFSET;	// Make address unique and process like a loco entry
	case 'l':	// "l<address>S<speed>F<functions>"  Add or update loco entry
		// Find the  entry for this address
		for (pLoco=locos; pLoco<&locos[LOCO_ENTRIES]; pLoco++) {
			if (pLoco->address == address)
				break;
			if (!pEmpty && (pLoco->address == UNUSED_ADDR))
				pEmpty = pLoco;
		}
		if (pLoco>=&locos[LOCO_ENTRIES]) { // Didn't find address
			if (!pEmpty)				// No room for more addresses
				break;
			pLoco = pEmpty;
			pLoco->address = address;	// Set up new row
			if (speed == UNSPECIFIED)
				speed = 0;
			if (functions == UNSPECIFIED)
				functions = 1;			// Default to enabling F0
		}
		if (speed <= MAX_SPEED)
			pLoco->speed = speed;
		if (functions != UNSPECIFIED)
			pLoco->functions = functions;
		pLoco->count = 0;
		break;

	case 's':	// "s"  Stop all locos
		for (pLoco=locos; pLoco<&locos[LOCO_ENTRIES]; pLoco++) {
			if (pLoco->address != UNUSED_ADDR)
				pLoco->speed = 0;
		}
		break;

	case 'r':	// "r<channel>"  Set radio channel
		cv.radioChannel = address;
		break;

	case 'f':	// "f" Update flash
		UpdateFlash();
		break;

	case 'p':	// "p" Print Loco Table
		PrintLocos();
		break;

	default:	// Otherwise print list of valid commands
		uart_puts(strCommands);
		break;
	}
}

/*
 * ParseCommand
 *
 * Parse the command buffer into a character command plus three integers:
 *      <command><address>:<speed> <functions>
 */
void ParseCommand(const char* cmdBuffer, char* pCommand, int* pAddress, int* pSpeed, int* pFunctions)
{
	const char* pBuffer = cmdBuffer;
	int count;

	*pCommand = '\0';
	*pAddress = *pSpeed = *pFunctions = UNSPECIFIED;

	if ((*pBuffer == '\n') || (*pBuffer == '\0'))
		return;

	// First get command byte
	*pCommand = *pBuffer++;

	// Next get address if specified
	*pAddress = GetInt(pBuffer, &count);
	pBuffer += count;

	// Next get speed or activate if specified
	if ((*pBuffer == 'S')||(*pBuffer == 'A')) {
		pBuffer++;
		*pSpeed = GetInt(pBuffer, &count);
		pBuffer += count;
	}

	// Get functions or device if specified
	if ((*pBuffer == 'F')||(*pBuffer == 'D')) {
		pBuffer++;
		*pFunctions = GetInt(pBuffer, &count);
		pBuffer += count;
	}
}

/* GetInt
 *
 * Parse buffer for a decimal integer, skipping leading blanks
 * Return integer plus number of characters consumed
 */
int GetInt(const char* pBuffer, int* pCount)
{
	int result = 0;
	int negative = false;
	*pCount = 0;

	// Skip leading spaces
	while (*pBuffer == ' ') {
		pBuffer++;
		(*pCount)++;
	}

	// Look for leading sign
	if (*pBuffer == '-') {
		pBuffer++;
		(*pCount)++;
		negative = true;
	}

	// Scan through digits
	while ((*pBuffer>='0') && (*pBuffer<='9')) {
		result = result*10 + *pBuffer++ - '0';
		(*pCount)++;
	}

	if (negative)
		result = -result;

	return result;
}

/*
 * PrintLocos
 *
 * Output the loco table plus radio channel to the serial port
 */
void PrintLocos(void)
{
	struct locoEntry* pLoco;

	for (pLoco=locos; pLoco<&locos[LOCO_ENTRIES]; pLoco++) {
		if (pLoco->address == UNUSED_ADDR)	// Unused row
			continue;

		PrintInt(pLoco->address);
		uart_putchar('S');
		PrintInt(pLoco->speed);
		uart_putchar('F');
		PrintInt(pLoco->functions);
		uart_puts("\n");
	}
}

/*
 * PrintInt
 *
 * Convert a decimal integer to characters and output to the serial port
 */
void PrintInt(int i)
{
	char buff[10];
	char* p = buff;
	int shifter;

	if(i<0){
		*p++ = '-';
		i *= -1;
	}

	shifter = i;
	do { //Move to where representation ends
		++p;
		shifter = shifter/10;
	} while(shifter);
	*p = '\0';

	do{ //Move back, inserting digits as u go
		*--p = i%10 + '0';
		i = i/10;
	}while(i);

	uart_puts(buff);
}

/*
 * PrintInt
 *
 * Convert a byte to hex characters and output to the serial port
 */
void PrintHex(const unsigned char c)
{
	uart_putchar((c>>4)<10? (c>>4) + '0' : (c>>4) + 'A'-10);
	uart_putchar((c&0x0F)<10? (c&0x0F) + '0' : (c&0x0F) + 'A'-10);
}

/*
 * UpdateLocos
 *
 * Increment the counter on each entry in the loco table,
 * and remove any entries with counter > 60
 */
void UpdateLocos(void)
{
	struct locoEntry* pLoco;

	if ((timerCount&0xff) == 0)	{	// Update every second
		for (pLoco=locos; pLoco<&locos[LOCO_ENTRIES]; pLoco++) {
			if (pLoco->address == UNUSED_ADDR)	// Unused row
				continue;
			if (pLoco->count++ >= 60)		// Address unused for 60 seconds
				pLoco->address = UNUSED_ADDR;
		}
	}
}

/*
 * FlashLED
 *
 * Flash the LED at the specified rate
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
		SET_BIT(P1OUT, LED1);
		ledCounter = 0;
		break;

	case FLASH_OFF:
		if (bLatch)
			SET_BIT(P1OUT, LED1);
		break;
	}

	if (bLatch && (ledCounter & 0x08))
		bLatch = false;
}

/*
 * SetupAccessoryPacket
 *
 * Accessory packets have the format:
 * {preamble} 0 10AAAAAA 0 1AAACAAD 0 EEEEEEEE 1
 * Where C activate or deactivates device
 *       D specifies which device at this address (e.g. set or clear switch)
 *       Least significant 2 bits of address are byte 2 bits 1 & 2
 *       Bits 2-7 of address are in byte 1
 *       Most significant 3 bits of address (8-10) are in byte 2 bits 4-6, ones complement
 *       EEEEEEEE is the checksum
 */
void SetupAccessoryPacket(unsigned int address, bool activate, byte device)
{
	byte dev = (device & 0x01) | (((byte)address<<1) & 0x06);		// Device includes bottom two bits of address
	unsigned int addr = (address>>2) + 1;			// Address is remainder of address, plus one

	dccBufferIndex = 1;			// Start transmission at dccAddress1
	dccBuffer.dccAddress0 = 0;
	dccBuffer.dccAddress1 = 0x80 | (byte)(addr & 0x003F);	// Address bits 2-7

	dccBuffer.dccInstruction = ~(addr>>2);					// Top three bits of address in upper nibble
	dccBuffer.dccInstruction &= 0x70;
	dccBuffer.dccInstruction |= 0x80;
	if (activate)
		dccBuffer.dccInstruction |= 0x08;				// Set bit 3 to activate
	dccBuffer.dccInstruction |= dev & 0x07;			// Device in bottom three bits
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
void SetupDCCBuffer()
{
	static int nCounter = 0;
	static struct locoEntry *pLoco=locos;
	int n;

	// Find next loco entry to transmit
	if ((nCounter==0) || (pLoco->address == UNUSED_ADDR)) {
		for (n=0; n<LOCO_ENTRIES; n++) {
			if (++pLoco>=&locos[LOCO_ENTRIES])
				pLoco = locos;
			if (pLoco->address != UNUSED_ADDR)
				break;
		}
	}

	if (pLoco->address == UNUSED_ADDR) {
		// Send an idle packet
		dccBuffer.dccAddress0 = 0;
		dccBuffer.dccAddress1 = 0xFF;
		dccBufferIndex = 1;			// Start transmission at dccAddress1
		dccBuffer.dccInstruction = 0;

	} else if (pLoco->address >= ACCESSORY_OFFSET) {
		SetupAccessoryPacket(pLoco->address-ACCESSORY_OFFSET, pLoco->speed, pLoco->functions);
		nCounter = 0;
	} else {
		dccBuffer.dccAddress0 = 0;
		dccBuffer.dccAddress1 = pLoco->address;
		dccBufferIndex = 1;			// Start transmission at dccAddress1
		if (pLoco->address >= 0x64) {	// Loco has a 14-bit address?
			dccBuffer.dccAddress0 = pLoco->address>>8 | 0xC0;	// Set top two bits of most significant byte
			dccBufferIndex = 0;		// Start transmission at dccAddress0 for 2 address bytes
		}

		if (nCounter==0)
			dccBuffer.dccInstruction = DCCSpeed(pLoco->speed);
		else
			dccBuffer.dccInstruction = DCCFunction(pLoco->functions, nCounter);
		if (nCounter++>=3)
			nCounter = 0;			// Repeat cycle

	}
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
 * DCCFunction
 *
 * Return a DCC instruction byte for the specified function group
 */
unsigned char DCCFunction(int functions, int group)
{
	unsigned char instruction;

	switch (group) {
	case 1:	// Function group 1, F0 and F1-F4
		instruction = DCCFGROUP1 | ((functions>>1)&0x0F);
		if (functions&0x01)
			instruction |= DCCF0;
		break;

	case 2:	// Function group 2a, F5-F8
		instruction = DCCFGROUP2A | ((functions>>5)&0x0F);
		break;

	case 3:	// Function group 2b, F9-F12
		instruction = DCCFGROUP2B | ((functions>>9)&0x0F);
		break;
	}
	return instruction;
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


