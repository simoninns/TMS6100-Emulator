/************************************************************************
	main.c

    TMS6100 Emulator (for use with TMS5220 VSP)
    Copyright (C) 2018 Simon Inns

	This file is part of the TMS6100-Emulator.

    The TMS6100-Emulator is free software: you can
	redistribute it and/or modify it under the terms of the GNU
	General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your
	option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Email: simon.inns@gmail.com

************************************************************************/

// Note: The TMS6100 was mask programmed for either 1-bit or 4-bit data
// transfer.  This emulation is for use with the TMS5220 VSP which only
// supports 1-bit mode.  Therefore 4-bit data is not emulated.

// This code has only been tested for use with the TMS5220 as a phrase ROM
// (PHROM) - If you use it with another device, your millage may vary :)

// Global includes
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>

// Include the required PHROM data image.  Available options are:
//
// PHROM_ACORN - The Acorn Speech System PHROM data
// PHROM_US - The TI American speech PHROM data
//
#ifdef PHROM_ACORN
	// Acorn PHROM data
	#pragma message ("Using Acorn Speech System PHROM data")
	#include "romdata_acorn.h"
#elif PHROM_US
	// TI US PHROM data
	#pragma message ("Using American TI Speech System PHROM data")
	#include "romdata_us.h"
#else
	// No PHROM was defined (or an unknown PHROM)
	#pragma message ("Using Acorn Speech System PHROM data (default)")
	#include "romdata_acorn.h"
#endif

// Include the hardware mapping
#include "hardwaremap.h"

// Some useful definitions
#define FALSE	0
#define TRUE	1

// Variables for holding the current state of the TMS6100
uint32_t currentAddress;
uint8_t m0ReadyReceived;

uint8_t outputBuffer;
uint8_t outputBufferPointer;

// Initialise the AVR hardware
void initialiseHardware(void)
{
	// Set M0 and M1 pins to input and turn off weak pull-ups
	TMS6100_M0_DDR &= ~TMS6100_M0;
	TMS6100_M1_DDR &= ~TMS6100_M1;
	
	TMS6100_M0_PORT &= ~TMS6100_M0;
	TMS6100_M1_PORT &= ~TMS6100_M1;

	// Set the address bus to input and turn off weak pull-ups
	TMS6100_ADD1_DDR &= ~TMS6100_ADD1;
	TMS6100_ADD2_DDR &= ~TMS6100_ADD2;
	TMS6100_ADD4_DDR &= ~TMS6100_ADD4;
	TMS6100_ADD8_DDR &= ~TMS6100_ADD8;
	
	TMS6100_ADD1_PORT &= ~TMS6100_ADD1;
	TMS6100_ADD2_PORT &= ~TMS6100_ADD2;
	TMS6100_ADD4_PORT &= ~TMS6100_ADD4;
	TMS6100_ADD8_PORT &= ~TMS6100_ADD8;
	
	// Set CLK to input and turn off weak pull-ups
	TMS6100_CLK_DDR &= ~TMS6100_CLK;
	TMS6100_CLK_PORT &= ~TMS6100_CLK;
	
	// Initialise the TMS6100 emulation:
	
	// Set the initial address pointer
	currentAddress = 0x00;
	
	// Initial M0 signal received flag
	// Note: this indicates if we've recieved the first M0 'ready' signal
	// following an M1 signal
	m0ReadyReceived = FALSE;
	
	// Initialise the output buffer
	outputBuffer = 0xFF;
	outputBufferPointer = 0;
	
	// Initialise the SPI pins (no longer used in this firmware)
	// (MISO configured by ADD8)
	TMS6100_MOSI_DDR &= ~TMS6100_MOSI; // Input
	TMS6100_SCK_DDR &= ~TMS6100_SCK; // Input
	TMS6100_SS_DDR &= ~TMS6100_SS; // Input
	
	// Initialise the debug pins as outputs
	// and set to off
	DEBUG0_DDR |= DEBUG0;
	DEBUG1_DDR |= DEBUG1;
	DEBUG2_DDR |= DEBUG2;
	
	DEBUG0_PORT &= ~DEBUG0;
	DEBUG1_PORT &= ~DEBUG1;
	DEBUG2_PORT &= ~DEBUG2;
}

// Function to handle the rising edge of M0
// Note: The falling edge of M0 indicates a READ DATA command
void m0SignalHandler(void) //ISR(TMS6100_M0_INT_VECT)
{
	if (m0ReadyReceived == FALSE) {
		// Show M0 handler active in debug
		DEBUG0_PORT |= DEBUG0;
		
		// This is the first M0 pulse after a M1 pulse (the 'ready' pulse)
		m0ReadyReceived = TRUE;
		
		// Load the byte to be transmitted
		//uint32_t currentBank = (currentAddress & 0x3C000) >> 14; // 0b 0011 1100 0000 0000 0000 = 0x03C000
		uint32_t localAddress = (currentAddress & 0x3FFF); // 0b 0000 0011 1111 1111 1111 = 0x03FFF
		outputBuffer = pgm_read_byte(&(phromData[localAddress]));
		
		// Reset the buffer pointer
		outputBufferPointer = 0; // LSB to MSB
		
		// Set the ADD8 bus pin to output mode and set the pin high
		// (as this is what the original TMS6100 does)
		TMS6100_ADD8_DDR |= TMS6100_ADD8;
		TMS6100_ADD8_PORT |= TMS6100_ADD8;
		
		// Show M0 handler inactive in debug
		DEBUG0_PORT &= ~DEBUG0;
	} else {
		// Show M0 handler active in debug
		DEBUG1_PORT |= DEBUG1;
		// This is a data read M0 pulse
		
		// Set the data on the output pin (so it is valid when the falling edge of M0 occurs)
		if (outputBuffer & (1 << outputBufferPointer)) TMS6100_ADD8_PORT |= TMS6100_ADD8;
		else TMS6100_ADD8_PORT &= ~TMS6100_ADD8;
		
		// Increment the bit pointer
		outputBufferPointer += 1;
		
		// Show M0 handler inactive in debug
		DEBUG1_PORT &= ~DEBUG1;
	}
	
	// Check if we need to reload the output buffer
	if (outputBufferPointer == 8) {
		// Get the next byte to transmit
		currentAddress++;
				
		//uint32_t currentBank = (currentAddress & 0x3C000) >> 14; // 0b 0011 1100 0000 0000 0000 = 0x03C000
		uint32_t localAddress = (currentAddress & 0x3FFF); // 0b 0000 0011 1111 1111 1111 = 0x03FFF
				
		outputBuffer = pgm_read_byte(&(phromData[localAddress]));
		outputBufferPointer = 0;
	}
}

// Function to handle the rising edge of M1
// Note: The rising edge of M1 indicates a LOAD ADDRESS command
void m1SignalHandler(void)
{
	uint32_t addressNibble = 0;
	
	// Show M1 handler active in debug
	DEBUG2_PORT |= DEBUG2;
	
	// Set the ADD8 bus pin to input mode
	TMS6100_ADD8_DDR &= ~TMS6100_ADD8;
	
	// Read the nibble from the address bus
	if ((TMS6100_ADD1_PIN & TMS6100_ADD1)) addressNibble += 1;
	if ((TMS6100_ADD2_PIN & TMS6100_ADD2)) addressNibble += 2;
	if ((TMS6100_ADD4_PIN & TMS6100_ADD4)) addressNibble += 4;
	if ((TMS6100_ADD8_PIN & TMS6100_ADD8)) addressNibble += 8;
	
	// The current address register is 20-bits wide
	// Addresses are loaded by transferring 5 nibbles (5x 4-bits)
	// The first nibble is the least significant nibble
	
	// Shift the current address register right one nibble
	currentAddress = currentAddress >> 4;
	
	// Place the received nibble in the top of the current address register
	currentAddress |= (addressNibble << 16);
	
	// Reset the M0 ready received flag
	m0ReadyReceived = FALSE;
	
	// Show M1 handler inactive
	DEBUG2_PORT &= ~DEBUG2;
}

// Note:  The TMS6100 supports a 3rd command (INDIRECT ADDRESS) which is
// indicated by both M0 and M1 rising at the same time.  This isn't used
// by the TMS5220 VSP, so it's not implemented in this emulator.

// Main function
int main(void)
{
	// Initialise the hardware
	initialiseHardware();
	
	// Disable the watchdog timer (if set in fuses)
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	// Disable the clock divider (if set in fuses)
	clock_prescale_set(clock_div_1);
	
	// Main processing loop	
	uint8_t m0State = FALSE;
	uint8_t m1State = FALSE;
	uint8_t m0PreviousState = FALSE;
	uint8_t m1PreviousState = FALSE;
	
    while (1) {
		m0PreviousState = m0State;
		m1PreviousState = m1State;
		
		if (TMS6100_M0_PIN & TMS6100_M0) m0State = TRUE; else m0State = FALSE;
		if (TMS6100_M1_PIN & TMS6100_M1) m1State = TRUE; else m1State = FALSE;
		
		// React on the leading edge
		if (m0State == TRUE && m0PreviousState == FALSE) m0SignalHandler();
		if (m1State == TRUE && m1PreviousState == FALSE) m1SignalHandler();
	}
}

