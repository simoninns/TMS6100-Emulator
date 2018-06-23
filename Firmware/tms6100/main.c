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
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>

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

// Structure for holding the current state of the TMS6100
// Note: All variables need to be 'volatile' as they are
// used within the interrupt handling routines.
volatile struct stateStruct {
	volatile uint32_t address;					// The current address the ROM is pointing to
	volatile uint32_t bankSelectNumber;			// The chip identifier sent by the host

	volatile uint8_t loadAddressNibble;			// The position of the address nibble we are currently waiting for (0-4)
	volatile uint8_t validAddressLoadedFlag;	// Flag indicating if there is a valid address loaded
	
	volatile uint8_t readDataActive;			// Flag indicating that a read data command is active
	volatile uint8_t currentBit;				// Pointer to the current bit of data to be transmitted 
	volatile uint8_t currentByte;				// The current byte to transmit
	
	volatile uint8_t add8InputFlag;				// Flag indicating that ADD8 is an input (or output if false)
	volatile uint8_t bankActiveFlag;			// Flag indicating that this PHROM's bank is active
} state;

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
	state.address = 0;
	state.loadAddressNibble = 0;
	state.validAddressLoadedFlag = FALSE;
	state.readDataActive = FALSE;
	state.currentBit = 0;
	state.currentByte = 0;
	state.add8InputFlag = TRUE;
	state.bankActiveFlag = FALSE;
	
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

// SPI buffer interrupt - called when the SPI buffer is empty
ISR(SPI_STC_vect)
{
	uint32_t currentBank, localAddress;
	
	// Point to the next address in sequence
	state.address++;
	
	// Is the current address within this PROM's bank?
	currentBank = (state.address & 0x3C000) >> 14; // 0b 0011 1100 0000 0000 0000 = 0x03C000
	localAddress = (state.address & 0x3FFF); // 0b 0000 0011 1111 1111 1111 = 0x03FFF
	
	// Only send data if the address is valid for this PHROM
	if (currentBank == PHROM_BANK) SPDR = pgm_read_byte(&(phromData[localAddress]));
	else SPDR = 0xFF; // Output should be high when not in use
}

// Function to handle external interrupt vector for the falling edge of M0
// Note: The falling edge of M0 indicates a READ DATA command
void m0SignalHandler(void) // ISR(TMS6100_M0_INT_VECT)
{
	uint32_t currentBank, localAddress;
	
	// Are we processing a read data?
	if (state.readDataActive == TRUE) {
		//// If the current address is valid for this PHROM's bank
		//// set ADD8 as an output
		//if (state.bankActiveFlag == TRUE) {
			//// This PHROM's bank is active, ensure ADD8 is an output
			//if (state.add8InputFlag == TRUE) {
				//TMS6100_ADD8_DDR |= TMS6100_ADD8;
				//state.add8InputFlag = FALSE; // Output
			//}
			//} else {
			//// This PHROM's bank is inactive, ensure ADD8 is an input
			//if (state.add8InputFlag == FALSE) {
				//TMS6100_ADD8_DDR &= ~TMS6100_ADD8;
				//TMS6100_ADD8_PORT &= ~TMS6100_ADD8;
				//state.add8InputFlag = TRUE; // Input
			//}
		//}
		//
		//// If this PHROM's bank is active, write data
		//if (state.bankActiveFlag == TRUE) {
			//// Place the bit of data onto the ADD8 pin
			//uint8_t dataBit = 0;
			//if ((state.currentByte & (1 << state.currentBit)) == 0) dataBit = 0; else dataBit = 1;
			//if (dataBit == 0) TMS6100_ADD8_PORT &= ~TMS6100_ADD8; else TMS6100_ADD8_PORT |= TMS6100_ADD8;
		//}
		//
		//// Point to the next bit
		//state.currentBit++;
		//
		//// End of current byte?
		//if (state.currentBit > 7) {
			//state.currentBit = 0;
			//
			//// Increment the address.  Note: this action can move the address
			//// over the bank boundary.
			//state.address++;
			//
			//// Get the next byte to transmit
			//
			//// Is the current address within this PROM's bank?
			//currentBank = (state.address & 0x3C000) >> 14; // 0b 0011 1100 0000 0000 0000 = 0x03C000
			//localAddress = (state.address & 0x3FFF); // 0b 0000 0011 1111 1111 1111 = 0x03FFF
			//
			//// Only send data if the address (and bank) is valid for this PHROM
			//if (currentBank == PHROM_BANK) {
				//state.currentByte = pgm_read_byte(&(phromData[localAddress]));
				//state.bankActiveFlag = TRUE;
				//
				//// Show bank active in debug
				//DEBUG2_PORT |= DEBUG2;
			//} else {
				//state.currentByte = 0xFF; // Current byte does not belong to this PHROM's bank
				//state.bankActiveFlag = FALSE;
				//
				//// Show bank inactive in debug
				//DEBUG2_PORT &= ~DEBUG2;
			//}
		//}
	} else {
		// There are two possible types of READ DATA command:
		// A 'dummy' read which indicates the TMS6100 should reset
		// and a real read which indicates the TMS6100 should transfer a bit of data
			
		// The reset can be detected because the TMS6100 requires 5 calls to the
		// LOAD ADDRESS command before a loaded address is considered 'valid',
		// so if we get a read, and there is not yet a valid address, the command
		// is a dummy read.
			
		// Check for a 'dummy' read (indicating reset requested)
		if (state.validAddressLoadedFlag == FALSE) {
			// There is no valid loaded address... Reset the TMS6100 to a known state
			state.address = 0;
			state.loadAddressNibble = 0;
			state.currentBit = 0;
		} else {
			// We have a valid address so this is a 'real' READ DATA command
				
			// This is triggered because the host sends a single M0 pulse
			// to initiate the DATA READ command (and this pulse is *not*
			// for data transfer) - so we can detect this pulse and use it
			// to turn on the SPI module for the actual (much higher speed)
			// data transfer

			// Set read data active
			state.readDataActive = TRUE;
			
			// Show read data active in debug
			DEBUG1_PORT |= DEBUG1;
				
			// Get the first byte of data to transmit
				
			// Is the current address within this PROM's bank?
			currentBank = (state.address & 0x3C000) >> 14; // 0b 0011 1100 0000 0000 0000 = 0x03C000
			localAddress = (state.address & 0x3FFF); // 0b 0000 0011 1111 1111 1111 = 0x03FFF
				
			// Only send data if the address (and bank) is valid for this PHROM
			if (currentBank == PHROM_BANK) {
				state.currentByte = pgm_read_byte(&(phromData[localAddress]));
				state.currentBit = 0;
				state.bankActiveFlag = TRUE;
			} else {
				state.currentByte = 0xFF; // Current byte does not belong to this PHROM's bank
				state.currentBit = 0;
				state.bankActiveFlag = FALSE;
			}
			
			// Whilst read data is active, we need to interrupt on the leading edge of M0
			// Set external interrupt on the leading edge of a M0 pulse
			//EICRA |= (1 << TMS6100_M0_ISC1) | (1 << TMS6100_M0_ISC0);
			
			// Turn off the M0 interrupt (so we only react using the SPI module
			// from here on)
			EIMSK &= ~(1 << TMS6100_M0_INT);
			
			// Ensure there is no pending interrupt on M0
			// (clear the interrupt flag by writing a logical one)
			EIFR |= (1 << TMS6100_M0_INTF);
			
			// Set ADD8 to output
			if (state.add8InputFlag == TRUE) {
				TMS6100_ADD8_DDR |= TMS6100_ADD8;
				state.add8InputFlag = FALSE; // Output
			}
			
			// Wait for the M0 signal to clear (otherwise the SPI module will
			// start sending on the initial 'ready' pulse of M0)
			while((TMS6100_M0_PIN & TMS6100_M0));
			while(!(TMS6100_M0_PIN & TMS6100_M0));
			
			// Configure the SPI module - slave mode, reverse data order
			// SPI Mode 0 = Sample (Rising) / Setup (Falling)
			// Turn on the buffer empty interrupt
			SPCR = (1 << DORD) | (1 << SPIE);
			
			// Turn on SPI
			SPCR |= (1 << SPE);
			
			// Show SPI active in debug
			DEBUG2_PORT |= DEBUG2;
			
			// Fill the SPI buffer with the first byte
			
			// Is the current address within this PROM's bank?
			currentBank = (state.address & 0x3C000) >> 14; // 0b 0011 1100 0000 0000 0000 = 0x03C000
			localAddress = (state.address & 0x3FFF); // 0b 0000 0011 1111 1111 1111 = 0x03FFF
			
			// Only send data if the address is valid for this PHROM
			if (currentBank == PHROM_BANK) SPDR = pgm_read_byte(&(phromData[localAddress]));
			else SPDR = 0xFF; // Output should be high when not in use
		}
	}
}

// Function to handle external interrupt vector for the rising edge of M1
// Note: The rising edge of M1 indicates a LOAD ADDRESS command
void m1SignalHandler(void) // ISR(TMS6100_M1_INT_VECT)
{
	uint32_t addressNibble = 0;
	
	// Since this could occur after an SPI transfer has
	// been in progress, we need to reset the SPI, switch
	// ADD8/MISO back to input and re-enable the M0
	// interrupt...
	
	// Turn the SPI off
	SPCR = 0;
	
	// Cancel the read data command
	state.readDataActive = FALSE;
	state.bankActiveFlag = FALSE;
	
	// Show read data inactive in debug
	DEBUG1_PORT &= ~DEBUG1;
	
	// Show SPI inactive in debug
	DEBUG2_PORT &= ~DEBUG2;
	
	// Set external interrupt on the falling edge of a M0 pulse
	EICRA |= (1 << TMS6100_M0_ISC1);
	EICRA &= ~(1 << TMS6100_M0_ISC0);
	
	// Ensure there is no pending interrupt on M0
	// (clear the interrupt flag by writing a logical one)
	EIFR |= (1 << TMS6100_M0_INTF);
	
	// Set the ADD8 bus pin to input mode
	if (state.add8InputFlag == FALSE) {
		TMS6100_ADD8_DDR &= ~TMS6100_ADD8;
		TMS6100_ADD8_PORT &= ~TMS6100_ADD8;
		state.add8InputFlag = TRUE;
	}
	
	// Read the nibble from the address bus
	if ((TMS6100_ADD1_PIN & TMS6100_ADD1)) addressNibble += 1;
	if ((TMS6100_ADD2_PIN & TMS6100_ADD2)) addressNibble += 2;
	if ((TMS6100_ADD4_PIN & TMS6100_ADD4)) addressNibble += 4;
	if ((TMS6100_ADD8_PIN & TMS6100_ADD8)) addressNibble += 8;
	
	// If this is the first nibble of a new 20-bit address, clear the address register
	if (state.loadAddressNibble == 0) state.address = 0;
	
	// Store the address nibble in the correct position of the 20-bit address register
	if (state.loadAddressNibble == 0) state.address |= addressNibble << 0;
	if (state.loadAddressNibble == 1) state.address |= addressNibble << 4;
	if (state.loadAddressNibble == 2) state.address |= addressNibble << 8;
	if (state.loadAddressNibble == 3) state.address |= addressNibble << 12;
	if (state.loadAddressNibble == 4) state.address |= addressNibble << 16;
	
	// Increment the current address register nibble pointer and range check
	state.loadAddressNibble++;
	
	// Was the received nibble the 5th and final nibble of an address?
	if (state.loadAddressNibble > 4)
	{
		// 5th nibble of an address received - Address is now valid
		state.validAddressLoadedFlag = TRUE;
		state.loadAddressNibble = 0;
		
		// Show valid address in debug
		DEBUG0_PORT |= DEBUG0;
		
		// We get 20 bits of address data from the host in 5 nibbles...
		
		// The format is - 2 bits (ignored) - 18 bits address
		// Note: the 4 MS Bits are the bank select address
		state.bankSelectNumber = (state.address & 0x3C000) >> 14; // 0b 0011 1100 0000 0000 0000 = 0x03C000
		
		// The address includes the chip select bank
		state.address = (state.address & 0x3FFFF); // 0b 0011 1111 1111 1111 1111 = 0x3FFFF
	} else {
		// We only have a partial address...
		
		// Mark the current address register as invalid
		state.validAddressLoadedFlag = FALSE;
		
		// Show invalid address in debug
		DEBUG0_PORT &= ~DEBUG0;
	}
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
	
	// We need to interrupt on the M0 and M1 pins
	// using INT (external interrupts) which can be
	// configured to interrupt on either the rising
	// or falling edge of a pulse:
	
	// External interrupt on the falling edge of a M0 pulse
	EICRA |= (1 << TMS6100_M0_ISC1);
	
	// External interrupt on the rising edge of a M1 pulse
	EICRA |= (1 << TMS6100_M1_ISC1) | (1 << TMS6100_M1_ISC0);
	
	// Turn SPI off
	SPCR = 0; 
	
	// Set global interrupts enabled
	sei();
	
	// Main processing loop	
    while (1) 
    {
		// Monitor and service the M0 and M1 signals
		if ((EIFR & (1 << TMS6100_M1_INTF)) == (1 << TMS6100_M1_INTF)) {
			// M1 Signal flagged
			
			// Clear the interrupt flag by writing a logical one
			EIFR |= (1 << TMS6100_M1_INTF);
			
			// Handle the signal
			m1SignalHandler();
		}
		
		if ((EIFR & (1 << TMS6100_M0_INTF)) == (1 << TMS6100_M0_INTF)) {
			// M0 Signal flagged
			
			// Clear the interrupt flag by writing a logical one
			EIFR |= (1 << TMS6100_M0_INTF);
			
			// Handle the signal
			m0SignalHandler();
		}
	}
}

