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

// Use the internal clock
#define F_CPU 16000000UL

// Note: The TMS6100 was mask programmed for either 1-bit or 4-bit data
// transfer.  This emulation is for use with the TMS5220 VSP which only
// supports 1-bit mode.  Therefore 4-bit data is not emulated.

// This code has only been tested for use with the TMS5220 as a phrase ROM
// (PHROM) - If you use it with another device, your millage may vary :)

// Global includes
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

// Include the PHROM data image
#include "romdata.h"

// Include the hardware mapping
#include "hardwaremap.h"

// Some useful definitions
#define FALSE	0
#define TRUE	1

// Structure for holding the current state of the TMS6100
// Note: All variables need to be 'volatile' as they are
// used within the interrupt handling routines.
volatile struct tms6100State {
	volatile uint32_t address;					// The current address the ROM is pointing to
	volatile uint32_t chipSelectNumber;			// The chip identifier sent by the host

	volatile uint8_t loadAddressNibble;			// The position of the address nibble we are currently waiting for (0-4)
	volatile uint8_t validAddressLoadedFlag;	// Flag indicating if there is a valid address loaded
} tms6100;

// Initialise the AVR hardware
void initialiseHardware(void)
{
	// Set M0 and M1 pins to input and turn off weak pull-ups
	TMS6100_M0_DDR &= ~TMS6100_M0;
	TMS6100_M0_PORT &= ~TMS6100_M0;
	TMS6100_M1_DDR &= ~TMS6100_M1;
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
	tms6100.address = 0;
	tms6100.loadAddressNibble = 0;
	tms6100.validAddressLoadedFlag = FALSE;
	
	// Initialise the SPI pins
	// (MISO configured by ADD8)
	TMS6100_MOSI_DDR &= ~TMS6100_MOSI; // Input
	TMS6100_SCK_DDR &= ~TMS6100_SCK; // Input
	TMS6100_SS_DDR &= ~TMS6100_SS; // Input
}

// SPI buffer interrupt - called when the SPI buffer is empty
ISR(SPI_STC_vect)
{
	// Point to the next byte of the PHROM
	tms6100.address++;
	
	// Read the next byte to transfer from our current
	// valid address and place it in the SPI transmit buffer
	SPDR = pgm_read_byte(&(phromData[tms6100.address]));
}

// Function to handle external interrupt vector for the falling edge of M0
// Note: The falling edge of M0 indicates a READ DATA command
ISR(TMS6100_M0_INT_VECT)
{
	// There are two possible types of READ DATA command:
	// A 'dummy' read which indicates the TMS6100 should reset
	// and a real read which indicates the TMS6100 should transfer a bit of data
	
	// The reset can be detected because the TMS6100 requires 5 calls to the
	// LOAD ADDRESS command before a loaded address is considered 'valid',
	// so if we get a read, and there is not yet a valid address, the command
	// is a dummy read.
	
	// Check for a 'dummy' read (indicating reset requested)
	if (tms6100.validAddressLoadedFlag == FALSE)
	{
		// There is no valid loaded address... Reset the TMS6100 to a known state
		tms6100.address = 0;
		tms6100.loadAddressNibble = 0;
	}
	else
	{
		// We have a valid address so this is a 'real' READ DATA command
		
		// This is triggered because the host sends a single M0 pulse
		// to initiate the DATA READ command (and this pulse is *not*
		// for data transfer) - so we can detect this pulse and use it
		// to turn on the SPI module for the actual (much higher speed)
		// data transfer

		// Set the ADD8 bus pin to output mode (this doubles as SPI MISO)
		TMS6100_ADD8_DDR |= TMS6100_ADD8;
			
		// Turn off the M0 interrupt (so we only react using the SPI module
		// from here on)
		EIMSK &= ~(1 << TMS6100_M0_INT);
			
		// Turn on the SPI module (slave mode, reverse data order, interrupt on,
		// sample on trailing edge)
		SPCR |= (1 << SPE) | (1 << DORD) | (1 << SPIE) | (1 << CPHA);
			
		// Fill the SPI buffer with the first byte
		SPDR = pgm_read_byte(&(phromData[tms6100.address]));
	}
}

// Function to handle external interrupt vector for the rising edge of M1
// Note: The rising edge of M1 indicates a LOAD ADDRESS command
ISR(TMS6100_M1_INT_VECT)
{
	uint32_t addressNibble = 0;
	
	// Since this could occur after an SPI transfer has
	// been in progress, we need to reset the SPI, switch
	// ADD8/MISO back to input and re-enable the M0
	// interrupt...
	
	// Turn the SPI off
	SPCR = 0;
	
	// Ensure there is no pending interrupt on M0
	// (clear the interrupt flag by writing a logical one)
	EIFR |= (1 << TMS6100_M0_INTF);
	
	// Enable the M0 interrupt
	EIMSK |= (1 << TMS6100_M0_INT);
	
	// Set the ADD8 bus pin to input mode
	TMS6100_ADD8_DDR &= ~TMS6100_ADD8;
	
	// Read the nibble from the address bus
	if ((TMS6100_ADD1_PIN & TMS6100_ADD1)) addressNibble += 1;
	if ((TMS6100_ADD2_PIN & TMS6100_ADD2)) addressNibble += 2;
	if ((TMS6100_ADD4_PIN & TMS6100_ADD4)) addressNibble += 4;
	if ((TMS6100_ADD8_PIN & TMS6100_ADD8)) addressNibble += 8;
	
	// If this is the first nibble of a new 20-bit address, clear the address register
	if (tms6100.loadAddressNibble == 0) tms6100.address = 0;
	
	// Store the address nibble in the correct position of the 20-bit address register
	if (tms6100.loadAddressNibble == 0) tms6100.address |= addressNibble << 0;
	if (tms6100.loadAddressNibble == 1) tms6100.address |= addressNibble << 4;
	if (tms6100.loadAddressNibble == 2) tms6100.address |= addressNibble << 8;
	if (tms6100.loadAddressNibble == 3) tms6100.address |= addressNibble << 12;
	if (tms6100.loadAddressNibble == 4) tms6100.address |= addressNibble << 16;
	
	// Increment the current address register nibble pointer and range check
	tms6100.loadAddressNibble++;
	
	// Was the received nibble the 5th and final nibble of an address?
	if (tms6100.loadAddressNibble > 4)
	{
		// 5th nibble of an address received - Address is now valid for use
		tms6100.validAddressLoadedFlag = TRUE;
		tms6100.loadAddressNibble = 0;
		
		// We get 20 bits of address data from the host in 5 nibbles...
		
		// The datasheet says to ignore the two most significant bits:
		tms6100.address &= 0x9FFFF; // Mask = 0b0011 1111 1111 1111 1111 = 0x9FFFF
		
		// The number of the selected chip is the 4 most significant bits of the remaining bits:
		// 0b11 1100 0000 0000 0000 = 0x3C000 >> 14
		tms6100.chipSelectNumber = (tms6100.address & 0x3C000) >> 14; // Should be 'FF' for the Acorn PHROM
		
		// Note: An original TMS6100 would be mask programmed with it's own chip number.
		// If you want to support multiple TMS6100 chips you could test it here.
		
		// Now we need to remove the chip select bits so we are left with only 14-bit address for this PHROM
		tms6100.address &= 0x3FFF; // 0b11 1111 1111 1111 = 0x3FFF
	}
	else
	{
		// We only have a partial address...
		
		// Mark the current address register as invalid
		tms6100.validAddressLoadedFlag = FALSE;
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
	
	// We need to interrupt on the M0 and M1 pins
	// using INT (external interrupts) which can be
	// configured to interrupt on either the rising
	// or falling edge of a pulse:
	
	// External interrupt on the falling edge of a M0 pulse
	EICRA |= (1 << TMS6100_M0_ISC1);
	
	// External interrupt on the rising edge of a M1 pulse
	EICRA |= (1 << TMS6100_M1_ISC1) | (1 << TMS6100_M1_ISC0);

	// Enable external interrupts for M0 and M1
	EIMSK |= (1 << TMS6100_M0_INT) | (1 << TMS6100_M1_INT);
	
	// Turn SPI off
	SPCR = 0; // Probably not required?
	
	// Enable interrupts globally
	sei();
	
	// Main processing loop	
    while (1) 
    {
		// Nothing to do here. Everything is performed
		// using interrupts.
	}
}

