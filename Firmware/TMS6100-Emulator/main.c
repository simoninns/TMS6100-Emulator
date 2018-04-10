/************************************************************************
	main.c

    Acorn TMS6100 PHROM Emulator
    Copyright (C) 2016 Simon Inns

	This file is part of the Acorn TMS6100 PHROM Emulator.

    The Acorn TMS6100 PHROM Emulator is free software: you can
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

#define F_CPU 16000000UL

// Note: The TMS6100 was mask programmed for either 1-bit or 4-bit data
// transfer.  This emulation is for use with the TMS5220 VSP which only
// supports 1-bit mode.  Therefore 4-bit data is not emulated.

// Note: This code is not synchronizing with the CLK signal... It relies on the fact that the
// AVR is exponentially fasted than the VSP chip, but this could be a seriously bad assumption?

// Include this define for weak pull-ups on the address bus pins
//#define ADDX_WEAK_PULLUPS

// Global includes
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

// Include the PHROM data
#include "tms6100romdata.h"

#define FALSE	0
#define TRUE	1

// Structure for holding the current state of the TMS6100
volatile struct tms6100State {
	volatile uint32_t address;					// The current address the ROM is pointing to
	volatile uint32_t chipSelectNumber;			// The chip identifier sent by the host

	volatile uint8_t loadAddressNibble;			// The position of the address nibble we are
												// currently waiting for (0-4)						
	volatile uint8_t validAddressLoadedFlag;	// Flag indicating if there is a valid address loaded
	
	// These variables are used to track the changing state of M0 and M1:
	volatile uint8_t m0CurrentState;			// The current state of the M0 pin
	volatile uint8_t m1CurrentState;			// The current state of the M1 pin
	volatile uint8_t m0PreviousState;			// The previous state of the M0 pin
	volatile uint8_t m1PreviousState;			// The previous state of the M1 pin
} tms6100;	

// Definitions for TMS6100 IO Pins

// M0 (PB1/SPI-SCLK)
#define TMS6100_M0_PORT		PORTB
#define TMS6100_M0_PIN		PINB
#define TMS6100_M0_DDR		DDRB
#define TMS6100_M0			(1 << 1)

// M1 (PB5) - Must be port B as we are using interrupt on change
#define TMS6100_M1_PORT		PORTB
#define TMS6100_M1_PIN		PINB
#define TMS6100_M1_DDR		DDRB
#define TMS6100_M1			(1 << 5)

// ADD1 (PD0)
#define TMS6100_ADD1_PORT	PORTD
#define TMS6100_ADD1_PIN	PIND
#define TMS6100_ADD1_DDR	DDRD
#define TMS6100_ADD1		(1 << 0)

// ADD2 (PD1)
#define TMS6100_ADD2_PORT	PORTD
#define TMS6100_ADD2_PIN	PIND
#define TMS6100_ADD2_DDR	DDRD
#define TMS6100_ADD2		(1 << 1)

// ADD4 (PD2)
#define TMS6100_ADD4_PORT	PORTD
#define TMS6100_ADD4_PIN	PIND
#define TMS6100_ADD4_DDR	DDRD
#define TMS6100_ADD4		(1 << 2)

// ADD8 (PB3/SPI-MISO)
#define TMS6100_ADD8_PORT	PORTB
#define TMS6100_ADD8_PIN	PINB
#define TMS6100_ADD8_DDR	DDRB
#define TMS6100_ADD8		(1 << 3)

// CLK (PB6) - Not currently used by the emulator
#define TMS6100_CLK_PORT	PORTB
#define TMS6100_CLK_PIN		PINB
#define TMS6100_CLK_DDR		DDRB
#define TMS6100_CLK			(1 << 6)

// Definitions for emulator pins (not connected to TMS6100 pins)

// DEBUG0 (PD7)
#define TMS6100_DEBUG0_PORT	PORTD
#define TMS6100_DEBUG0_PIN	PIND
#define TMS6100_DEBUG0_DDR	DDRD
#define TMS6100_DEBUG0		(1 << 7)

// DEBUG1 (PE6)
#define TMS6100_DEBUG1_PORT	PORTE
#define TMS6100_DEBUG1_PIN	PINE
#define TMS6100_DEBUG1_DDR	DDRE
#define TMS6100_DEBUG1		(1 << 6)

// !Slave select control (PB4)
#define TMS6100_SSC_PORT	PORTB
#define TMS6100_SSC_PIN		PINB
#define TMS6100_SSC_DDR		DDRB
#define TMS6100_SSC			(1 << 4)

// !Slave select pin (PB0)
#define TMS6100_SS_PORT		PORTB
#define TMS6100_SS_PIN		PINB
#define TMS6100_SS_DDR		DDRB
#define TMS6100_SS			(1 << 0)

// TMS6100 power-up reset
void tms6100PowerUpReset(void)
{
	// Initialise the AVR:
	
	// Disable UART modules
	UCSR1B = 0;
	
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
	
	// Configure the debug pins as output and turn pins off
	TMS6100_DEBUG0_DDR |= TMS6100_DEBUG0;
	TMS6100_DEBUG1_DDR |= TMS6100_DEBUG1;
	
	TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0;
	TMS6100_DEBUG1_PORT &= ~TMS6100_DEBUG1;
	
	// Configure the !Slave select control pin and turn pin on (!SS = off)
	// Note: this pin is connected to the !SS pin and is used by the 
	// emulator to enable the slave SPI locally or reset the SPI slave.
	TMS6100_SSC_DDR |= TMS6100_SSC;
	TMS6100_SSC_PORT |= TMS6100_SSC;
	
	// Set the !SS pin as an input and turn on weak pull-ups
	TMS6100_SS_DDR &= ~TMS6100_SS;
	TMS6100_SS_PORT &= ~TMS6100_SS;
	
	// Initialise the SPI module as slave to handle the M0 (serial clock
	// in) and ADD8 pin - SPI is disabled by the !Slave Select Control pin
	
	// From the ATmega32u4 datasheet:
	// When SS is driven high, all pins are inputs, and the SPI is passive, which
	// means that it will not receive incoming data. Note that the SPI logic will be reset once the SS pin
	// is driven high.
	
	// Disable slave SPI module
	SPCR &= ~(1 << SPE);
	
	// Disable slave SPI interrupt
	SPCR &= ~(1 << SPIF);
	
	// Initialise the TMS6100 emulation:
	tms6100.address = 0;
	tms6100.loadAddressNibble = 0;
	tms6100.validAddressLoadedFlag = FALSE;
	
	// Set up interrupt on change for M0 (PCINT3 on PB3)
	// and M1 (PCINT5 on PB5) and use the interrupt run the
	// TMS6100 emulation
	PCMSK0 |= (1 << PCINT1 | 1 << PCINT5); // Use PCINT1/PB1 and PCINT5/PB5
	PCICR |= (1<<PCIE0);   // Enable Pin change interrupt
	sei(); // Enable interrupts globally
}

// Function to handle SPI buffer interrupt vector
// Called whenever a transmit or receive is complete
ISR(SPI_STC_vect)
{
	// If the transmit buffer is empty, load the next byte from
	// the PHROM
	
	// Read the byte stored at the current PHROM address
	// Pass the byte to the SPI buffer
	SPDR = pgm_read_byte(&(phromData[tms6100.address]));
	
	// Move the address pointer to the next byte
	tms6100.address++;
}

// Function to handle the pin changed interrupt vector
ISR(PCINT0_vect)
{
	//unsigned char readByte;
	uint32_t addressNibble = 0;
	
	// Choose state depending on the status of M0 and M1
	// M0 L - M1 L = Idle (waiting for command)
	// M0 L - M1 H = Load address (waiting for address)
	// M0 H - M1 L = Read (sending bit)
	
	// The TMS6100 supports 3 different commands indicated by M0 and M1:
	//
	// The falling edge of M0 indicates a READ DATA command
	// The rising edge of M1 indicates a LOAD ADDRESS command
	// If both M0 and M1 rise at the same time it indicates an INDIRECT ADDRESS command
	
	// Read the current state of M0 and M1 from the pins
	if ((TMS6100_M0_PIN & TMS6100_M0)) tms6100.m0CurrentState = 1; else tms6100.m0CurrentState = 0;
	if ((TMS6100_M1_PIN & TMS6100_M1)) tms6100.m1CurrentState = 1; else tms6100.m1CurrentState = 0;
	
	// Process the commands
	
	// M0 = 0 (falling edge) and M1 = 0 = READ DATA command
	if ((tms6100.m0PreviousState == 1 && tms6100.m1PreviousState == 0) && (tms6100.m0CurrentState == 0 && tms6100.m1CurrentState == 0))
	{
		// There are two possible types of READ DATA command:
		// A 'dummy' read which indicates the TMS6100 should reset
		// and a real read which indicates the TMS6100 should transfer a bit of data
		
		// Whilst we have an invalid address we read the M0 pin directly.
		// Once a valid address is received we turn on the SPI slave module
		// and use the AVRs on board SPI to send the data bits synced with
		// the pulses on M0.  This is required otherwise the AVR cannot keep
		// up with the data transfer rate required from the TMS6100.
		
		// This works because the TMS6100 requires 5 calls to the LOAD ADDRESS command
		// before a loaded address is considered 'valid', so if we get a read, and there
		// is not a valid address, the command is a dummy read.
		
		// Check for a 'dummy' read (indicating reset requested)
		if (tms6100.validAddressLoadedFlag == FALSE)
		{
			// Pulse debug0 once
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			
			// There is no valid loaded address... Reset the TMS6100 to a known state
			tms6100.address = 0;
			tms6100.loadAddressNibble = 0;
		}
		else
		{
			// This is a 'real' read command:
			// M0 has pulsed high to indicate a transfer is beginning and
			// we have a valid address to read from...
			
			// Set the ADD8 bus pin to output mode
			TMS6100_ADD8_DDR |= TMS6100_ADD8;
			uint8_t dataBit = 0;
			uint8_t dataByte = pgm_read_byte(&(phromData[tms6100.address]));
			while(1)
			{
				// Wait for the first clock pulse
				while(!(TMS6100_M0_PIN & TMS6100_M0)); // wait for M0 to go high
				while((TMS6100_M0_PIN & TMS6100_M0)); // wait for M0 to go low
				// Latch the data
				if (((1 << dataBit) & dataByte) != 0) TMS6100_ADD8_PORT |= TMS6100_ADD8;  // 1
				else TMS6100_ADD8_PORT &= ~TMS6100_ADD8;  // 0
				
				dataBit++;
				if (dataBit > 7)
				{
					tms6100.address++;
					dataBit = 0;
					dataByte = pgm_read_byte(&(phromData[tms6100.address]));
				}
			}
			
			//// Disable the M0 interrupt on pin change
			//PCMSK0 &= ~(1 << PCINT1); // Disable PCINT1/PB1
			//
			//// Configure the AVR's SPI module (as slave) ready to send data to the host
			//// Enable slave SPI module with LSB sent first
			//SPCR |= (1 << SPE) | (1 << DORD);
			//
			//// Set the ADD8 bus pin to output mode
			//TMS6100_ADD8_DDR |= TMS6100_ADD8;
			//
			//// Set M0 to input
			//TMS6100_M0_DDR &= ~TMS6100_M0;
			//
			//// Put the first byte into the SPI buffer (the rest are sent by
			//// the SPI interrupt handler)
			//
			//// Read the byte stored at the current PHROM address
			//readByte = pgm_read_byte(&(phromData[tms6100.address]));			
//
			//// Pass the byte to the SPI buffer
			//SPDR = readByte;
			//
			//// Change the !Slave Select pin to low
			//TMS6100_SSC_PORT &= ~TMS6100_SSC;
			//
			//// Move the address pointer to the next byte
			//tms6100.address++;
			//
			//// Enable slave SPI interrupt for the actual transfers
			//SPCR |= (1 << SPIF);
			
			// Pulse debug0 two times
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0

		}
	}
	
	// M0 = 0 and M1 = 1 (rising edge) = LOAD ADDRESS command
	else if ((tms6100.m0PreviousState == 0 && tms6100.m1PreviousState == 0) && (tms6100.m0CurrentState == 0 && tms6100.m1CurrentState == 1))
	{		
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
					
			// Now we need to remove the chip select bits so we are left with only 14-bit address in this PHROM
			tms6100.address &= 0x3FFF; // 0b11 1111 1111 1111 = 0x3FFF
			
			// We now wait for a pulse of M0 to indicate the start of a 'real' read command
			
			// Pulse debug0 five times
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			
			// FOR TEST DATA WE SET THE ADDRESS TO ZERO ALWAYS
			tms6100.address = 0;
		}
		else
		{
			// We only have a partial address
			
			// If we previously had a valid address, disable the SPI until we get a new
			// full 20-bit address and turn on the M0 pin change interrupt
			if (tms6100.validAddressLoadedFlag == TRUE)
			{
				// Set the slave select control to 1 - !Slave Select disabled
				TMS6100_SSC_PORT |= TMS6100_SSC;
							
				// Ensure that the AVR's SPI module is disabled (so we can detect 'dummy' reads)
				// Disable slave SPI interrupt
				SPCR &= ~(1 << SPIF);
							
				// Disable slave SPI module
				SPCR &= ~(1 << SPE);
							
				// Set the ADD8 bus pin to input mode
				TMS6100_ADD8_DDR &= ~TMS6100_ADD8;
							
				// Set M0 to input
				TMS6100_M0_DDR &= ~TMS6100_M0;
							
				// Enable the M0 interrupt on pin change
				PCMSK0 |= (1 << PCINT1); // Enable PCINT1/PB1
			}
			
			// Mark the current address as invalid
			tms6100.validAddressLoadedFlag = FALSE;
			
			// Pulse debug0 four times
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
			TMS6100_DEBUG0_PORT |= TMS6100_DEBUG0;  // 1
			TMS6100_DEBUG0_PORT &= ~TMS6100_DEBUG0; // 0
		}
	}
	
	// M0 = 1 (rising edge) and M1 = 1 (rising edge) = INDIRECT ADDRESS command
	else if ((tms6100.m0PreviousState == 0 && tms6100.m1PreviousState == 0) && (tms6100.m0CurrentState == 1 && tms6100.m1CurrentState == 1))
	{
		// Not implemented
	}

	// Store the previous state of M0 and M1
	tms6100.m0PreviousState = tms6100.m0CurrentState;
	tms6100.m1PreviousState = tms6100.m1CurrentState;
}

// Main processing function
int main(void)
{
	// Power-up initialisation
	tms6100PowerUpReset();
	
    // Loop forever
    while (1) 
    {
		// Do nothing: All processing is interrupt driven
    }
}



	

	
