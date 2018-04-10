/************************************************************************
	hardwaremap.h

    Physical hardware mapping definitions for portability
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

#ifndef HARDWAREMAP_H_
#define HARDWAREMAP_H_

// Target device: ATmega32u2
// Also tested on the ATmega32u4 (Arduino Leonardo mini pro)

// Definitions for TMS6100 IO Pins --------------------------------------

// ADD1 (PD2)
#define TMS6100_ADD1_PORT	PORTD
#define TMS6100_ADD1_PIN	PIND
#define TMS6100_ADD1_DDR	DDRD
#define TMS6100_ADD1		(1 << 2)

// ADD2 (PD3)
#define TMS6100_ADD2_PORT	PORTD
#define TMS6100_ADD2_PIN	PIND
#define TMS6100_ADD2_DDR	DDRD
#define TMS6100_ADD2		(1 << 3)

// ADD4 (PD4)
#define TMS6100_ADD4_PORT	PORTD
#define TMS6100_ADD4_PIN	PIND
#define TMS6100_ADD4_DDR	DDRD
#define TMS6100_ADD4		(1 << 4)

// ADD8 (PB3/MOSI)
#define TMS6100_ADD8_PORT	PORTB
#define TMS6100_ADD8_PIN	PINB
#define TMS6100_ADD8_DDR	DDRB
#define TMS6100_ADD8		(1 << 3)

// M0 (PD0/INT0)
#define TMS6100_M0_PORT		PORTD
#define TMS6100_M0_PIN		PIND
#define TMS6100_M0_DDR		DDRD
#define TMS6100_M0			(1 << 0)
#define TMS6100_M0_INT		INT0
#define TMS6100_M0_INT_VECT	INT0_vect
#define TMS6100_M0_ISC0		ISC00
#define TMS6100_M0_ISC1		ISC01
#define TMS6100_M0_INTF		INTF0

// M1 (PD1/INT1)
#define TMS6100_M1_PORT		PORTD
#define TMS6100_M1_PIN		PIND
#define TMS6100_M1_DDR		DDRD
#define TMS6100_M1			(1 << 1)
#define TMS6100_M1_INT		INT1
#define TMS6100_M1_INT_VECT	INT1_vect
#define TMS6100_M1_ISC0		ISC10
#define TMS6100_M1_ISC1		ISC11
#define TMS6100_M1_INTF		INTF1

// CLK (PB4) - Not currently used by the emulator as we use
// the SPI module to asynchronously time data based on the 
// M0 pulses (which has the effect of making the data in sync
// with the clock).
#define TMS6100_CLK_PORT	PORTB
#define TMS6100_CLK_PIN		PINB
#define TMS6100_CLK_DDR		DDRB
#define TMS6100_CLK			(1 << 4)

// Definitions for SPI pins ---------------------------------------------

// MISO - This is ADD8 defined above 

// MOSI (PB2)
#define TMS6100_MOSI_PORT	PORTB
#define TMS6100_MOSI_PIN	PINB
#define TMS6100_MOSI_DDR	DDRB
#define TMS6100_MOSI		(1 << 2)

// SCK (PB1)
#define TMS6100_SCK_PORT	PORTB
#define TMS6100_SCK_PIN		PINB
#define TMS6100_SCK_DDR		DDRB
#define TMS6100_SCK			(1 << 1)

// !SS (PB0)
#define TMS6100_SS_PORT		PORTB
#define TMS6100_SS_PIN		PINB
#define TMS6100_SS_DDR		DDRB
#define TMS6100_SS			(1 << 0)

#endif /* HARDWAREMAP_H_ */