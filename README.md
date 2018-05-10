## Synopsis

TMS6100 speech PHROM (PHrase Read Only Memory) hardware emulator for use with the TMS5220 Voice Synthesis Processor in an Acorn BBC Microcomputer

## Motivation

This project creates a TMS6100 speech PHROM (PHrase Read Only Memory) for use with the TMS5220 Voice Synthesis Processor in an Acorn BBC Microcomputer. Whilst it’s still possible to purchase TMS5220 ICs from sites like Ebay, the TMS6100 was a mask-programmed ROM that Acorn produced specifically for their speech upgrade and is therefore difficult and expensive to source. Instead of using the original ROM, this project uses an ATmega32U2 microcontroller to both emulate the ROM and provide the original Acorn voice sample data recorded by BBC newsreader Kenneth Kendall.

## Installation

Note: This is an Atmel Studio 7 project that can be loaded and compiled by the IDE

Please see http://www.waitingforfriday.com/?p=30 for detailed documentation about TMS6100-Emulator

The PHROM data for both the Acorn Speech System PHROM and the American TI PHROM is included in the source-code.  By default it will compile using the Acorn Speech Data.  Specifying the compiler macro PHROM_ACORN selects the Acorn PHROM and PHROM_US selects the American PHROM.  Note that the American PHROM is not generally for use with the BBC Micro and is included for experimental use (or use in other microcomputer systems).

## Author

TMS6100-Emulator is written and maintained by Simon Inns.

## License (Software)

    TMS6100-Emulator is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    TMS6100-Emulator is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with TMS6100-Emulator. If not, see <http://www.gnu.org/licenses/>.

## License (Hardware)

Both the schematic and the PCB design of TMS6100-Emulator are covered by a Creative Commons license; as with the software you are welcome (and encouraged!) to extend, re-spin and otherwise use and modify the design as allowed by the license.  However; under the terms of the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) license you are required to release your design (or redesign) under the same license.  For details of the licensing requirements please see <https://creativecommons.org/licenses/by-sa/4.0/>
