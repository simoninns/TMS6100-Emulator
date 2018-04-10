EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA32U2-AU U?
U 1 1 5ACA7F52
P 7850 2800
F 0 "U?" H 7000 4150 50  0000 C CNN
F 1 "ATMEGA32U2-AU" H 8600 1450 50  0000 C CNN
F 2 "TQFP-32" H 7700 2850 50  0001 C CNN
F 3 "http://www.atmel.com/Images/doc7799.pdf" H 7950 1350 50  0001 C CNN
	1    7850 2800
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y?
U 1 1 5ACA8008
P 5500 2400
F 0 "Y?" H 5500 2550 50  0000 C CNN
F 1 "Crystal" H 5500 2250 50  0000 C CNN
F 2 "" H 5500 2400 50  0001 C CNN
F 3 "" H 5500 2400 50  0001 C CNN
	1    5500 2400
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5ACA8069
P 5200 1950
F 0 "C?" H 5225 2050 50  0000 L CNN
F 1 "C" H 5225 1850 50  0000 L CNN
F 2 "" H 5238 1800 50  0001 C CNN
F 3 "" H 5200 1950 50  0001 C CNN
	1    5200 1950
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5ACA809C
P 5200 2800
F 0 "C?" H 5225 2900 50  0000 L CNN
F 1 "C" H 5225 2700 50  0000 L CNN
F 2 "" H 5238 2650 50  0001 C CNN
F 3 "" H 5200 2800 50  0001 C CNN
	1    5200 2800
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5ACA80FA
P 6600 2100
F 0 "R?" V 6680 2100 50  0000 C CNN
F 1 "R" V 6600 2100 50  0000 C CNN
F 2 "" V 6530 2100 50  0001 C CNN
F 3 "" H 6600 2100 50  0001 C CNN
	1    6600 2100
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 5ACA81C4
P 6250 2200
F 0 "#PWR?" H 6250 2050 50  0001 C CNN
F 1 "VCC" H 6250 2350 50  0000 C CNN
F 2 "" H 6250 2200 50  0001 C CNN
F 3 "" H 6250 2200 50  0001 C CNN
	1    6250 2200
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 5ACA8225
P 6750 3100
F 0 "#PWR?" H 6750 2950 50  0001 C CNN
F 1 "VCC" H 6750 3250 50  0000 C CNN
F 2 "" H 6750 3100 50  0001 C CNN
F 3 "" H 6750 3100 50  0001 C CNN
	1    6750 3100
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 5ACA823F
P 7850 1400
F 0 "#PWR?" H 7850 1250 50  0001 C CNN
F 1 "VCC" H 7850 1550 50  0000 C CNN
F 2 "" H 7850 1400 50  0001 C CNN
F 3 "" H 7850 1400 50  0001 C CNN
	1    7850 1400
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5ACA827C
P 6450 2100
F 0 "#PWR?" H 6450 1950 50  0001 C CNN
F 1 "VCC" H 6450 2250 50  0000 C CNN
F 2 "" H 6450 2100 50  0001 C CNN
F 3 "" H 6450 2100 50  0001 C CNN
	1    6450 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6250 2200 6500 2200
Wire Wire Line
	6500 2200 6500 2250
Wire Wire Line
	6500 2250 6700 2250
Wire Wire Line
	6700 2250 6700 2200
Wire Wire Line
	6700 2200 6750 2200
Wire Wire Line
	6750 3400 6750 4200
Wire Wire Line
	6750 4200 7850 4200
Wire Wire Line
	6750 2300 5800 2300
Wire Wire Line
	5800 2300 5800 2250
Wire Wire Line
	5800 2250 5500 2250
Wire Wire Line
	5500 2550 5800 2550
Wire Wire Line
	5800 2550 5800 2400
Wire Wire Line
	5800 2400 6750 2400
Wire Wire Line
	5350 1950 5500 1950
Wire Wire Line
	5500 1950 5500 2250
Wire Wire Line
	5500 2550 5500 2800
Wire Wire Line
	5500 2800 5350 2800
Wire Wire Line
	5050 1100 5050 2800
$Comp
L GND #PWR?
U 1 1 5ACA8B89
P 4850 2400
F 0 "#PWR?" H 4850 2150 50  0001 C CNN
F 1 "GND" H 4850 2250 50  0000 C CNN
F 2 "" H 4850 2400 50  0001 C CNN
F 3 "" H 4850 2400 50  0001 C CNN
	1    4850 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2400 5050 2400
Connection ~ 5050 2400
Wire Wire Line
	5050 1100 8950 1100
Wire Wire Line
	8950 1100 8950 1700
Connection ~ 5050 1950
Text Label 8950 2600 0    60   ~ 0
MO/SCK
Text Label 8950 2700 0    60   ~ 0
M1
Text Label 8950 2800 0    60   ~ 0
ADD1
Text Label 8950 2900 0    60   ~ 0
ADD2
Text Label 8950 3000 0    60   ~ 0
ADD4
Text Label 8950 1800 0    60   ~ 0
M0/SCK
Text Label 8950 1900 0    60   ~ 0
MOSI
Text Label 8950 2000 0    60   ~ 0
ADD8/MISO
Text Label 8950 2100 0    60   ~ 0
CLK
Wire Wire Line
	6750 1850 6750 2100
Wire Wire Line
	6550 1850 6750 1850
Text Label 6550 1850 2    60   ~ 0
/RESET
$EndSCHEMATC
