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
LIBS:bbot_parts
LIBS:bbot-cache
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
Text GLabel 7400 4750 0    60   Input ~ 0
IN
Text GLabel 7400 4850 0    60   Input ~ 0
INH
Text GLabel 5950 4950 0    60   Input ~ 0
MOTOR_OUT
Text GLabel 7400 5050 0    60   Input ~ 0
SR
$Comp
L GND #PWR01
U 1 1 55DBECC5
P 4900 4150
F 0 "#PWR01" H 4900 3900 50  0001 C CNN
F 1 "GND" H 4900 4000 50  0000 C CNN
F 2 "" H 4900 4150 60  0000 C CNN
F 3 "" H 4900 4150 60  0000 C CNN
	1    4900 4150
	1    0    0    -1  
$EndComp
Text GLabel 4900 3450 1    60   Input ~ 0
SR
$Comp
L R R5
U 1 1 55DBED84
P 4900 3700
F 0 "R5" V 4980 3700 50  0000 C CNN
F 1 "0.51k" V 4900 3700 39  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4830 3700 30  0001 C CNN
F 3 "" H 4900 3700 30  0000 C CNN
	1    4900 3700
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 55DBEE61
P 4700 3700
F 0 "C7" H 4725 3800 50  0000 L CNN
F 1 "100nF" H 4450 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4738 3550 30  0001 C CNN
F 3 "" H 4700 3700 60  0000 C CNN
	1    4700 3700
	1    0    0    -1  
$EndComp
Text Notes 4300 850  0    60   ~ 0
Slew rate resistors
Text Notes 6000 4250 0    60   ~ 0
Half bridge
$Comp
L GND #PWR02
U 1 1 55DCB8DC
P 7100 4650
F 0 "#PWR02" H 7100 4400 50  0001 C CNN
F 1 "GND" H 7100 4500 50  0000 C CNN
F 2 "" H 7100 4650 60  0000 C CNN
F 3 "" H 7100 4650 60  0000 C CNN
	1    7100 4650
	0    1    1    0   
$EndComp
$Comp
L GND #PWR03
U 1 1 55DCBA75
P 3450 4700
F 0 "#PWR03" H 3450 4450 50  0001 C CNN
F 1 "GND" H 3450 4550 50  0000 C CNN
F 2 "" H 3450 4700 60  0000 C CNN
F 3 "" H 3450 4700 60  0000 C CNN
	1    3450 4700
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR04
U 1 1 55DCBEE9
P 7100 5250
F 0 "#PWR04" H 7100 5100 50  0001 C CNN
F 1 "VCC" H 7100 5400 50  0000 C CNN
F 2 "" H 7100 5250 60  0000 C CNN
F 3 "" H 7100 5250 60  0000 C CNN
	1    7100 5250
	0    -1   -1   0   
$EndComp
Text GLabel 3350 5000 0    60   Input ~ 0
IN
Text GLabel 3350 4900 0    60   Input ~ 0
INH
$Comp
L R R13
U 1 1 55DD44FF
P 3500 5000
F 0 "R13" V 3580 5000 50  0000 C CNN
F 1 "10k" V 3500 5000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3430 5000 30  0001 C CNN
F 3 "" H 3500 5000 30  0000 C CNN
	1    3500 5000
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 55DD4622
P 3800 4900
F 0 "R10" V 3880 4900 50  0000 C CNN
F 1 "10k" V 3800 4900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3730 4900 30  0001 C CNN
F 3 "" H 3800 4900 30  0000 C CNN
	1    3800 4900
	0    1    1    0   
$EndComp
$Comp
L C C4
U 1 1 55DE12C3
P 6650 4750
F 0 "C4" H 6675 4850 50  0000 L CNN
F 1 "220nF" H 6675 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6688 4600 30  0001 C CNN
F 3 "" H 6650 4750 60  0000 C CNN
	1    6650 4750
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 55DE1484
P 6200 4750
F 0 "C1" H 6225 4850 50  0000 L CNN
F 1 "220nF" H 6225 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6238 4600 30  0001 C CNN
F 3 "" H 6200 4750 60  0000 C CNN
	1    6200 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 55DE15CA
P 6200 4600
F 0 "#PWR05" H 6200 4350 50  0001 C CNN
F 1 "GND" H 6200 4450 50  0000 C CNN
F 2 "" H 6200 4600 60  0000 C CNN
F 3 "" H 6200 4600 60  0000 C CNN
	1    6200 4600
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR06
U 1 1 55DE162C
P 6650 4600
F 0 "#PWR06" H 6650 4450 50  0001 C CNN
F 1 "VCC" H 6650 4750 50  0000 C CNN
F 2 "" H 6650 4600 60  0000 C CNN
F 3 "" H 6650 4600 60  0000 C CNN
	1    6650 4600
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 55DE33DD
P 5700 5150
F 0 "R1" V 5780 5150 50  0000 C CNN
F 1 "1k" V 5700 5150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5630 5150 30  0001 C CNN
F 3 "" H 5700 5150 30  0000 C CNN
	1    5700 5150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR07
U 1 1 55DE35E8
P 5350 5150
F 0 "#PWR07" H 5350 4900 50  0001 C CNN
F 1 "GND" H 5350 5000 50  0000 C CNN
F 2 "" H 5350 5150 60  0000 C CNN
F 3 "" H 5350 5150 60  0000 C CNN
	1    5350 5150
	0    1    1    0   
$EndComp
Text Notes 3000 4200 0    60   ~ 0
Connectors
$Comp
L BTN8982TA U1
U 1 1 55DE2F00
P 7900 4950
F 0 "U1" H 7900 5150 60  0000 C CNN
F 1 "BTN8982TA" H 7850 4950 60  0000 C CNN
F 2 "bbot_footprints:TO263-7" H 7850 5000 60  0001 C CNN
F 3 "" H 7850 5000 60  0000 C CNN
	1    7900 4950
	0    1    1    0   
$EndComp
$Comp
L C C10
U 1 1 55DEF47B
P 7350 5450
F 0 "C10" H 7375 5550 50  0000 L CNN
F 1 "100nF" H 7450 5450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7388 5300 30  0001 C CNN
F 3 "" H 7350 5450 60  0000 C CNN
	1    7350 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 55DEFA8D
P 7350 5600
F 0 "#PWR08" H 7350 5350 50  0001 C CNN
F 1 "GND" H 7350 5450 50  0000 C CNN
F 2 "" H 7350 5600 60  0000 C CNN
F 3 "" H 7350 5600 60  0000 C CNN
	1    7350 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 3450 4900 3550
Wire Wire Line
	4900 3550 4700 3550
Wire Wire Line
	4700 3850 4900 3850
Wire Wire Line
	4900 3850 4900 4150
Wire Wire Line
	3650 4900 3350 4900
Wire Wire Line
	5950 4950 7400 4950
Wire Wire Line
	7100 4650 7400 4650
Wire Wire Line
	6200 4900 6200 4950
Connection ~ 6200 4950
Wire Wire Line
	6650 4900 6650 4950
Connection ~ 6650 4950
Wire Wire Line
	7400 5150 5850 5150
Wire Wire Line
	5350 5150 5550 5150
Wire Wire Line
	7100 5250 7400 5250
Wire Wire Line
	7350 5300 7350 5250
Connection ~ 7350 5250
$Comp
L C C2
U 1 1 55E2466B
P 5700 5400
F 0 "C2" H 5725 5500 50  0000 L CNN
F 1 "1nF" H 5725 5300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5738 5250 30  0001 C CNN
F 3 "" H 5700 5400 60  0000 C CNN
	1    5700 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	5550 5400 5400 5400
Wire Wire Line
	5400 5400 5400 5150
Connection ~ 5400 5150
Wire Wire Line
	5850 5400 5950 5400
Wire Wire Line
	5950 5400 5950 5150
Connection ~ 5950 5150
Text GLabel 6100 5350 3    60   Input ~ 0
IS
Wire Wire Line
	6100 5350 6100 5150
Connection ~ 6100 5150
$Comp
L CONN_01X04 P3
U 1 1 55E24B63
P 4200 4850
F 0 "P3" H 4200 5100 50  0000 C CNN
F 1 "CONN_01X04" V 4300 4850 50  0000 C CNN
F 2 "w_conn_df13:df13-4p-125h" H 4200 4850 60  0001 C CNN
F 3 "" H 4200 4850 60  0000 C CNN
	1    4200 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4900 4000 4900
Wire Wire Line
	3650 5000 4000 5000
Wire Wire Line
	3450 4700 4000 4700
Text GLabel 3850 4800 0    60   Input ~ 0
IS
Wire Wire Line
	3850 4800 4000 4800
$EndSCHEMATC
