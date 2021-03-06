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
LIBS:special
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
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:stm32
LIBS:bldc-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
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
L +12V #PWR01
U 1 1 5500CB9C
P 2400 1100
F 0 "#PWR01" H 2400 1050 20  0001 C CNN
F 1 "+12V" H 2500 1200 30  0000 C CNN
F 2 "" H 2400 1100 60  0000 C CNN
F 3 "" H 2400 1100 60  0000 C CNN
	1    2400 1100
	1    0    0    -1  
$EndComp
$Comp
L IRF7343PBF Q13
U 1 1 5500CC4F
P 6350 2150
F 0 "Q13" H 6350 2002 40  0000 R CNN
F 1 "IRF7343PBF" H 6900 2050 40  0000 R CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6171 2251 29  0001 C CNN
F 3 "" H 6350 2150 60  0000 C CNN
	1    6350 2150
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5500CDD4
P 5650 1250
F 0 "R1" V 5730 1250 40  0000 C CNN
F 1 "330" V 5657 1251 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5580 1250 30  0001 C CNN
F 3 "" H 5650 1250 30  0000 C CNN
	1    5650 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5500D99F
P 6450 2450
F 0 "#PWR02" H 6450 2450 30  0001 C CNN
F 1 "GND" H 6450 2380 30  0001 C CNN
F 2 "" H 6450 2450 60  0000 C CNN
F 3 "" H 6450 2450 60  0000 C CNN
	1    6450 2450
	1    0    0    -1  
$EndComp
Text Label 7300 1950 2    60   ~ 0
MOTORA
$Comp
L IRF7343PBF Q13
U 2 1 5500CCB8
P 6350 1750
F 0 "Q13" H 6350 1602 40  0000 R CNN
F 1 "IRF7343PBF" H 6900 1700 40  0000 R CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6171 1851 29  0001 C CNN
F 3 "" H 6350 1750 60  0000 C CNN
	2    6350 1750
	1    0    0    1   
$EndComp
$Comp
L GND #PWR03
U 1 1 55038F4B
P 5650 2450
F 0 "#PWR03" H 5650 2450 30  0001 C CNN
F 1 "GND" H 5650 2380 30  0001 C CNN
F 2 "" H 5650 2450 60  0000 C CNN
F 3 "" H 5650 2450 60  0000 C CNN
	1    5650 2450
	1    0    0    -1  
$EndComp
Text Label 7300 3800 2    60   ~ 0
MOTORB
Text Label 7300 5600 2    60   ~ 0
MOTORC
Wire Wire Line
	6450 1950 7300 1950
Connection ~ 6450 1950
Wire Wire Line
	6450 2350 6450 2450
Connection ~ 6550 1950
Wire Wire Line
	5650 1400 5650 1950
Wire Wire Line
	5650 1050 6800 1050
Wire Wire Line
	5100 2200 5350 2200
Wire Wire Line
	5650 2350 5650 2450
Wire Wire Line
	6450 950  6450 1550
Connection ~ 6450 1050
$Comp
L CONN_01X03 P2
U 1 1 5503CAB1
P 2050 2550
F 0 "P2" H 2050 2750 50  0000 C CNN
F 1 "CONN_01X03" V 2150 2550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 2050 2550 60  0001 C CNN
F 3 "" H 2050 2550 60  0000 C CNN
	1    2050 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2450 1750 2450
Wire Wire Line
	1750 2450 1750 2900
Wire Wire Line
	1750 2650 1850 2650
Connection ~ 1750 2650
$Comp
L GND #PWR04
U 1 1 5503CE08
P 1750 2900
F 0 "#PWR04" H 1750 2900 30  0001 C CNN
F 1 "GND" H 1750 2830 30  0001 C CNN
F 2 "" H 1750 2900 60  0000 C CNN
F 3 "" H 1750 2900 60  0000 C CNN
	1    1750 2900
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR05
U 1 1 5503CF99
P 1150 2550
F 0 "#PWR05" H 1150 2500 20  0001 C CNN
F 1 "+12V" V 1250 2600 30  0000 C CNN
F 2 "" H 1150 2550 60  0000 C CNN
F 3 "" H 1150 2550 60  0000 C CNN
	1    1150 2550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1150 2550 1850 2550
$Comp
L C C1
U 1 1 5503DDD7
P 6800 1200
F 0 "C1" H 6825 1300 50  0000 L CNN
F 1 "0.1" H 6825 1100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6838 1050 30  0001 C CNN
F 3 "" H 6800 1200 60  0000 C CNN
	1    6800 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5503E59D
P 6800 1350
F 0 "#PWR06" H 6800 1350 30  0001 C CNN
F 1 "GND" H 6800 1280 30  0001 C CNN
F 2 "" H 6800 1350 60  0000 C CNN
F 3 "" H 6800 1350 60  0000 C CNN
	1    6800 1350
	1    0    0    -1  
$EndComp
$Comp
L CP C4
U 1 1 55041B6B
P 3000 1250
F 0 "C4" H 3025 1350 50  0000 L CNN
F 1 "4.7" H 3025 1150 50  0000 L CNN
F 2 "Capacitors_Elko_ThroughHole:Elko_vert_11.2x6.3mm_RM2.5_CopperClear" H 3038 1100 30  0001 C CNN
F 3 "" H 3000 1250 60  0000 C CNN
	1    3000 1250
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P3
U 1 1 55045B34
P 9300 2450
F 0 "P3" H 9300 2650 50  0000 C CNN
F 1 "CONN_01X03" V 9400 2450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 9300 2450 60  0001 C CNN
F 3 "" H 9300 2450 60  0000 C CNN
	1    9300 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 2350 8650 2350
Wire Wire Line
	9100 2450 8650 2450
Wire Wire Line
	9100 2550 8650 2550
Text Label 8650 2350 0    60   ~ 0
MOTORA
Text Label 8650 2450 0    60   ~ 0
MOTORB
Text Label 8650 2550 0    60   ~ 0
MOTORC
$Comp
L GND #PWR08
U 1 1 55046EBD
P 6800 1350
F 0 "#PWR08" H 6800 1350 30  0001 C CNN
F 1 "GND" H 6800 1280 30  0001 C CNN
F 2 "" H 6800 1350 60  0000 C CNN
F 3 "" H 6800 1350 60  0000 C CNN
	1    6800 1350
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR09
U 1 1 5505325E
P 2050 4250
F 0 "#PWR09" H 2050 4200 20  0001 C CNN
F 1 "+12V" H 2150 4350 30  0000 C CNN
F 2 "" H 2050 4250 60  0000 C CNN
F 3 "" H 2050 4250 60  0000 C CNN
	1    2050 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 4250 2250 4250
Wire Wire Line
	5650 1050 5650 1100
Wire Wire Line
	5650 1700 6150 1700
$Comp
L MMBF170 Q8
U 1 1 550386CD
P 5550 2150
F 0 "Q8" H 5550 2001 40  0000 R CNN
F 1 "IRLML2502" H 5600 1900 40  0000 R CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23_Handsoldering" H 5420 2252 29  0001 C CNN
F 3 "" H 5550 2150 60  0000 C CNN
	1    5550 2150
	1    0    0    -1  
$EndComp
Connection ~ 5650 1700
Text Label 5100 2200 0    60   ~ 0
M1A
Wire Wire Line
	6150 2200 5900 2200
Text Label 5900 2200 0    60   ~ 0
~M1A
$Sheet
S 2250 4150 1450 1500
U 55196E3C
F0 "STM32" 60
F1 "stm32.sch" 60
F2 "M1A" O R 3700 4250 60 
F3 "M1B" O R 3700 4400 60 
F4 "M1C" O R 3700 4550 60 
F5 "~M1A" O R 3700 4700 60 
F6 "~M1B" O R 3700 4850 60 
F7 "~M1C" O R 3700 5000 60 
F8 "VMOT" O L 2250 4250 60 
$EndSheet
$Comp
L IRF7343PBF Q4
U 1 1 551B42C7
P 6350 4000
F 0 "Q4" H 6350 3852 40  0000 R CNN
F 1 "IRF7343PBF" H 6900 3900 40  0000 R CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6171 4101 29  0001 C CNN
F 3 "" H 6350 4000 60  0000 C CNN
	1    6350 4000
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 551B42CD
P 5650 3100
F 0 "R2" V 5730 3100 40  0000 C CNN
F 1 "330" V 5657 3101 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5580 3100 30  0001 C CNN
F 3 "" H 5650 3100 30  0000 C CNN
	1    5650 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 551B42D3
P 6450 4300
F 0 "#PWR010" H 6450 4300 30  0001 C CNN
F 1 "GND" H 6450 4230 30  0001 C CNN
F 2 "" H 6450 4300 60  0000 C CNN
F 3 "" H 6450 4300 60  0000 C CNN
	1    6450 4300
	1    0    0    -1  
$EndComp
$Comp
L IRF7343PBF Q4
U 2 1 551B42DA
P 6350 3600
F 0 "Q4" H 6350 3452 40  0000 R CNN
F 1 "IRF7343PBF" H 6900 3550 40  0000 R CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6171 3701 29  0001 C CNN
F 3 "" H 6350 3600 60  0000 C CNN
	2    6350 3600
	1    0    0    1   
$EndComp
$Comp
L GND #PWR011
U 1 1 551B42E0
P 5650 4300
F 0 "#PWR011" H 5650 4300 30  0001 C CNN
F 1 "GND" H 5650 4230 30  0001 C CNN
F 2 "" H 5650 4300 60  0000 C CNN
F 3 "" H 5650 4300 60  0000 C CNN
	1    5650 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3800 7300 3800
Connection ~ 6450 3800
Wire Wire Line
	6450 4200 6450 4300
Connection ~ 6550 3800
Wire Wire Line
	5650 3250 5650 3800
Wire Wire Line
	5650 2900 6800 2900
Wire Wire Line
	5100 4050 5350 4050
Wire Wire Line
	5650 4200 5650 4300
Wire Wire Line
	6450 2800 6450 3400
Connection ~ 6450 2900
$Comp
L C C3
U 1 1 551B42F0
P 6800 3050
F 0 "C3" H 6825 3150 50  0000 L CNN
F 1 "0.1" H 6825 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6838 2900 30  0001 C CNN
F 3 "" H 6800 3050 60  0000 C CNN
	1    6800 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 551B42F6
P 6800 3200
F 0 "#PWR012" H 6800 3200 30  0001 C CNN
F 1 "GND" H 6800 3130 30  0001 C CNN
F 2 "" H 6800 3200 60  0000 C CNN
F 3 "" H 6800 3200 60  0000 C CNN
	1    6800 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 551B42FC
P 6800 3200
F 0 "#PWR013" H 6800 3200 30  0001 C CNN
F 1 "GND" H 6800 3130 30  0001 C CNN
F 2 "" H 6800 3200 60  0000 C CNN
F 3 "" H 6800 3200 60  0000 C CNN
	1    6800 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 2900 5650 2950
Wire Wire Line
	5650 3550 6150 3550
$Comp
L MMBF170 Q2
U 1 1 551B4304
P 5550 4000
F 0 "Q2" H 5550 3851 40  0000 R CNN
F 1 "IRLML2502" H 5600 3750 40  0000 R CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23_Handsoldering" H 5420 4102 29  0001 C CNN
F 3 "" H 5550 4000 60  0000 C CNN
	1    5550 4000
	1    0    0    -1  
$EndComp
Connection ~ 5650 3550
Text Label 5100 4050 0    60   ~ 0
M1B
Wire Wire Line
	6150 4050 5900 4050
Text Label 5900 4050 0    60   ~ 0
~M1B
$Comp
L IRF7343PBF Q5
U 1 1 551B4A0C
P 6350 5800
F 0 "Q5" H 6350 5652 40  0000 R CNN
F 1 "IRF7343PBF" H 6900 5700 40  0000 R CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6171 5901 29  0001 C CNN
F 3 "" H 6350 5800 60  0000 C CNN
	1    6350 5800
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 551B4A12
P 5650 4900
F 0 "R3" V 5730 4900 40  0000 C CNN
F 1 "330" V 5657 4901 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5580 4900 30  0001 C CNN
F 3 "" H 5650 4900 30  0000 C CNN
	1    5650 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 551B4A18
P 6450 6100
F 0 "#PWR014" H 6450 6100 30  0001 C CNN
F 1 "GND" H 6450 6030 30  0001 C CNN
F 2 "" H 6450 6100 60  0000 C CNN
F 3 "" H 6450 6100 60  0000 C CNN
	1    6450 6100
	1    0    0    -1  
$EndComp
$Comp
L IRF7343PBF Q5
U 2 1 551B4A1E
P 6350 5400
F 0 "Q5" H 6350 5252 40  0000 R CNN
F 1 "IRF7343PBF" H 6900 5350 40  0000 R CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6171 5501 29  0001 C CNN
F 3 "" H 6350 5400 60  0000 C CNN
	2    6350 5400
	1    0    0    1   
$EndComp
$Comp
L GND #PWR015
U 1 1 551B4A24
P 5650 6100
F 0 "#PWR015" H 5650 6100 30  0001 C CNN
F 1 "GND" H 5650 6030 30  0001 C CNN
F 2 "" H 5650 6100 60  0000 C CNN
F 3 "" H 5650 6100 60  0000 C CNN
	1    5650 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 5600 7300 5600
Connection ~ 6450 5600
Wire Wire Line
	6450 6000 6450 6100
Connection ~ 6550 5600
Wire Wire Line
	5650 5050 5650 5600
Wire Wire Line
	5650 4700 6800 4700
Wire Wire Line
	5100 5850 5350 5850
Wire Wire Line
	5650 6000 5650 6100
Wire Wire Line
	6450 4600 6450 5200
Connection ~ 6450 4700
$Comp
L C C5
U 1 1 551B4A38
P 6800 4850
F 0 "C5" H 6825 4950 50  0000 L CNN
F 1 "0.1" H 6825 4750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 6838 4700 30  0001 C CNN
F 3 "" H 6800 4850 60  0000 C CNN
	1    6800 4850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 551B4A3E
P 6800 5000
F 0 "#PWR016" H 6800 5000 30  0001 C CNN
F 1 "GND" H 6800 4930 30  0001 C CNN
F 2 "" H 6800 5000 60  0000 C CNN
F 3 "" H 6800 5000 60  0000 C CNN
	1    6800 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 551B4A44
P 6800 5000
F 0 "#PWR017" H 6800 5000 30  0001 C CNN
F 1 "GND" H 6800 4930 30  0001 C CNN
F 2 "" H 6800 5000 60  0000 C CNN
F 3 "" H 6800 5000 60  0000 C CNN
	1    6800 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 4700 5650 4750
Wire Wire Line
	5650 5350 6150 5350
$Comp
L MMBF170 Q3
U 1 1 551B4A4C
P 5550 5800
F 0 "Q3" H 5550 5651 40  0000 R CNN
F 1 "IRLML2502" H 5600 5550 40  0000 R CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23_Handsoldering" H 5420 5902 29  0001 C CNN
F 3 "" H 5550 5800 60  0000 C CNN
	1    5550 5800
	1    0    0    -1  
$EndComp
Connection ~ 5650 5350
Text Label 5100 5850 0    60   ~ 0
M1C
Wire Wire Line
	6150 5850 5900 5850
Text Label 5900 5850 0    60   ~ 0
~M1C
$Comp
L IRF7606PBF Q1
U 1 1 551B7560
P 3800 1300
F 0 "Q1" H 3600 1600 50  0000 L CNN
F 1 "FDS6675" H 3600 1050 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 3800 1500 50  0001 C CIN
F 3 "" V 3800 1300 50  0000 L CNN
	1    3800 1300
	-1   0    0    -1  
$EndComp
Text Label 6450 950  0    60   ~ 0
VMOT
Text Label 6450 2800 0    60   ~ 0
VMOT
Text Label 6450 4600 0    60   ~ 0
VMOT
Wire Wire Line
	4100 1100 4100 1300
Connection ~ 4100 1200
Wire Wire Line
	4100 1100 5000 1100
Text Label 5000 1100 2    60   ~ 0
VMOT
Wire Wire Line
	3500 1100 3500 1400
Connection ~ 3500 1300
Connection ~ 3500 1200
Wire Wire Line
	2600 1100 3500 1100
$Comp
L GND #PWR018
U 1 1 551BAFAF
P 4100 1550
F 0 "#PWR018" H 4100 1550 30  0001 C CNN
F 1 "GND" H 4100 1480 30  0001 C CNN
F 2 "" H 4100 1550 60  0000 C CNN
F 3 "" H 4100 1550 60  0000 C CNN
	1    4100 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1450 4100 1550
$Comp
L CP C2
U 1 1 551BB97E
P 4600 1350
F 0 "C2" H 4625 1450 50  0000 L CNN
F 1 "10.0" H 4625 1250 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:TantalC_SizeC_EIA-6032_Reflow" H 4638 1200 30  0001 C CNN
F 3 "" H 4600 1350 60  0000 C CNN
	1    4600 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1200 4600 1100
Connection ~ 4600 1100
$Comp
L GND #PWR019
U 1 1 551BBB46
P 4600 1550
F 0 "#PWR019" H 4600 1550 30  0001 C CNN
F 1 "GND" H 4600 1480 30  0001 C CNN
F 2 "" H 4600 1550 60  0000 C CNN
F 3 "" H 4600 1550 60  0000 C CNN
	1    4600 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1500 4600 1550
Wire Wire Line
	3700 4250 4100 4250
Wire Wire Line
	3700 4400 4100 4400
Wire Wire Line
	3700 4550 4100 4550
Wire Wire Line
	3700 4700 4100 4700
Wire Wire Line
	3700 4850 4100 4850
Wire Wire Line
	3700 5000 4100 5000
Text Label 4100 4250 2    60   ~ 0
M1A
Text Label 4100 4700 2    60   ~ 0
~M1A
Text Label 4100 4400 2    60   ~ 0
M1B
Text Label 4100 4850 2    60   ~ 0
~M1B
Text Label 4100 4550 2    60   ~ 0
M1C
Text Label 4100 5000 2    60   ~ 0
~M1C
Connection ~ 3500 1100
$Comp
L Jumper_NO_Small JP4
U 1 1 5527116D
P 2700 1100
F 0 "JP4" H 2700 1180 50  0000 C CNN
F 1 "Jumper_NO_Small" H 2710 1040 50  0001 C CNN
F 2 "Connect:PINHEAD1-2" H 2700 1100 60  0001 C CNN
F 3 "" H 2700 1100 60  0000 C CNN
	1    2700 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 1100 2400 1100
$Comp
L GND #PWR020
U 1 1 5529D8A8
P 3000 1550
F 0 "#PWR020" H 3000 1550 30  0001 C CNN
F 1 "GND" H 3000 1480 30  0001 C CNN
F 2 "" H 3000 1550 60  0000 C CNN
F 3 "" H 3000 1550 60  0000 C CNN
	1    3000 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1400 3000 1550
Connection ~ 3000 1100
$EndSCHEMATC
