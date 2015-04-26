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
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32
LIBS:bldc-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
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
L +3.3VP #PWR?
U 1 1 5518746C
P 5750 1150
F 0 "#PWR?" H 5800 1180 20  0001 C CNN
F 1 "+3.3VP" H 5750 1240 30  0000 C CNN
F 2 "" H 5750 1150 60  0000 C CNN
F 3 "" H 5750 1150 60  0000 C CNN
	1    5750 1150
	1    0    0    -1  
$EndComp
$Comp
L +3.3VP #PWR?
U 1 1 5518748D
P 6500 1150
F 0 "#PWR?" H 6550 1180 20  0001 C CNN
F 1 "+3.3VP" H 6500 1240 30  0000 C CNN
F 2 "" H 6500 1150 60  0000 C CNN
F 3 "" H 6500 1150 60  0000 C CNN
	1    6500 1150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 55187935
P 4900 1350
F 0 "C?" H 4925 1450 50  0000 L CNN
F 1 "0.1" H 4925 1250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4938 1200 30  0001 C CNN
F 3 "" H 4900 1350 60  0000 C CNN
	1    4900 1350
	1    0    0    -1  
$EndComp
$Comp
L Crystal_Small Y?
U 1 1 55187EE7
P 3150 2400
F 0 "Y?" H 3150 2500 50  0000 C CNN
F 1 "8MHz" H 3150 2300 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-U_Vertical" H 3150 2400 60  0001 C CNN
F 3 "" H 3150 2400 60  0000 C CNN
	1    3150 2400
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 55187EF0
P 2850 2700
F 0 "C?" H 2875 2800 50  0000 L CNN
F 1 "20" H 2875 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2888 2550 30  0001 C CNN
F 3 "" H 2850 2700 60  0000 C CNN
	1    2850 2700
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 55187EF7
P 3150 2700
F 0 "C?" H 3175 2800 50  0000 L CNN
F 1 "20" H 3175 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3188 2550 30  0001 C CNN
F 3 "" H 3150 2700 60  0000 C CNN
	1    3150 2700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 55187F02
P 1050 1700
F 0 "R?" V 1130 1700 50  0000 C CNN
F 1 "10K" V 1050 1700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 980 1700 30  0001 C CNN
F 3 "" H 1050 1700 30  0000 C CNN
	1    1050 1700
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 55187F10
P 1050 2250
F 0 "SW?" H 1200 2360 50  0000 C CNN
F 1 "SW_PUSH" H 1050 2170 50  0001 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_PTS645" H 1050 2250 60  0001 C CNN
F 3 "" H 1050 2250 60  0000 C CNN
	1    1050 2250
	0    -1   -1   0   
$EndComp
Text Label 600  1950 0    60   ~ 0
~RESET
$Comp
L GNDD #PWR?
U 1 1 55187F20
P 1050 2550
F 0 "#PWR?" H 1050 2300 60  0001 C CNN
F 1 "GNDD" H 1050 2400 60  0000 C CNN
F 2 "" H 1050 2550 60  0000 C CNN
F 3 "" H 1050 2550 60  0000 C CNN
	1    1050 2550
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 551883A4
P 1600 1500
F 0 "SW?" H 1750 1610 50  0000 C CNN
F 1 "SW_PUSH" H 1600 1420 50  0001 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_PTS645" H 1600 1500 60  0001 C CNN
F 3 "" H 1600 1500 60  0000 C CNN
	1    1600 1500
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 55188427
P 1600 2350
F 0 "R?" V 1680 2350 50  0000 C CNN
F 1 "10K" V 1600 2350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1530 2350 30  0001 C CNN
F 3 "" H 1600 2350 30  0000 C CNN
	1    1600 2350
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 55188502
P 1600 2550
F 0 "#PWR?" H 1600 2300 60  0001 C CNN
F 1 "GNDD" H 1600 2400 60  0000 C CNN
F 2 "" H 1600 2550 60  0000 C CNN
F 3 "" H 1600 2550 60  0000 C CNN
	1    1600 2550
	1    0    0    -1  
$EndComp
$Comp
L +3.3VP #PWR?
U 1 1 551885FE
P 1050 1150
F 0 "#PWR?" H 1100 1180 20  0001 C CNN
F 1 "+3.3VP" H 1050 1240 30  0000 C CNN
F 2 "" H 1050 1150 60  0000 C CNN
F 3 "" H 1050 1150 60  0000 C CNN
	1    1050 1150
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 55188B41
P 2850 2850
F 0 "#PWR?" H 2850 2600 60  0001 C CNN
F 1 "GNDD" H 2850 2700 60  0000 C CNN
F 2 "" H 2850 2850 60  0000 C CNN
F 3 "" H 2850 2850 60  0000 C CNN
	1    2850 2850
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 55188B5E
P 3150 2850
F 0 "#PWR?" H 3150 2600 60  0001 C CNN
F 1 "GNDD" H 3150 2700 60  0000 C CNN
F 2 "" H 3150 2850 60  0000 C CNN
F 3 "" H 3150 2850 60  0000 C CNN
	1    3150 2850
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 55188EDC
P 8800 4100
F 0 "R?" V 8880 4100 50  0000 C CNN
F 1 "10K" V 8800 4100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8730 4100 30  0001 C CNN
F 3 "" H 8800 4100 30  0000 C CNN
	1    8800 4100
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 55188F74
P 8800 4250
F 0 "#PWR?" H 8800 4000 60  0001 C CNN
F 1 "GNDD" H 8800 4100 60  0000 C CNN
F 2 "" H 8800 4250 60  0000 C CNN
F 3 "" H 8800 4250 60  0000 C CNN
	1    8800 4250
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 55189368
P 5150 1350
F 0 "C?" H 5175 1450 50  0000 L CNN
F 1 "0.1" H 5175 1250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5188 1200 30  0001 C CNN
F 3 "" H 5150 1350 60  0000 C CNN
	1    5150 1350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 55189396
P 5400 1350
F 0 "C?" H 5425 1450 50  0000 L CNN
F 1 "0.1" H 5425 1250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5438 1200 30  0001 C CNN
F 3 "" H 5400 1350 60  0000 C CNN
	1    5400 1350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 55189516
P 6750 1350
F 0 "C?" H 6775 1450 50  0000 L CNN
F 1 "0.1" H 6775 1250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6788 1200 30  0001 C CNN
F 3 "" H 6750 1350 60  0000 C CNN
	1    6750 1350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 551896F6
P 6750 1500
F 0 "#PWR?" H 6750 1250 60  0001 C CNN
F 1 "GNDA" H 6750 1350 60  0000 C CNN
F 2 "" H 6750 1500 60  0000 C CNN
F 3 "" H 6750 1500 60  0000 C CNN
	1    6750 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 55189971
P 5400 1500
F 0 "#PWR?" H 5400 1250 60  0001 C CNN
F 1 "GND" H 5400 1350 60  0000 C CNN
F 2 "" H 5400 1500 60  0000 C CNN
F 3 "" H 5400 1500 60  0000 C CNN
	1    5400 1500
	1    0    0    -1  
$EndComp
Text Label 900  4100 0    60   ~ 0
RXD
Text Label 900  3900 0    60   ~ 0
TXD
$Comp
L STM32F103RB U?
U 1 1 5518A6DB
P 6200 3650
F 0 "U?" H 4850 5600 50  0000 C CNN
F 1 "STM32F103RB" H 7350 1700 50  0000 C CNN
F 2 "LQFP64" H 6200 3650 50  0000 C CNN
F 3 "" H 6200 3950 60  0000 C CNN
	1    6200 3650
	1    0    0    -1  
$EndComp
Text Label 8350 2850 2    60   ~ 0
M1A
Text Label 8350 2950 2    60   ~ 0
M1B
Text Label 8350 3050 2    60   ~ 0
M1C
Text Label 8350 5050 2    60   ~ 0
~M1A
Text Label 8350 5150 2    60   ~ 0
~M1B
Text Label 8350 5250 2    60   ~ 0
~M1C
$Comp
L R R?
U 1 1 55189BE3
P 1300 3900
F 0 "R?" V 1380 3900 50  0000 C CNN
F 1 "330" V 1300 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1230 3900 30  0001 C CNN
F 3 "" H 1300 3900 30  0000 C CNN
	1    1300 3900
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 55189DAE
P 1300 4100
F 0 "R?" V 1380 4100 50  0000 C CNN
F 1 "330" V 1300 4100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1230 4100 30  0001 C CNN
F 3 "" H 1300 4100 30  0000 C CNN
	1    1300 4100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5518A993
P 5750 5750
F 0 "#PWR?" H 5750 5500 60  0001 C CNN
F 1 "GND" H 5750 5600 60  0000 C CNN
F 2 "" H 5750 5750 60  0000 C CNN
F 3 "" H 5750 5750 60  0000 C CNN
	1    5750 5750
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5518A9BC
P 6500 5750
F 0 "#PWR?" H 6500 5500 60  0001 C CNN
F 1 "GNDA" H 6500 5600 60  0000 C CNN
F 2 "" H 6500 5750 60  0000 C CNN
F 3 "" H 6500 5750 60  0000 C CNN
	1    6500 5750
	1    0    0    -1  
$EndComp
Text Label 8350 3350 2    60   ~ 0
SWDIO
Text Label 8350 3450 2    60   ~ 0
SWDCLK
$Comp
L CONN_01X06 P?
U 1 1 5518B7FA
P 2000 5200
F 0 "P?" H 2000 5550 50  0000 C CNN
F 1 "CONN_01X06" V 2100 5200 50  0000 C CNN
F 2 "" H 2000 5200 60  0000 C CNN
F 3 "" H 2000 5200 60  0000 C CNN
	1    2000 5200
	1    0    0    -1  
$EndComp
$Comp
L +3.3VP #PWR?
U 1 1 5518BB0F
P 1800 4800
F 0 "#PWR?" H 1850 4830 20  0001 C CNN
F 1 "+3.3VP" H 1800 4890 30  0000 C CNN
F 2 "" H 1800 4800 60  0000 C CNN
F 3 "" H 1800 4800 60  0000 C CNN
	1    1800 4800
	1    0    0    -1  
$EndComp
Text Label 1050 5050 0    60   ~ 0
SWDCLK
Text Label 1050 5250 0    60   ~ 0
SWDCLK
Text Label 1050 5350 0    60   ~ 0
~RESET
NoConn ~ 1800 5450
$Comp
L CONN_01X03 P?
U 1 1 5518CE4A
P 1950 4000
F 0 "P?" H 1950 4200 50  0000 C CNN
F 1 "CONN_01X03" V 2050 4000 50  0000 C CNN
F 2 "" H 1950 4000 60  0000 C CNN
F 3 "" H 1950 4000 60  0000 C CNN
	1    1950 4000
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 5518CF84
P 1650 5450
F 0 "#PWR?" H 1650 5200 60  0001 C CNN
F 1 "GNDD" H 1650 5300 60  0000 C CNN
F 2 "" H 1650 5450 60  0000 C CNN
F 3 "" H 1650 5450 60  0000 C CNN
	1    1650 5450
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 5518D088
P 1750 4250
F 0 "#PWR?" H 1750 4000 60  0001 C CNN
F 1 "GNDD" H 1750 4100 60  0000 C CNN
F 2 "" H 1750 4250 60  0000 C CNN
F 3 "" H 1750 4250 60  0000 C CNN
	1    1750 4250
	1    0    0    -1  
$EndComp
Text Label 8350 2550 2    60   ~ 0
SPI1_SCK
Text Label 8350 2650 2    60   ~ 0
SPI1_MISO
Text Label 8350 2750 2    60   ~ 0
SPI1_MOSI
Text Label 8350 4750 2    60   ~ 0
SCL
Text Label 8350 4850 2    60   ~ 0
SDA
Text Label 8350 4950 2    60   ~ 0
LED1
Text Label 8350 4350 2    60   ~ 0
TIM4_1
Text Label 8350 4450 2    60   ~ 0
TIM4_2
Text Label 8350 4550 2    60   ~ 0
TIM4_3
Text Label 8350 4650 2    60   ~ 0
TIM4_4
Text Label 8350 3150 2    60   ~ 0
TIM1_4
Text Label 8300 2050 2    60   ~ 0
TIM2_1
Text Label 8300 2150 2    60   ~ 0
TIM2_2
Text Label 8300 2250 2    60   ~ 0
TIM2_3
Text Label 8300 2350 2    60   ~ 0
TIM2_4
Text Label 8350 3750 2    60   ~ 0
TIM3_3
Text Label 8350 3850 2    60   ~ 0
TIM3_4
Text Label 4000 3750 0    60   ~ 0
ANAL1
Text Label 4000 3850 0    60   ~ 0
ANAL2
Text Label 4000 3950 0    60   ~ 0
ANAL3
$Comp
L Jumper_NO_Small JP?
U 1 1 5518F77D
P 6350 5700
F 0 "JP?" H 6350 5780 50  0000 C CNN
F 1 "Jumper_NO_Small" H 6360 5640 50  0001 C CNN
F 2 "" H 6350 5700 60  0000 C CNN
F 3 "" H 6350 5700 60  0000 C CNN
	1    6350 5700
	1    0    0    -1  
$EndComp
Text HLabel 10400 2500 2    60   Output ~ 0
M1A
Text HLabel 10400 2600 2    60   Output ~ 0
M1B
Text HLabel 10400 2700 2    60   Output ~ 0
M1C
Text HLabel 10400 2800 2    60   Output ~ 0
~M1A
Text HLabel 10400 2900 2    60   Output ~ 0
~M1B
Text HLabel 10400 3000 2    60   Output ~ 0
~M1C
Text Label 10050 2500 0    60   ~ 0
M1A
Text Label 10050 2600 0    60   ~ 0
M1B
Text Label 10050 2700 0    60   ~ 0
M1C
Text Label 10050 2800 0    60   ~ 0
~M1A
Text Label 10050 2900 0    60   ~ 0
~M1B
Text Label 10050 3000 0    60   ~ 0
~M1C
Text HLabel 1050 6200 0    60   Output ~ 0
VMOT
$Comp
L NCP1117ST18T3G U?
U 1 1 55192849
P 1700 6250
F 0 "U?" H 1700 6500 40  0000 C CNN
F 1 "NCP1117ST18T3G" H 1700 6450 40  0000 C CNN
F 2 "" H 1700 6250 60  0000 C CNN
F 3 "" H 1700 6250 60  0000 C CNN
	1    1700 6250
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 551928E4
P 1700 6550
F 0 "#PWR?" H 1700 6300 60  0001 C CNN
F 1 "GNDD" H 1700 6400 60  0000 C CNN
F 2 "" H 1700 6550 60  0000 C CNN
F 3 "" H 1700 6550 60  0000 C CNN
	1    1700 6550
	1    0    0    -1  
$EndComp
$Comp
L +3.3VP #PWR?
U 1 1 55192CE2
P 2650 6200
F 0 "#PWR?" H 2700 6230 20  0001 C CNN
F 1 "+3.3VP" H 2650 6290 30  0000 C CNN
F 2 "" H 2650 6200 60  0000 C CNN
F 3 "" H 2650 6200 60  0000 C CNN
	1    2650 6200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 55192F01
P 2200 6400
F 0 "C?" H 2225 6500 50  0000 L CNN
F 1 "0.1" H 2225 6300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2238 6250 30  0001 C CNN
F 3 "" H 2200 6400 60  0000 C CNN
	1    2200 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1600 5900 1600
Wire Wire Line
	5900 1600 6050 1600
Wire Wire Line
	6050 1600 6200 1600
Connection ~ 5900 1600
Wire Wire Line
	6500 1150 6500 1200
Wire Wire Line
	6500 1200 6500 1600
Wire Wire Line
	5750 1150 5750 1200
Wire Wire Line
	5750 1200 5750 1600
Connection ~ 5750 1600
Wire Wire Line
	2850 2300 3150 2300
Wire Wire Line
	3150 2300 4650 2300
Wire Wire Line
	3150 2500 4650 2500
Wire Wire Line
	3150 2500 3150 2550
Wire Wire Line
	2850 2300 2850 2550
Connection ~ 3150 2300
Wire Wire Line
	600  1950 1050 1950
Wire Wire Line
	1050 1950 4650 1950
Wire Wire Line
	1050 1950 1050 1850
Connection ~ 3150 2500
Connection ~ 1050 1950
Wire Wire Line
	1600 2100 4650 2100
Wire Wire Line
	1600 1800 1600 2100
Wire Wire Line
	1600 2100 1600 2200
Connection ~ 1600 2100
Wire Wire Line
	1600 2500 1600 2550
Wire Wire Line
	1050 1150 1050 1200
Wire Wire Line
	1050 1200 1050 1550
Wire Wire Line
	1600 1200 1050 1200
Connection ~ 1050 1200
Wire Wire Line
	4650 2500 4650 2450
Wire Wire Line
	5750 1200 4900 1200
Connection ~ 5750 1200
Wire Wire Line
	6750 1200 6500 1200
Connection ~ 6500 1200
Wire Wire Line
	4900 1500 5150 1500
Wire Wire Line
	5150 1500 5400 1500
Connection ~ 5150 1500
Connection ~ 5400 1500
Wire Wire Line
	7750 2950 8350 2950
Wire Wire Line
	7750 3050 8350 3050
Connection ~ 6050 1600
Wire Wire Line
	7750 3950 8800 3950
Wire Wire Line
	7750 5050 8350 5050
Wire Wire Line
	7750 5150 8350 5150
Wire Wire Line
	7750 5250 8350 5250
Wire Wire Line
	7750 2850 8350 2850
Wire Wire Line
	1150 4100 900  4100
Wire Wire Line
	1150 3900 900  3900
Wire Wire Line
	1450 3900 1750 3900
Wire Wire Line
	1450 4100 1600 4100
Wire Wire Line
	6500 5700 6500 5750
Wire Wire Line
	5750 5700 5900 5700
Wire Wire Line
	5900 5700 6050 5700
Wire Wire Line
	6050 5700 6200 5700
Wire Wire Line
	6200 5700 6250 5700
Connection ~ 6050 5700
Connection ~ 5900 5700
Wire Wire Line
	5750 5700 5750 5750
Connection ~ 5750 5700
Wire Wire Line
	7750 3350 8350 3350
Wire Wire Line
	7750 3450 8350 3450
Wire Wire Line
	1800 4800 1800 4950
Wire Wire Line
	1800 5050 1050 5050
Wire Wire Line
	1800 5150 1650 5150
Wire Wire Line
	1650 5150 1650 5450
Wire Wire Line
	1800 5250 1050 5250
Wire Wire Line
	1800 5350 1050 5350
Wire Wire Line
	1750 4000 1600 4000
Wire Wire Line
	1600 4000 1600 4100
Wire Wire Line
	1750 4100 1750 4250
Wire Wire Line
	7750 2550 8350 2550
Wire Wire Line
	7750 2650 8350 2650
Wire Wire Line
	7750 2750 8350 2750
Wire Wire Line
	7750 4350 8350 4350
Wire Wire Line
	7750 4450 8350 4450
Wire Wire Line
	7750 4550 8350 4550
Wire Wire Line
	7750 4650 8350 4650
Wire Wire Line
	7750 4750 8350 4750
Wire Wire Line
	7750 4850 8350 4850
Wire Wire Line
	7750 4950 8350 4950
Wire Wire Line
	7750 2150 8300 2150
Wire Wire Line
	7750 2250 8300 2250
Wire Wire Line
	7750 2350 8300 2350
Wire Wire Line
	7750 3150 8350 3150
Wire Wire Line
	7750 2050 8300 2050
Wire Wire Line
	7750 3750 8350 3750
Wire Wire Line
	7750 3850 8350 3850
Wire Wire Line
	4650 3750 4000 3750
Wire Wire Line
	4650 3850 4000 3850
Wire Wire Line
	4650 3950 4000 3950
Connection ~ 6200 5700
Wire Wire Line
	6450 5700 6500 5700
Connection ~ 6500 5700
Wire Wire Line
	10400 2500 10050 2500
Wire Wire Line
	10400 2600 10050 2600
Wire Wire Line
	10400 2700 10050 2700
Wire Wire Line
	10400 2800 10050 2800
Wire Wire Line
	10400 2900 10050 2900
Wire Wire Line
	10400 3000 10050 3000
Wire Wire Line
	1050 6200 1200 6200
Wire Wire Line
	1200 6200 1300 6200
Wire Wire Line
	1700 6500 1700 6550
Wire Wire Line
	2100 6200 2200 6200
Wire Wire Line
	2200 6200 2650 6200
Wire Wire Line
	2200 6250 2200 6200
Connection ~ 2200 6200
$Comp
L GNDD #PWR?
U 1 1 551931D7
P 2200 6550
F 0 "#PWR?" H 2200 6300 60  0001 C CNN
F 1 "GNDD" H 2200 6400 60  0000 C CNN
F 2 "" H 2200 6550 60  0000 C CNN
F 3 "" H 2200 6550 60  0000 C CNN
	1    2200 6550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5519322B
P 1200 6400
F 0 "C?" H 1225 6500 50  0000 L CNN
F 1 "0.1" H 1225 6300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1238 6250 30  0001 C CNN
F 3 "" H 1200 6400 60  0000 C CNN
	1    1200 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 6200 1200 6250
Connection ~ 1200 6200
$Comp
L GNDD #PWR?
U 1 1 55193323
P 1200 6550
F 0 "#PWR?" H 1200 6300 60  0001 C CNN
F 1 "GNDD" H 1200 6400 60  0000 C CNN
F 2 "" H 1200 6550 60  0000 C CNN
F 3 "" H 1200 6550 60  0000 C CNN
	1    1200 6550
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 55193384
P 2650 6400
F 0 "C?" H 2675 6500 50  0000 L CNN
F 1 "CP" H 2675 6300 50  0000 L CNN
F 2 "" H 2688 6250 30  0000 C CNN
F 3 "" H 2650 6400 60  0000 C CNN
	1    2650 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 6200 2650 6250
Connection ~ 2650 6200
$Comp
L GNDD #PWR?
U 1 1 551934C4
P 2650 6550
F 0 "#PWR?" H 2650 6300 60  0001 C CNN
F 1 "GNDD" H 2650 6400 60  0000 C CNN
F 2 "" H 2650 6550 60  0000 C CNN
F 3 "" H 2650 6550 60  0000 C CNN
	1    2650 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 4100 10250 4100
Wire Wire Line
	9700 4200 10250 4200
Wire Wire Line
	9700 4300 10250 4300
Wire Wire Line
	9700 4400 10250 4400
Wire Wire Line
	9550 4500 10250 4500
Wire Wire Line
	9700 4600 10250 4600
Wire Wire Line
	9700 4000 10250 4000
Wire Wire Line
	9700 3900 10250 3900
Text Label 9700 3900 0    60   ~ 0
SPI1_MISO
Text Label 9700 4000 0    60   ~ 0
SPI1_SCK
Text Label 9700 4100 0    60   ~ 0
SPI1_MOSI
Text Label 9700 4200 0    60   ~ 0
~ADNS_CS
Text Label 9700 4300 0    60   ~ 0
ADNS_MOT
$Comp
L +3.3VP #PWR?
U 1 1 5519486B
P 9700 4600
F 0 "#PWR?" H 9750 4630 20  0001 C CNN
F 1 "+3.3VP" H 9700 4690 30  0000 C CNN
F 2 "" H 9700 4600 60  0000 C CNN
F 3 "" H 9700 4600 60  0000 C CNN
	1    9700 4600
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 551949D6
P 9700 4400
F 0 "#PWR?" H 9700 4150 60  0001 C CNN
F 1 "GND" H 9700 4250 60  0000 C CNN
F 2 "" H 9700 4400 60  0000 C CNN
F 3 "" H 9700 4400 60  0000 C CNN
	1    9700 4400
	0    1    1    0   
$EndComp
$Comp
L GNDA #PWR?
U 1 1 55194AE9
P 9550 4500
F 0 "#PWR?" H 9550 4250 60  0001 C CNN
F 1 "GNDA" H 9550 4350 60  0000 C CNN
F 2 "" H 9550 4500 60  0000 C CNN
F 3 "" H 9550 4500 60  0000 C CNN
	1    9550 4500
	0    1    1    0   
$EndComp
$Comp
L CONN_01X08 P?
U 1 1 55194F0E
P 10450 4250
F 0 "P?" H 10450 4700 50  0000 C CNN
F 1 "CONN_01X08" V 10550 4250 50  0000 C CNN
F 2 "" H 10450 4250 60  0000 C CNN
F 3 "" H 10450 4250 60  0000 C CNN
	1    10450 4250
	1    0    0    -1  
$EndComp
$EndSCHEMATC