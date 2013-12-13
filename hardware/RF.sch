EESchema Schematic File Version 2
LIBS:stm32-rfid-clone
LIBS:stm32-rfid-clone-cache
EELAYER 24 0
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
L R R201
U 1 1 52AA8ADA
P 1150 4100
F 0 "R201" V 1095 4100 30  0000 C CNN
F 1 "1.2K" V 1205 4100 30  0000 C CNN
F 2 "" H 1150 4100 60  0000 C CNN
F 3 "" H 1150 4100 60  0000 C CNN
	1    1150 4100
	0    1    1    0   
$EndComp
$Comp
L NPN Q201
U 1 1 52AA8AE2
P 1550 4100
F 0 "Q201" H 1485 4255 40  0000 R BNN
F 1 "2N2222" H 1480 3960 40  0000 R TNN
F 2 "" H 1550 4100 60  0000 C CNN
F 3 "" H 1550 4100 60  0000 C CNN
	1    1550 4100
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR033
U 1 1 52AA8AE8
P 1600 4400
F 0 "#PWR033" H 1600 4400 30  0001 C CNN
F 1 "GNDPWR" H 1600 4330 30  0001 C CNN
F 2 "" H 1600 4400 60  0000 C CNN
F 3 "" H 1600 4400 60  0000 C CNN
	1    1600 4400
	1    0    0    -1  
$EndComp
$Comp
L DIODE D202
U 1 1 52AA8AF0
P 1600 3600
F 0 "D202" H 1600 3700 40  0000 C CNN
F 1 "DIODE" H 1600 3500 40  0000 C CNN
F 2 "" H 1600 3600 60  0000 C CNN
F 3 "" H 1600 3600 60  0000 C CNN
	1    1600 3600
	0    1    1    0   
$EndComp
$Comp
L DIODE D201
U 1 1 52AA8AF6
P 1600 3100
F 0 "D201" H 1600 3200 40  0000 C CNN
F 1 "DIODE" H 1600 3000 40  0000 C CNN
F 2 "" H 1600 3100 60  0000 C CNN
F 3 "" H 1600 3100 60  0000 C CNN
	1    1600 3100
	0    1    1    0   
$EndComp
$Comp
L R R202
U 1 1 52AA8AFC
P 1600 2600
F 0 "R202" V 1545 2600 30  0000 C CNN
F 1 "470R" V 1655 2600 30  0000 C CNN
F 2 "" H 1600 2600 60  0000 C CNN
F 3 "" H 1600 2600 60  0000 C CNN
	1    1600 2600
	1    0    0    -1  
$EndComp
$Comp
L NPN Q202
U 1 1 52AA8B02
P 2200 2800
F 0 "Q202" H 2135 2955 40  0000 R BNN
F 1 "2N2222" H 2130 2660 40  0000 R TNN
F 2 "" H 2200 2800 60  0000 C CNN
F 3 "" H 2200 2800 60  0000 C CNN
	1    2200 2800
	1    0    0    -1  
$EndComp
$Comp
L +9V #PWR034
U 1 1 52AA8B0E
P 1600 2150
F 0 "#PWR034" H 1600 2240 20  0001 C CNN
F 1 "+9V" H 1600 2240 30  0000 C CNN
F 2 "" H 1600 2150 60  0000 C CNN
F 3 "" H 1600 2150 60  0000 C CNN
	1    1600 2150
	1    0    0    -1  
$EndComp
$Comp
L R R203
U 1 1 52AA8B19
P 2250 3200
F 0 "R203" V 2195 3200 30  0000 C CNN
F 1 "10R" V 2305 3200 30  0000 C CNN
F 2 "" H 2250 3200 60  0000 C CNN
F 3 "" H 2250 3200 60  0000 C CNN
	1    2250 3200
	1    0    0    -1  
$EndComp
$Comp
L R R204
U 1 1 52AA8B1F
P 2250 3600
F 0 "R204" V 2195 3600 30  0000 C CNN
F 1 "10R" V 2305 3600 30  0000 C CNN
F 2 "" H 2250 3600 60  0000 C CNN
F 3 "" H 2250 3600 60  0000 C CNN
	1    2250 3600
	1    0    0    -1  
$EndComp
$Comp
L PNP Q203
U 1 1 52AA8B25
P 2200 4000
F 0 "Q203" H 2120 4150 40  0000 R BNN
F 1 "2N2907" H 2115 3865 40  0000 R TNN
F 2 "" H 2200 4000 60  0000 C CNN
F 3 "" H 2200 4000 60  0000 C CNN
	1    2200 4000
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR035
U 1 1 52AA8B33
P 2250 4400
F 0 "#PWR035" H 2250 4400 30  0001 C CNN
F 1 "GNDPWR" H 2250 4330 30  0001 C CNN
F 2 "" H 2250 4400 60  0000 C CNN
F 3 "" H 2250 4400 60  0000 C CNN
	1    2250 4400
	1    0    0    -1  
$EndComp
$Comp
L P02 P201
U 1 1 52AA8B3B
P 2600 3100
F 0 "P201" H 2650 3150 60  0000 C CNN
F 1 "ANT" H 2600 2750 60  0000 L CNN
F 2 "" H 2600 3100 60  0000 C CNN
F 3 "" H 2600 3100 60  0000 C CNN
	1    2600 3100
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D203
U 1 1 52AA8B44
P 3100 3400
F 0 "D203" H 3100 3500 40  0000 C CNN
F 1 "DIODE" H 3100 3300 40  0000 C CNN
F 2 "" H 3100 3400 60  0000 C CNN
F 3 "" H 3100 3400 60  0000 C CNN
	1    3100 3400
	1    0    0    -1  
$EndComp
$Comp
L C C202
U 1 1 52AA8B4D
P 2800 4000
F 0 "C202" H 2825 4050 30  0000 L CNN
F 1 "1000pF" H 2825 3950 30  0000 L CNN
F 2 "" H 2800 4000 60  0000 C CNN
F 3 "" H 2800 4000 60  0000 C CNN
	1    2800 4000
	1    0    0    -1  
$EndComp
$Comp
L C C203
U 1 1 52AA8B59
P 3400 4000
F 0 "C203" H 3425 4050 30  0000 L CNN
F 1 "2200pF" H 3425 3950 30  0000 L CNN
F 2 "" H 3400 4000 60  0000 C CNN
F 3 "" H 3400 4000 60  0000 C CNN
	1    3400 4000
	1    0    0    -1  
$EndComp
$Comp
L R R205
U 1 1 52AA8B60
P 3700 4000
F 0 "R205" V 3645 4000 30  0000 C CNN
F 1 "390K" V 3755 4000 30  0000 C CNN
F 2 "" H 3700 4000 60  0000 C CNN
F 3 "" H 3700 4000 60  0000 C CNN
	1    3700 4000
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR036
U 1 1 52AA8B66
P 2800 4400
F 0 "#PWR036" H 2800 4400 30  0001 C CNN
F 1 "GNDPWR" H 2800 4330 30  0001 C CNN
F 2 "" H 2800 4400 60  0000 C CNN
F 3 "" H 2800 4400 60  0000 C CNN
	1    2800 4400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR037
U 1 1 52AA8B6C
P 3400 4400
F 0 "#PWR037" H 3400 4400 30  0001 C CNN
F 1 "GNDPWR" H 3400 4330 30  0001 C CNN
F 2 "" H 3400 4400 60  0000 C CNN
F 3 "" H 3400 4400 60  0000 C CNN
	1    3400 4400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR038
U 1 1 52AA8B7A
P 3700 4400
F 0 "#PWR038" H 3700 4400 30  0001 C CNN
F 1 "GNDPWR" H 3700 4330 30  0001 C CNN
F 2 "" H 3700 4400 60  0000 C CNN
F 3 "" H 3700 4400 60  0000 C CNN
	1    3700 4400
	1    0    0    -1  
$EndComp
$Comp
L C C204
U 1 1 52AA8B81
P 3900 3400
F 0 "C204" H 3925 3450 30  0000 L CNN
F 1 "1000pF" H 3925 3350 30  0000 L CNN
F 2 "" H 3900 3400 60  0000 C CNN
F 3 "" H 3900 3400 60  0000 C CNN
	1    3900 3400
	0    -1   -1   0   
$EndComp
$Comp
L R R206
U 1 1 52AA8B87
P 4150 3050
F 0 "R206" V 4095 3050 30  0000 C CNN
F 1 "1M" V 4205 3050 30  0000 C CNN
F 2 "" H 4150 3050 60  0000 C CNN
F 3 "" H 4150 3050 60  0000 C CNN
	1    4150 3050
	1    0    0    -1  
$EndComp
$Comp
L R R208
U 1 1 52AA8B8D
P 4700 2850
F 0 "R208" V 4645 2850 30  0000 C CNN
F 1 "47.5K" V 4755 2850 30  0000 C CNN
F 2 "" H 4700 2850 60  0000 C CNN
F 3 "" H 4700 2850 60  0000 C CNN
	1    4700 2850
	0    -1   -1   0   
$EndComp
$Comp
L R R207
U 1 1 52AA8B93
P 4450 2350
F 0 "R207" V 4395 2350 30  0000 C CNN
F 1 "1.82K" V 4505 2350 30  0000 C CNN
F 2 "" H 4450 2350 60  0000 C CNN
F 3 "" H 4450 2350 60  0000 C CNN
	1    4450 2350
	0    -1   -1   0   
$EndComp
$Comp
L C C205
U 1 1 52AA8B99
P 4500 2650
F 0 "C205" H 4525 2700 30  0000 L CNN
F 1 "1000pF" H 4525 2600 30  0000 L CNN
F 2 "" H 4500 2650 60  0000 C CNN
F 3 "" H 4500 2650 60  0000 C CNN
	1    4500 2650
	0    -1   -1   0   
$EndComp
$Comp
L C C206
U 1 1 52AA8B9F
P 5050 2650
F 0 "C206" H 5075 2700 30  0000 L CNN
F 1 "1000pF" H 5075 2600 30  0000 L CNN
F 2 "" H 5050 2650 60  0000 C CNN
F 3 "" H 5050 2650 60  0000 C CNN
	1    5050 2650
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR039
U 1 1 52AA8BC0
P 4150 2150
F 0 "#PWR039" H 4150 2240 20  0001 C CNN
F 1 "+3.3V" H 4150 2240 30  0000 C CNN
F 2 "" H 4150 2150 60  0000 C CNN
F 3 "" H 4150 2150 60  0000 C CNN
	1    4150 2150
	1    0    0    -1  
$EndComp
$Comp
L TL084 U201
U 3 1 52AA8BC6
P 4750 3300
F 0 "U201" H 4750 3450 40  0000 L BNN
F 1 "TL084" H 4750 3150 40  0000 L TNN
F 2 "" H 4750 3300 60  0000 C CNN
F 3 "" H 4750 3300 60  0000 C CNN
	3    4750 3300
	1    0    0    1   
$EndComp
$Comp
L R R209
U 1 1 52AA8BCC
P 5450 2650
F 0 "R209" V 5395 2650 30  0000 C CNN
F 1 "2.67K" V 5505 2650 30  0000 C CNN
F 2 "" H 5450 2650 60  0000 C CNN
F 3 "" H 5450 2650 60  0000 C CNN
	1    5450 2650
	0    -1   -1   0   
$EndComp
$Comp
L C C207
U 1 1 52AA8BD2
P 5650 2850
F 0 "C207" H 5675 2900 30  0000 L CNN
F 1 "1000pF" H 5675 2800 30  0000 L CNN
F 2 "" H 5650 2850 60  0000 C CNN
F 3 "" H 5650 2850 60  0000 C CNN
	1    5650 2850
	1    0    0    -1  
$EndComp
$Comp
L C C208
U 1 1 52AA8BD8
P 6650 2850
F 0 "C208" H 6675 2900 30  0000 L CNN
F 1 "1000pF" H 6675 2800 30  0000 L CNN
F 2 "" H 6650 2850 60  0000 C CNN
F 3 "" H 6650 2850 60  0000 C CNN
	1    6650 2850
	1    0    0    -1  
$EndComp
$Comp
L R R211
U 1 1 52AA8BDE
P 6150 3100
F 0 "R211" V 6095 3100 30  0000 C CNN
F 1 "82.5K" V 6205 3100 30  0000 C CNN
F 2 "" H 6150 3100 60  0000 C CNN
F 3 "" H 6150 3100 60  0000 C CNN
	1    6150 3100
	0    -1   -1   0   
$EndComp
$Comp
L TL084 U201
U 2 1 52AA8BE4
P 6150 3550
F 0 "U201" H 6150 3700 40  0000 L BNN
F 1 "TL084" H 6150 3400 40  0000 L TNN
F 2 "" H 6150 3550 60  0000 C CNN
F 3 "" H 6150 3550 60  0000 C CNN
	2    6150 3550
	1    0    0    1   
$EndComp
$Comp
L R R210
U 1 1 52AA8BEA
P 5650 3900
F 0 "R210" V 5595 3900 30  0000 C CNN
F 1 "10M" V 5705 3900 30  0000 C CNN
F 2 "" H 5650 3900 60  0000 C CNN
F 3 "" H 5650 3900 60  0000 C CNN
	1    5650 3900
	-1   0    0    1   
$EndComp
$Comp
L +3.3V #PWR040
U 1 1 52AA8BFD
P 5400 3500
F 0 "#PWR040" H 5400 3590 20  0001 C CNN
F 1 "+3.3V" H 5400 3590 30  0000 C CNN
F 2 "" H 5400 3500 60  0000 C CNN
F 3 "" H 5400 3500 60  0000 C CNN
	1    5400 3500
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR041
U 1 1 52AA8C0A
P 5650 4400
F 0 "#PWR041" H 5650 4400 30  0001 C CNN
F 1 "GNDPWR" H 5650 4330 30  0001 C CNN
F 2 "" H 5650 4400 60  0000 C CNN
F 3 "" H 5650 4400 60  0000 C CNN
	1    5650 4400
	1    0    0    -1  
$EndComp
$Comp
L TL084 U201
U 4 1 52AA8C11
P 7900 3050
F 0 "U201" H 7900 3200 40  0000 L BNN
F 1 "TL084" H 7900 2900 40  0000 L TNN
F 2 "" H 7900 3050 60  0000 C CNN
F 3 "" H 7900 3050 60  0000 C CNN
	4    7900 3050
	1    0    0    1   
$EndComp
Text HLabel 750  4100 0    60   Input ~ 0
TX
$Comp
L R R212
U 1 1 52AA9A21
P 6850 3550
F 0 "R212" V 6795 3550 30  0000 C CNN
F 1 "10K" V 6905 3550 30  0000 C CNN
F 2 "" H 6850 3550 60  0000 C CNN
F 3 "" H 6850 3550 60  0000 C CNN
	1    6850 3550
	0    -1   -1   0   
$EndComp
$Comp
L R R213
U 1 1 52AA9A2C
P 7050 3350
F 0 "R213" V 6995 3350 30  0000 C CNN
F 1 "10K" V 7105 3350 30  0000 C CNN
F 2 "" H 7050 3350 60  0000 C CNN
F 3 "" H 7050 3350 60  0000 C CNN
	1    7050 3350
	-1   0    0    1   
$EndComp
$Comp
L C C210
U 1 1 52AA9B47
P 7550 3550
F 0 "C210" H 7575 3600 30  0000 L CNN
F 1 "1000pF" H 7575 3500 30  0000 L CNN
F 2 "" H 7550 3550 60  0000 C CNN
F 3 "" H 7550 3550 60  0000 C CNN
	1    7550 3550
	0    -1   -1   0   
$EndComp
$Comp
L C C209
U 1 1 52AA9BF1
P 7050 2850
F 0 "C209" H 7075 2900 30  0000 L CNN
F 1 "1000pF" H 7075 2800 30  0000 L CNN
F 2 "" H 7050 2850 60  0000 C CNN
F 3 "" H 7050 2850 60  0000 C CNN
	1    7050 2850
	1    0    0    -1  
$EndComp
$Comp
L R R214
U 1 1 52AA9C75
P 7250 2650
F 0 "R214" V 7195 2650 30  0000 C CNN
F 1 "10K" V 7305 2650 30  0000 C CNN
F 2 "" H 7250 2650 60  0000 C CNN
F 3 "" H 7250 2650 60  0000 C CNN
	1    7250 2650
	0    1    1    0   
$EndComp
$Comp
L R R215
U 1 1 52AA9C80
P 7900 2650
F 0 "R215" V 7845 2650 30  0000 C CNN
F 1 "15.5K" V 7955 2650 30  0000 C CNN
F 2 "" H 7900 2650 60  0000 C CNN
F 3 "" H 7900 2650 60  0000 C CNN
	1    7900 2650
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR042
U 1 1 52AA9E88
P 7050 2150
F 0 "#PWR042" H 7050 2240 20  0001 C CNN
F 1 "+3.3V" H 7050 2240 30  0000 C CNN
F 2 "" H 7050 2150 60  0000 C CNN
F 3 "" H 7050 2150 60  0000 C CNN
	1    7050 2150
	1    0    0    -1  
$EndComp
$Comp
L TL084 U201
U 1 1 52AA9E93
P 9250 3150
F 0 "U201" H 9250 3300 40  0000 L BNN
F 1 "TL084" H 9250 3000 40  0000 L TNN
F 2 "" H 9250 3150 60  0000 C CNN
F 3 "" H 9250 3150 60  0000 C CNN
	1    9250 3150
	1    0    0    1   
$EndComp
$Comp
L R R216
U 1 1 52AA9EA8
P 8700 2750
F 0 "R216" V 8645 2750 30  0000 C CNN
F 1 "10K" V 8755 2750 30  0000 C CNN
F 2 "" H 8700 2750 60  0000 C CNN
F 3 "" H 8700 2750 60  0000 C CNN
	1    8700 2750
	1    0    0    -1  
$EndComp
$Comp
L R R218
U 1 1 52AA9EB3
P 8700 3750
F 0 "R218" V 8645 3750 30  0000 C CNN
F 1 "10K" V 8755 3750 30  0000 C CNN
F 2 "" H 8700 3750 60  0000 C CNN
F 3 "" H 8700 3750 60  0000 C CNN
	1    8700 3750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR043
U 1 1 52AA9FEC
P 8700 2150
F 0 "#PWR043" H 8700 2240 20  0001 C CNN
F 1 "+3.3V" H 8700 2240 30  0000 C CNN
F 2 "" H 8700 2150 60  0000 C CNN
F 3 "" H 8700 2150 60  0000 C CNN
	1    8700 2150
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR044
U 1 1 52AAA178
P 8700 4400
F 0 "#PWR044" H 8700 4400 30  0001 C CNN
F 1 "GNDPWR" H 8700 4330 30  0001 C CNN
F 2 "" H 8700 4400 60  0000 C CNN
F 3 "" H 8700 4400 60  0000 C CNN
	1    8700 4400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR045
U 1 1 52AAA183
P 9350 2800
F 0 "#PWR045" H 9350 2800 30  0001 C CNN
F 1 "GNDPWR" H 9350 2730 30  0001 C CNN
F 2 "" H 9350 2800 60  0000 C CNN
F 3 "" H 9350 2800 60  0000 C CNN
	1    9350 2800
	1    0    0    -1  
$EndComp
$Comp
L +9V #PWR046
U 1 1 52AAA1BB
P 9350 3600
F 0 "#PWR046" H 9350 3690 20  0001 C CNN
F 1 "+9V" H 9350 3690 30  0000 C CNN
F 2 "" H 9350 3600 60  0000 C CNN
F 3 "" H 9350 3600 60  0000 C CNN
	1    9350 3600
	1    0    0    -1  
$EndComp
$Comp
L R R219
U 1 1 52AAA3A3
P 9750 3350
F 0 "R219" V 9695 3350 30  0000 C CNN
F 1 "6.2K" V 9805 3350 30  0000 C CNN
F 2 "" H 9750 3350 60  0000 C CNN
F 3 "" H 9750 3350 60  0000 C CNN
	1    9750 3350
	1    0    0    -1  
$EndComp
$Comp
L R R220
U 1 1 52AAA3B8
P 9750 3750
F 0 "R220" V 9695 3750 30  0000 C CNN
F 1 "3.6K" V 9805 3750 30  0000 C CNN
F 2 "" H 9750 3750 60  0000 C CNN
F 3 "" H 9750 3750 60  0000 C CNN
	1    9750 3750
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR047
U 1 1 52AAA3C3
P 9750 4400
F 0 "#PWR047" H 9750 4400 30  0001 C CNN
F 1 "GNDPWR" H 9750 4330 30  0001 C CNN
F 2 "" H 9750 4400 60  0000 C CNN
F 3 "" H 9750 4400 60  0000 C CNN
	1    9750 4400
	1    0    0    -1  
$EndComp
Text HLabel 10100 3550 2    60   Output ~ 0
RX
$Comp
L CVAR C201
U 1 1 52AA9035
P 2550 4000
F 0 "C201" H 2575 4050 30  0000 L CNN
F 1 "100pF" H 2575 3950 30  0000 L CNN
F 2 "" H 2550 4000 60  0000 C CNN
F 3 "" H 2550 4000 60  0000 C CNN
	1    2550 4000
	1    0    0    -1  
$EndComp
$Comp
L POT R217
U 1 1 52AA90EF
P 8700 3250
F 0 "R217" V 8645 3250 30  0000 C CNN
F 1 "500R" V 8750 3350 30  0000 C CNN
F 2 "" H 8700 3250 60  0000 C CNN
F 3 "" H 8700 3250 60  0000 C CNN
	1    8700 3250
	1    0    0    -1  
$EndComp
$Comp
L TP TP202
U 1 1 52AA91DF
P 3400 3250
F 0 "TP202" H 3400 3450 40  0000 C CNN
F 1 "RF1" H 3430 3310 40  0000 L CNN
F 2 "" H 3400 3250 60  0000 C CNN
F 3 "" H 3400 3250 60  0000 C CNN
	1    3400 3250
	1    0    0    -1  
$EndComp
$Comp
L TP TP203
U 1 1 52AA97AC
P 5250 2550
F 0 "TP203" H 5250 2750 40  0000 C CNN
F 1 "RF2" H 5280 2610 40  0000 L CNN
F 2 "" H 5250 2550 60  0000 C CNN
F 3 "" H 5250 2550 60  0000 C CNN
	1    5250 2550
	1    0    0    -1  
$EndComp
$Comp
L TP TP204
U 1 1 52AA983B
P 6650 3650
F 0 "TP204" H 6650 3850 40  0000 C CNN
F 1 "RF3" H 6680 3710 40  0000 L CNN
F 2 "" H 6650 3650 60  0000 C CNN
F 3 "" H 6650 3650 60  0000 C CNN
	1    6650 3650
	-1   0    0    1   
$EndComp
$Comp
L TP TP205
U 1 1 52AA993A
P 8400 2550
F 0 "TP205" H 8400 2750 40  0000 C CNN
F 1 "RF4" H 8430 2610 40  0000 L CNN
F 2 "" H 8400 2550 60  0000 C CNN
F 3 "" H 8400 2550 60  0000 C CNN
	1    8400 2550
	1    0    0    -1  
$EndComp
$Comp
L TP TP206
U 1 1 52AA9A30
P 10000 3400
F 0 "TP206" H 10000 3600 40  0000 C CNN
F 1 "RFRX" H 10030 3460 40  0000 L CNN
F 2 "" H 10000 3400 60  0000 C CNN
F 3 "" H 10000 3400 60  0000 C CNN
	1    10000 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 4100 750  4100
Wire Wire Line
	1600 4300 1600 4400
Wire Wire Line
	1250 4100 1350 4100
Connection ~ 1600 2800
Wire Wire Line
	1600 2150 1600 2500
Wire Wire Line
	2250 2600 2250 2400
Wire Wire Line
	2250 2400 1600 2400
Connection ~ 1600 2400
Wire Wire Line
	1600 3850 1850 3850
Wire Wire Line
	1850 3850 1850 4000
Wire Wire Line
	1850 4000 2000 4000
Connection ~ 1600 3850
Wire Wire Line
	2250 3800 2250 3700
Wire Wire Line
	2250 3300 2250 3500
Wire Wire Line
	2250 3100 2250 3000
Wire Wire Line
	2250 4200 2250 4400
Wire Wire Line
	2000 2800 1600 2800
Wire Wire Line
	2250 3400 2700 3400
Wire Wire Line
	2700 3400 2700 3300
Connection ~ 2250 3400
Wire Wire Line
	2800 3300 2800 3900
Connection ~ 2800 3400
Wire Wire Line
	2800 4100 2800 4400
Wire Wire Line
	3400 4100 3400 4400
Wire Wire Line
	3400 3250 3400 3900
Wire Wire Line
	3700 3400 3700 3900
Connection ~ 3400 3400
Wire Wire Line
	3700 4100 3700 4400
Connection ~ 3700 3400
Wire Wire Line
	4000 3400 4350 3400
Wire Wire Line
	4150 3400 4150 3150
Connection ~ 4150 3400
Wire Wire Line
	4300 3200 4350 3200
Wire Wire Line
	4300 2650 4300 3200
Wire Wire Line
	4300 2850 4600 2850
Wire Wire Line
	4800 2850 5250 2850
Wire Wire Line
	5250 2550 5250 3300
Wire Wire Line
	5250 3300 5150 3300
Wire Wire Line
	4400 2650 4300 2650
Connection ~ 4300 2850
Wire Wire Line
	5150 2650 5350 2650
Connection ~ 5250 2850
Wire Wire Line
	4600 2650 4950 2650
Wire Wire Line
	4550 2350 4750 2350
Wire Wire Line
	4750 2350 4750 2650
Connection ~ 4750 2650
Wire Wire Line
	4350 2350 4150 2350
Wire Wire Line
	4150 2150 4150 2950
Connection ~ 4150 2350
Connection ~ 5250 2650
Wire Wire Line
	5550 2650 6650 2650
Wire Wire Line
	5650 2650 5650 2750
Connection ~ 5650 2650
Wire Wire Line
	6050 3100 5650 3100
Wire Wire Line
	5650 2950 5650 3800
Wire Wire Line
	6250 3100 6650 3100
Wire Wire Line
	5650 3450 5750 3450
Connection ~ 5650 3100
Connection ~ 5650 3450
Wire Wire Line
	5750 3650 5400 3650
Wire Wire Line
	5400 3650 5400 3500
Wire Wire Line
	6650 2650 6650 2750
Wire Wire Line
	6650 2950 6650 3650
Wire Wire Line
	6550 3550 6750 3550
Connection ~ 6650 3100
Wire Wire Line
	5650 4000 5650 4400
Connection ~ 6650 3550
Wire Wire Line
	6950 3550 7450 3550
Wire Wire Line
	7050 3550 7050 3450
Wire Wire Line
	7050 2950 7050 3250
Wire Wire Line
	7050 3150 7500 3150
Connection ~ 7050 3550
Wire Wire Line
	8400 3550 7650 3550
Wire Wire Line
	8400 2550 8400 3550
Wire Wire Line
	8300 3050 8850 3050
Connection ~ 7050 3150
Wire Wire Line
	7500 2950 7450 2950
Wire Wire Line
	7450 2950 7450 2650
Wire Wire Line
	7350 2650 7800 2650
Connection ~ 7450 2650
Wire Wire Line
	7150 2650 7050 2650
Wire Wire Line
	7050 2150 7050 2750
Connection ~ 7050 2650
Wire Wire Line
	8000 2650 8400 2650
Connection ~ 8400 3050
Wire Wire Line
	8700 2650 8700 2150
Wire Wire Line
	8700 3850 8700 4400
Wire Wire Line
	9750 3850 9750 4400
Wire Wire Line
	9750 3450 9750 3650
Wire Wire Line
	9750 3250 9750 3150
Wire Wire Line
	9750 3150 9650 3150
Wire Wire Line
	9750 3550 10100 3550
Connection ~ 9750 3550
Wire Wire Line
	1600 2700 1600 3000
Wire Wire Line
	1600 3200 1600 3500
Wire Wire Line
	1600 3700 1600 3900
Wire Wire Line
	2800 3400 3000 3400
Wire Wire Line
	3200 3400 3800 3400
Wire Wire Line
	2800 3750 2550 3750
Wire Wire Line
	2550 3750 2550 3900
Connection ~ 2800 3750
Wire Wire Line
	2550 4100 2550 4250
Wire Wire Line
	2550 4250 2800 4250
Connection ~ 2800 4250
Wire Wire Line
	8850 3250 8800 3250
Wire Wire Line
	8700 3350 8700 3650
Wire Wire Line
	8700 3150 8700 2850
Wire Wire Line
	9150 2850 9150 2700
Wire Wire Line
	9150 2700 9350 2700
Wire Wire Line
	9350 2700 9350 2800
Wire Wire Line
	9350 3600 9350 3700
Wire Wire Line
	9350 3700 9150 3700
Wire Wire Line
	9150 3700 9150 3450
Connection ~ 8400 2650
Wire Wire Line
	10000 3400 10000 3550
Connection ~ 10000 3550
$Comp
L TP TP201
U 1 1 52AA9CC1
P 950 4000
F 0 "TP201" H 950 4200 40  0000 C CNN
F 1 "RFTX" H 980 4060 40  0000 L CNN
F 2 "" H 950 4000 60  0000 C CNN
F 3 "" H 950 4000 60  0000 C CNN
	1    950  4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  4000 950  4100
Connection ~ 950  4100
$EndSCHEMATC
