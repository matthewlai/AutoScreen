EESchema Schematic File Version 4
LIBS:autoscreen-cache
EELAYER 30 0
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
L MCU_ST_STM32F7:STM32F722RETx U2
U 1 1 5D7213F8
P 3650 4850
F 0 "U2" H 3300 6700 50  0000 C CNN
F 1 "STM32F722RETx" H 3050 6600 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 3050 3150 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00330506.pdf" H 3650 4850 50  0001 C CNN
	1    3650 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5D724D8D
P 3650 6950
F 0 "#PWR012" H 3650 6700 50  0001 C CNN
F 1 "GND" H 3655 6777 50  0000 C CNN
F 2 "" H 3650 6950 50  0001 C CNN
F 3 "" H 3650 6950 50  0001 C CNN
	1    3650 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 6650 3450 6800
Wire Wire Line
	3450 6800 3550 6800
Wire Wire Line
	3650 6800 3650 6950
Wire Wire Line
	3550 6650 3550 6800
Connection ~ 3550 6800
Wire Wire Line
	3550 6800 3650 6800
Wire Wire Line
	3650 6650 3650 6800
Connection ~ 3650 6800
Wire Wire Line
	3650 6800 3750 6800
Wire Wire Line
	3750 6800 3750 6650
Wire Wire Line
	3750 6800 3850 6800
Wire Wire Line
	3850 6800 3850 6650
Connection ~ 3750 6800
$Comp
L power:+3V3 #PWR011
U 1 1 5D726017
P 3650 2850
F 0 "#PWR011" H 3650 2700 50  0001 C CNN
F 1 "+3V3" H 3665 3023 50  0000 C CNN
F 2 "" H 3650 2850 50  0001 C CNN
F 3 "" H 3650 2850 50  0001 C CNN
	1    3650 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2850 3650 2950
Wire Wire Line
	3450 3050 3450 2950
Wire Wire Line
	3450 2950 3550 2950
Connection ~ 3650 2950
Wire Wire Line
	3650 2950 3650 3050
Wire Wire Line
	3550 3050 3550 2950
Connection ~ 3550 2950
Wire Wire Line
	3550 2950 3650 2950
Wire Wire Line
	3650 2950 3750 2950
Wire Wire Line
	3750 2950 3750 3050
Wire Wire Line
	3750 2950 3850 2950
Wire Wire Line
	3850 2950 3850 3050
Connection ~ 3750 2950
$Comp
L Switch:SW_Push SW2
U 1 1 5D727384
P 5300 1750
F 0 "SW2" V 5254 1898 50  0000 L CNN
F 1 "SW_Push" V 5345 1898 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_SPST_B3S-1000" H 5300 1950 50  0001 C CNN
F 3 "~" H 5300 1950 50  0001 C CNN
	1    5300 1750
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5D7285C2
P 4350 1200
F 0 "SW1" V 4304 1348 50  0000 L CNN
F 1 "SW_Push" V 4395 1348 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_SPST_B3S-1000" H 4350 1400 50  0001 C CNN
F 3 "~" H 4350 1400 50  0001 C CNN
	1    4350 1200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5D728E4F
P 5300 2050
F 0 "#PWR022" H 5300 1800 50  0001 C CNN
F 1 "GND" H 5305 1877 50  0000 C CNN
F 2 "" H 5300 2050 50  0001 C CNN
F 3 "" H 5300 2050 50  0001 C CNN
	1    5300 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2050 5300 2000
$Comp
L Device:C C16
U 1 1 5D729891
P 5000 1750
F 0 "C16" H 5115 1796 50  0000 L CNN
F 1 "0.1uF" H 5115 1705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5038 1600 50  0001 C CNN
F 3 "~" H 5000 1750 50  0001 C CNN
	1    5000 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1600 5000 1450
Wire Wire Line
	5000 1450 5150 1450
Wire Wire Line
	5300 1450 5300 1550
Wire Wire Line
	5000 1900 5000 2000
Wire Wire Line
	5000 2000 5300 2000
Connection ~ 5300 2000
Wire Wire Line
	5300 2000 5300 1950
Text Label 5150 1350 0    50   ~ 0
NRST
Wire Wire Line
	5150 1350 5150 1450
Connection ~ 5150 1450
Wire Wire Line
	5150 1450 5300 1450
Text Label 2800 3250 2    50   ~ 0
NRST
Wire Wire Line
	2800 3250 2950 3250
Text Label 2800 3450 2    50   ~ 0
BOOT0
Wire Wire Line
	2800 3450 2950 3450
Text Label 4250 1450 2    50   ~ 0
BOOT0
$Comp
L power:GND #PWR015
U 1 1 5D72C05E
P 4350 2050
F 0 "#PWR015" H 4350 1800 50  0001 C CNN
F 1 "GND" H 4355 1877 50  0000 C CNN
F 2 "" H 4350 2050 50  0001 C CNN
F 3 "" H 4350 2050 50  0001 C CNN
	1    4350 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2050 4350 1950
Wire Wire Line
	4350 1400 4350 1450
Wire Wire Line
	4250 1450 4350 1450
Connection ~ 4350 1450
Wire Wire Line
	4350 1450 4350 1650
$Comp
L power:+3V3 #PWR014
U 1 1 5D72FB9F
P 4350 900
F 0 "#PWR014" H 4350 750 50  0001 C CNN
F 1 "+3V3" H 4365 1073 50  0000 C CNN
F 2 "" H 4350 900 50  0001 C CNN
F 3 "" H 4350 900 50  0001 C CNN
	1    4350 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 900  4350 1000
$Comp
L Device:R R3
U 1 1 5D730ACD
P 4350 1800
F 0 "R3" H 4420 1846 50  0000 L CNN
F 1 "10kR" H 4420 1755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4280 1800 50  0001 C CNN
F 3 "~" H 4350 1800 50  0001 C CNN
	1    4350 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5D730F33
P 2200 3800
F 0 "C7" H 2315 3846 50  0000 L CNN
F 1 "4.7uF" H 2315 3755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2238 3650 50  0001 C CNN
F 3 "~" H 2200 3800 50  0001 C CNN
	1    2200 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5D7315A8
P 2200 4050
F 0 "#PWR05" H 2200 3800 50  0001 C CNN
F 1 "GND" H 2205 3877 50  0000 C CNN
F 2 "" H 2200 4050 50  0001 C CNN
F 3 "" H 2200 4050 50  0001 C CNN
	1    2200 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR08
U 1 1 5D732B69
P 2850 3750
F 0 "#PWR08" H 2850 3600 50  0001 C CNN
F 1 "+3V3" V 2865 3878 50  0000 L CNN
F 2 "" H 2850 3750 50  0001 C CNN
F 3 "" H 2850 3750 50  0001 C CNN
	1    2850 3750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 4050 2200 3950
Wire Wire Line
	2200 3650 2950 3650
Wire Wire Line
	2850 3750 2950 3750
$Comp
L Device:Q_Photo_NPN_EC Q1
U 1 1 5D738E26
P 5150 3600
F 0 "Q1" H 5341 3646 50  0000 L CNN
F 1 "Q_Photo_NPN_EC" H 5341 3555 50  0000 L CNN
F 2 "LED_THT:LED_D5.0mm_Horizontal_O1.27mm_Z3.0mm" H 5350 3700 50  0001 C CNN
F 3 "~" H 5150 3600 50  0001 C CNN
	1    5150 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5D73B314
P 5200 5800
F 0 "D2" V 5239 5683 50  0000 R CNN
F 1 "LED" V 5148 5683 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm_Horizontal_O1.27mm_Z3.0mm_IRBlack" H 5200 5800 50  0001 C CNN
F 3 "~" H 5200 5800 50  0001 C CNN
	1    5200 5800
	0    -1   -1   0   
$EndComp
Text Notes 5350 6150 0    50   ~ 0
Vf=1.4V typical at 100mA\n3.3V - 1.4V = 1.9V\n1.9V / 20mA = 100 ohms (per pin)
$Comp
L power:+3V3 #PWR019
U 1 1 5D73E106
P 5200 5600
F 0 "#PWR019" H 5200 5450 50  0001 C CNN
F 1 "+3V3" H 5215 5773 50  0000 C CNN
F 2 "" H 5200 5600 50  0001 C CNN
F 3 "" H 5200 5600 50  0001 C CNN
	1    5200 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 5600 5200 5650
$Comp
L Device:R R4
U 1 1 5D73F6C8
P 4500 6150
F 0 "R4" V 4450 6350 50  0000 C CNN
F 1 "100R" V 4500 6150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4430 6150 50  0001 C CNN
F 3 "~" H 4500 6150 50  0001 C CNN
	1    4500 6150
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5D73FB65
P 4500 6250
F 0 "R5" V 4450 6450 50  0000 C CNN
F 1 "100R" V 4500 6250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4430 6250 50  0001 C CNN
F 3 "~" H 4500 6250 50  0001 C CNN
	1    4500 6250
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5D73FD11
P 4500 6350
F 0 "R6" V 4450 6550 50  0000 C CNN
F 1 "100R" V 4500 6350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4430 6350 50  0001 C CNN
F 3 "~" H 4500 6350 50  0001 C CNN
	1    4500 6350
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5D73FFDD
P 4500 6450
F 0 "R7" V 4450 6650 50  0000 C CNN
F 1 "100R" V 4500 6450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4430 6450 50  0001 C CNN
F 3 "~" H 4500 6450 50  0001 C CNN
	1    4500 6450
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 6150 4350 6150
Wire Wire Line
	4250 6250 4350 6250
Wire Wire Line
	4250 6350 4350 6350
Wire Wire Line
	4250 6450 4350 6450
Wire Wire Line
	5200 6450 5200 6350
Connection ~ 5200 6350
Wire Wire Line
	5200 6350 5200 6250
Connection ~ 5200 6250
Wire Wire Line
	5200 6250 5200 6150
Connection ~ 5200 6150
Wire Wire Line
	5200 6150 5200 5950
$Comp
L power:GND #PWR021
U 1 1 5D74CC7F
P 5250 3900
F 0 "#PWR021" H 5250 3650 50  0001 C CNN
F 1 "GND" H 5255 3727 50  0000 C CNN
F 2 "" H 5250 3900 50  0001 C CNN
F 3 "" H 5250 3900 50  0001 C CNN
	1    5250 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3900 5250 3800
$Comp
L Device:R R10
U 1 1 5D74DBBF
P 5250 3150
F 0 "R10" H 5320 3196 50  0000 L CNN
F 1 "470R" H 5320 3105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5180 3150 50  0001 C CNN
F 3 "~" H 5250 3150 50  0001 C CNN
	1    5250 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR020
U 1 1 5D74ECD4
P 5250 2950
F 0 "#PWR020" H 5250 2800 50  0001 C CNN
F 1 "+3V3" H 5265 3123 50  0000 C CNN
F 2 "" H 5250 2950 50  0001 C CNN
F 3 "" H 5250 2950 50  0001 C CNN
	1    5250 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2950 5250 3000
$Comp
L Device:Crystal Y1
U 1 1 5D756C7B
P 2250 4650
F 0 "Y1" H 2250 4382 50  0000 C CNN
F 1 "12MHz" H 2250 4473 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_HC49-SD_HandSoldering" H 2250 4650 50  0001 C CNN
F 3 "~" H 2250 4650 50  0001 C CNN
	1    2250 4650
	-1   0    0    1   
$EndComp
$Comp
L Device:C C9
U 1 1 5D75ED05
P 2500 4850
F 0 "C9" H 2615 4896 50  0000 L CNN
F 1 "16pF" H 2615 4805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2538 4700 50  0001 C CNN
F 3 "~" H 2500 4850 50  0001 C CNN
	1    2500 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4650 2500 4650
Wire Wire Line
	2000 4550 2000 4650
Wire Wire Line
	2000 4650 2100 4650
Wire Wire Line
	2000 4550 2950 4550
$Comp
L Device:C C5
U 1 1 5D7653ED
P 2000 4850
F 0 "C5" H 2115 4896 50  0000 L CNN
F 1 "16pF" H 2115 4805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2038 4700 50  0001 C CNN
F 3 "~" H 2000 4850 50  0001 C CNN
	1    2000 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 4700 2500 4650
Connection ~ 2500 4650
Wire Wire Line
	2500 4650 2950 4650
Wire Wire Line
	2000 4700 2000 4650
Connection ~ 2000 4650
$Comp
L power:GND #PWR07
U 1 1 5D768183
P 2500 5050
F 0 "#PWR07" H 2500 4800 50  0001 C CNN
F 1 "GND" H 2505 4877 50  0000 C CNN
F 2 "" H 2500 5050 50  0001 C CNN
F 3 "" H 2500 5050 50  0001 C CNN
	1    2500 5050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5D76880B
P 2000 5050
F 0 "#PWR04" H 2000 4800 50  0001 C CNN
F 1 "GND" H 2005 4877 50  0000 C CNN
F 2 "" H 2000 5050 50  0001 C CNN
F 3 "" H 2000 5050 50  0001 C CNN
	1    2000 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 5050 2000 5000
Wire Wire Line
	2500 5000 2500 5050
$Comp
L Connector:USB_B_Micro J1
U 1 1 5D773DBB
P 950 2900
F 0 "J1" H 1007 3367 50  0000 C CNN
F 1 "USB_B_Micro" H 1007 3276 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1100 2850 50  0001 C CNN
F 3 "~" H 1100 2850 50  0001 C CNN
	1    950  2900
	1    0    0    -1  
$EndComp
Text Label 1400 2700 0    50   ~ 0
Vbus
Wire Wire Line
	1400 2700 1250 2700
Text Label 1400 2900 0    50   ~ 0
USB_DP
Wire Wire Line
	1400 2900 1250 2900
Text Label 1400 3000 0    50   ~ 0
USB_DM
Wire Wire Line
	1400 3000 1250 3000
$Comp
L power:GND #PWR02
U 1 1 5D77B6E3
P 850 3450
F 0 "#PWR02" H 850 3200 50  0001 C CNN
F 1 "GND" H 855 3277 50  0000 C CNN
F 2 "" H 850 3450 50  0001 C CNN
F 3 "" H 850 3450 50  0001 C CNN
	1    850  3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  3450 850  3350
Wire Wire Line
	850  3350 950  3350
Wire Wire Line
	950  3350 950  3300
Connection ~ 850  3350
Wire Wire Line
	850  3350 850  3300
Text Label 4500 4350 0    50   ~ 0
USB_DM
Wire Wire Line
	4500 4350 4250 4350
Text Label 4500 4450 0    50   ~ 0
USB_DP
Wire Wire Line
	4500 4450 4250 4450
Text Label 4500 4150 0    50   ~ 0
Vbus
Wire Wire Line
	4500 4150 4250 4150
Text Label 800  1100 2    50   ~ 0
Vbus
Wire Wire Line
	800  1100 850  1100
$Comp
L Device:D_Schottky D1
U 1 1 5D790430
P 1050 1100
F 0 "D1" H 1050 884 50  0000 C CNN
F 1 "D_Schottky" H 1050 975 50  0000 C CNN
F 2 "Diode_SMD:D_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 1050 1100 50  0001 C CNN
F 3 "~" H 1050 1100 50  0001 C CNN
	1    1050 1100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 5D7911FA
P 1400 1100
F 0 "R2" V 1193 1100 50  0000 C CNN
F 1 "4.7R" V 1284 1100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1330 1100 50  0001 C CNN
F 3 "~" H 1400 1100 50  0001 C CNN
	1    1400 1100
	0    1    1    0   
$EndComp
$Comp
L Device:CP C1
U 1 1 5D793C36
P 1150 1550
F 0 "C1" H 1100 1750 50  0000 L CNN
F 1 "2.2mF" H 1050 1700 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 1188 1400 50  0001 C CNN
F 3 "~" H 1150 1550 50  0001 C CNN
	1    1150 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5D794284
P 1400 1550
F 0 "C2" H 1350 1750 50  0000 L CNN
F 1 "2.2mF" H 1300 1700 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 1438 1400 50  0001 C CNN
F 3 "~" H 1400 1550 50  0001 C CNN
	1    1400 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C3
U 1 1 5D7947C7
P 1650 1550
F 0 "C3" H 1600 1750 50  0000 L CNN
F 1 "2.2mF" H 1550 1700 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 1688 1400 50  0001 C CNN
F 3 "~" H 1650 1550 50  0001 C CNN
	1    1650 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C4
U 1 1 5D794B8F
P 1900 1550
F 0 "C4" H 1850 1750 50  0000 L CNN
F 1 "2.2mF" H 1800 1700 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 1938 1400 50  0001 C CNN
F 3 "~" H 1900 1550 50  0001 C CNN
	1    1900 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C6
U 1 1 5D798950
P 2150 1550
F 0 "C6" H 2100 1750 50  0000 L CNN
F 1 "2.2mF" H 2050 1700 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 2188 1400 50  0001 C CNN
F 3 "~" H 2150 1550 50  0001 C CNN
	1    2150 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1100 1650 1250
Wire Wire Line
	1150 1400 1150 1250
Wire Wire Line
	1150 1250 1400 1250
Connection ~ 1650 1250
Wire Wire Line
	1650 1250 1650 1400
Wire Wire Line
	1400 1400 1400 1250
Connection ~ 1400 1250
Wire Wire Line
	1400 1250 1650 1250
Wire Wire Line
	1650 1250 1900 1250
Wire Wire Line
	1900 1250 1900 1400
Wire Wire Line
	1900 1250 2150 1250
Wire Wire Line
	2150 1250 2150 1400
Connection ~ 1900 1250
Wire Wire Line
	1150 1700 1150 1800
Wire Wire Line
	1150 1800 1400 1800
Wire Wire Line
	2150 1800 2150 1700
Wire Wire Line
	1900 1700 1900 1800
Connection ~ 1900 1800
Wire Wire Line
	1900 1800 2150 1800
Wire Wire Line
	1650 1700 1650 1800
Connection ~ 1650 1800
Wire Wire Line
	1650 1800 1900 1800
Wire Wire Line
	1400 1700 1400 1800
Connection ~ 1400 1800
Wire Wire Line
	1400 1800 1650 1800
$Comp
L power:GND #PWR03
U 1 1 5D7B230E
P 1650 1900
F 0 "#PWR03" H 1650 1650 50  0001 C CNN
F 1 "GND" H 1655 1727 50  0000 C CNN
F 2 "" H 1650 1900 50  0001 C CNN
F 3 "" H 1650 1900 50  0001 C CNN
	1    1650 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1900 1650 1800
$Comp
L Regulator_Linear:AZ1117-3.3 U1
U 1 1 5D7C555D
P 3000 1100
F 0 "U1" H 3000 1342 50  0000 C CNN
F 1 "AZ1117-3.3" H 3000 1251 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 3000 1350 50  0001 C CIN
F 3 "https://www.diodes.com/assets/Datasheets/AZ1117.pdf" H 3000 1100 50  0001 C CNN
	1    3000 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5D7C60CA
P 3000 1900
F 0 "#PWR09" H 3000 1650 50  0001 C CNN
F 1 "GND" H 3005 1727 50  0000 C CNN
F 2 "" H 3000 1900 50  0001 C CNN
F 3 "" H 3000 1900 50  0001 C CNN
	1    3000 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1100 2500 1100
Connection ~ 1650 1100
$Comp
L power:+3V3 #PWR013
U 1 1 5D7CCC53
P 3750 1000
F 0 "#PWR013" H 3750 850 50  0001 C CNN
F 1 "+3V3" H 3765 1173 50  0000 C CNN
F 2 "" H 3750 1000 50  0001 C CNN
F 3 "" H 3750 1000 50  0001 C CNN
	1    3750 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1000 3750 1100
Wire Wire Line
	3750 1100 3400 1100
$Comp
L Device:C C8
U 1 1 5D7DC168
P 2500 1550
F 0 "C8" H 2615 1596 50  0000 L CNN
F 1 "10uF" H 2615 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2538 1400 50  0001 C CNN
F 3 "~" H 2500 1550 50  0001 C CNN
	1    2500 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1400 2500 1100
Wire Wire Line
	2700 1100 2500 1100
Connection ~ 2500 1100
$Comp
L power:GND #PWR06
U 1 1 5D7E3C29
P 2500 1900
F 0 "#PWR06" H 2500 1650 50  0001 C CNN
F 1 "GND" H 2505 1727 50  0000 C CNN
F 2 "" H 2500 1900 50  0001 C CNN
F 3 "" H 2500 1900 50  0001 C CNN
	1    2500 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5D7E7AEA
P 3400 1550
F 0 "C10" H 3515 1596 50  0000 L CNN
F 1 "10uF" H 3515 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3438 1400 50  0001 C CNN
F 3 "~" H 3400 1550 50  0001 C CNN
	1    3400 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5D7E7DFB
P 3400 1900
F 0 "#PWR010" H 3400 1650 50  0001 C CNN
F 1 "GND" H 3405 1727 50  0000 C CNN
F 2 "" H 3400 1900 50  0001 C CNN
F 3 "" H 3400 1900 50  0001 C CNN
	1    3400 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1400 3400 1100
Wire Wire Line
	3300 1100 3400 1100
Connection ~ 3400 1100
Text Label 5250 4250 0    50   ~ 0
Vbus
$Comp
L Device:R R8
U 1 1 5D7FDE53
P 5200 4450
F 0 "R8" H 5270 4496 50  0000 L CNN
F 1 "10kR" H 5270 4405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5130 4450 50  0001 C CNN
F 3 "~" H 5200 4450 50  0001 C CNN
	1    5200 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5D7FEBC2
P 5200 4850
F 0 "R9" H 5270 4896 50  0000 L CNN
F 1 "10kR" H 5270 4805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5130 4850 50  0001 C CNN
F 3 "~" H 5200 4850 50  0001 C CNN
	1    5200 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4700 5200 4650
Wire Wire Line
	5200 4300 5200 4250
Wire Wire Line
	5200 4250 5250 4250
$Comp
L power:GND #PWR018
U 1 1 5D80712B
P 5200 5050
F 0 "#PWR018" H 5200 4800 50  0001 C CNN
F 1 "GND" H 5205 4877 50  0000 C CNN
F 2 "" H 5200 5050 50  0001 C CNN
F 3 "" H 5200 5050 50  0001 C CNN
	1    5200 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 5000 5200 5050
Text Label 5300 4650 0    50   ~ 0
Vbus_div2
Wire Wire Line
	5300 4650 5200 4650
Connection ~ 5200 4650
Wire Wire Line
	5200 4650 5200 4600
Text Label 2900 5050 2    50   ~ 0
Vbus_div2
$Comp
L Device:R R1
U 1 1 5D814639
P 850 1450
F 0 "R1" H 1000 1400 50  0000 R CNN
F 1 "10kR" H 1100 1500 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 780 1450 50  0001 C CNN
F 3 "~" H 850 1450 50  0001 C CNN
	1    850  1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	850  1300 850  1100
Connection ~ 850  1100
Wire Wire Line
	850  1100 900  1100
$Comp
L power:GND #PWR01
U 1 1 5D819497
P 850 1900
F 0 "#PWR01" H 850 1650 50  0001 C CNN
F 1 "GND" H 855 1727 50  0000 C CNN
F 2 "" H 850 1900 50  0001 C CNN
F 3 "" H 850 1900 50  0001 C CNN
	1    850  1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  1900 850  1600
$Comp
L Device:C C12
U 1 1 5D8517C5
P 4300 2700
F 0 "C12" H 4250 2800 50  0000 L CNN
F 1 "0.1uF" H 4200 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4338 2550 50  0001 C CNN
F 3 "~" H 4300 2700 50  0001 C CNN
	1    4300 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5D851A6B
P 4500 2700
F 0 "C13" H 4450 2800 50  0000 L CNN
F 1 "0.1uF" H 4400 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4538 2550 50  0001 C CNN
F 3 "~" H 4500 2700 50  0001 C CNN
	1    4500 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5D851C7E
P 4700 2700
F 0 "C14" H 4650 2800 50  0000 L CNN
F 1 "0.1uF" H 4600 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4738 2550 50  0001 C CNN
F 3 "~" H 4700 2700 50  0001 C CNN
	1    4700 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 5D85B7E9
P 4900 2700
F 0 "C15" H 4850 2800 50  0000 L CNN
F 1 "0.1uF" H 4800 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4938 2550 50  0001 C CNN
F 3 "~" H 4900 2700 50  0001 C CNN
	1    4900 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5D85BAB3
P 4500 2950
F 0 "#PWR017" H 4500 2700 50  0001 C CNN
F 1 "GND" H 4505 2777 50  0000 C CNN
F 2 "" H 4500 2950 50  0001 C CNN
F 3 "" H 4500 2950 50  0001 C CNN
	1    4500 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 2900 4500 2950
Wire Wire Line
	4300 2850 4300 2900
Wire Wire Line
	4300 2900 4500 2900
Wire Wire Line
	4500 2850 4500 2900
Connection ~ 4500 2900
Wire Wire Line
	4700 2850 4700 2900
Wire Wire Line
	4700 2900 4500 2900
Wire Wire Line
	4900 2850 4900 2900
Wire Wire Line
	4900 2900 4700 2900
Connection ~ 4700 2900
$Comp
L power:+3V3 #PWR016
U 1 1 5D874251
P 4500 2450
F 0 "#PWR016" H 4500 2300 50  0001 C CNN
F 1 "+3V3" H 4515 2623 50  0000 C CNN
F 2 "" H 4500 2450 50  0001 C CNN
F 3 "" H 4500 2450 50  0001 C CNN
	1    4500 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 2450 4500 2500
Wire Wire Line
	4500 2500 4300 2500
Wire Wire Line
	4300 2500 4300 2550
Connection ~ 4500 2500
Wire Wire Line
	4500 2500 4500 2550
Wire Wire Line
	4500 2500 4700 2500
Wire Wire Line
	4700 2500 4700 2550
Wire Wire Line
	4700 2500 4900 2500
Wire Wire Line
	4900 2500 4900 2550
Connection ~ 4700 2500
Wire Wire Line
	4650 6150 5200 6150
Wire Wire Line
	4650 6250 5200 6250
Wire Wire Line
	4650 6350 5200 6350
Wire Wire Line
	4650 6450 5200 6450
Wire Wire Line
	1200 1100 1250 1100
Wire Wire Line
	1550 1100 1650 1100
Wire Wire Line
	2500 1700 2500 1900
Wire Wire Line
	3400 1700 3400 1900
Wire Wire Line
	3000 1400 3000 1900
Wire Wire Line
	5250 3300 5250 3350
Wire Wire Line
	5250 3350 4800 3350
Wire Wire Line
	4800 3350 4800 3850
Wire Wire Line
	4800 3850 4250 3850
Connection ~ 5250 3350
Wire Wire Line
	5250 3350 5250 3400
Wire Wire Line
	2900 5050 2950 5050
$Comp
L Mechanical:MountingHole H1
U 1 1 5D94DF4E
P 6250 1050
F 0 "H1" H 6350 1096 50  0000 L CNN
F 1 "MountingHole" H 6350 1005 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6250 1050 50  0001 C CNN
F 3 "~" H 6250 1050 50  0001 C CNN
	1    6250 1050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5D94E422
P 6250 1250
F 0 "H2" H 6350 1296 50  0000 L CNN
F 1 "MountingHole" H 6350 1205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6250 1250 50  0001 C CNN
F 3 "~" H 6250 1250 50  0001 C CNN
	1    6250 1250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5D94E628
P 6250 1450
F 0 "H3" H 6350 1496 50  0000 L CNN
F 1 "MountingHole" H 6350 1405 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6250 1450 50  0001 C CNN
F 3 "~" H 6250 1450 50  0001 C CNN
	1    6250 1450
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5D94E855
P 6250 1650
F 0 "H4" H 6350 1696 50  0000 L CNN
F 1 "MountingHole" H 6350 1605 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6250 1650 50  0001 C CNN
F 3 "~" H 6250 1650 50  0001 C CNN
	1    6250 1650
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Male J2
U 1 1 5D9683B9
P 6200 2350
F 0 "J2" H 6308 2731 50  0000 C CNN
F 1 "Conn_01x06_Male" H 6308 2640 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 6200 2350 50  0001 C CNN
F 3 "~" H 6200 2350 50  0001 C CNN
	1    6200 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5D968E32
P 6600 2700
F 0 "#PWR0101" H 6600 2450 50  0001 C CNN
F 1 "GND" H 6605 2527 50  0000 C CNN
F 2 "" H 6600 2700 50  0001 C CNN
F 3 "" H 6600 2700 50  0001 C CNN
	1    6600 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2700 6600 2650
Wire Wire Line
	6600 2650 6400 2650
Wire Wire Line
	6400 2550 6600 2550
Wire Wire Line
	6600 2550 6600 2650
Connection ~ 6600 2650
$Comp
L power:+3V3 #PWR0102
U 1 1 5D975180
P 6650 2150
F 0 "#PWR0102" H 6650 2000 50  0001 C CNN
F 1 "+3V3" V 6665 2278 50  0000 L CNN
F 2 "" H 6650 2150 50  0001 C CNN
F 3 "" H 6650 2150 50  0001 C CNN
	1    6650 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	6650 2150 6400 2150
Text Label 5300 6450 0    50   ~ 0
Vled
Wire Wire Line
	5300 6450 5200 6450
Connection ~ 5200 6450
Text Label 6700 2250 0    50   ~ 0
Vled
Wire Wire Line
	6700 2250 6400 2250
Text Label 5350 3350 0    50   ~ 0
Vphoto
Wire Wire Line
	5350 3350 5250 3350
Text Label 6700 2350 0    50   ~ 0
Vphoto
Wire Wire Line
	6700 2350 6400 2350
Text Label 6700 2450 0    50   ~ 0
Vbus
Wire Wire Line
	6700 2450 6400 2450
$EndSCHEMATC
