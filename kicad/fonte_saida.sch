EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 5
Title "Fonte Saída"
Date "2021-11-10"
Rev "0.1"
Comp "Julio Cesar Kochhann"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R R2
U 1 1 6175F64A
P 1850 3550
F 0 "R2" V 1643 3550 50  0000 C CNN
F 1 "100k" V 1734 3550 50  0000 C CNN
F 2 "" V 1780 3550 50  0001 C CNN
F 3 "~" H 1850 3550 50  0001 C CNN
	1    1850 3550
	0    1    1    0   
$EndComp
$Comp
L Device:CP C4
U 1 1 6175F85C
P 2350 4450
F 0 "C4" H 2468 4496 50  0000 L CNN
F 1 "1u" H 2468 4405 50  0000 L CNN
F 2 "" H 2388 4300 50  0001 C CNN
F 3 "~" H 2350 4450 50  0001 C CNN
	1    2350 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 61761C21
P 2350 4800
F 0 "#PWR09" H 2350 4550 50  0001 C CNN
F 1 "GND" H 2355 4627 50  0000 C CNN
F 2 "" H 2350 4800 50  0001 C CNN
F 3 "" H 2350 4800 50  0001 C CNN
	1    2350 4800
	1    0    0    -1  
$EndComp
Text GLabel 1350 3550 0    50   Input ~ 0
PWM
Wire Wire Line
	1350 3550 1700 3550
$Comp
L Amplifier_Operational:LM358 U2
U 3 1 6176D57F
P 3850 3650
F 0 "U2" H 3808 3696 50  0001 L CNN
F 1 "LM358" H 3808 3605 50  0001 L CNN
F 2 "" H 3850 3650 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 3850 3650 50  0001 C CNN
	3    3850 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 61789F9B
P 3050 4450
F 0 "R3" H 3120 4496 50  0000 L CNN
F 1 "1k" H 3120 4405 50  0000 L CNN
F 2 "" V 2980 4450 50  0001 C CNN
F 3 "~" H 3050 4450 50  0001 C CNN
	1    3050 4450
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM358 U2
U 1 1 6176C95A
P 3850 3650
F 0 "U2" H 3900 3800 50  0000 C CNN
F 1 "LM358" H 3950 3500 50  0000 C CNN
F 2 "" H 3850 3650 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 3850 3650 50  0001 C CNN
	1    3850 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 617AEC0E
P 3050 4800
F 0 "#PWR010" H 3050 4550 50  0001 C CNN
F 1 "GND" H 3055 4627 50  0000 C CNN
F 2 "" H 3050 4800 50  0001 C CNN
F 3 "" H 3050 4800 50  0001 C CNN
	1    3050 4800
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR011
U 1 1 617AF830
P 3750 2700
F 0 "#PWR011" H 3750 2550 50  0001 C CNN
F 1 "+24V" H 3765 2873 50  0000 C CNN
F 2 "" H 3750 2700 50  0001 C CNN
F 3 "" H 3750 2700 50  0001 C CNN
	1    3750 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 617B017D
P 3750 4800
F 0 "#PWR012" H 3750 4550 50  0001 C CNN
F 1 "GND" H 3755 4627 50  0000 C CNN
F 2 "" H 3750 4800 50  0001 C CNN
F 3 "" H 3750 4800 50  0001 C CNN
	1    3750 4800
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM358 U2
U 2 1 617B2051
P 5900 3750
F 0 "U2" H 5950 3900 50  0000 C CNN
F 1 "LM358" H 6000 3600 50  0000 C CNN
F 2 "" H 5900 3750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 5900 3750 50  0001 C CNN
	2    5900 3750
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM358 U3
U 3 1 617B96A2
P 5900 3750
F 0 "U3" H 5858 3796 50  0001 L CNN
F 1 "LM358" H 5858 3705 50  0001 L CNN
F 2 "" H 5900 3750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 5900 3750 50  0001 C CNN
	3    5900 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 617BE847
P 5100 4450
F 0 "R4" H 5170 4496 50  0000 L CNN
F 1 "6k8" H 5170 4405 50  0000 L CNN
F 2 "" V 5030 4450 50  0001 C CNN
F 3 "~" H 5100 4450 50  0001 C CNN
	1    5100 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4600 2350 4800
Wire Wire Line
	3050 4600 3050 4800
$Comp
L power:GND #PWR013
U 1 1 617CCFA3
P 5100 4800
F 0 "#PWR013" H 5100 4550 50  0001 C CNN
F 1 "GND" H 5105 4627 50  0000 C CNN
F 2 "" H 5100 4800 50  0001 C CNN
F 3 "" H 5100 4800 50  0001 C CNN
	1    5100 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4800 5100 4600
$Comp
L power:+24V #PWR014
U 1 1 617CE59F
P 5800 2700
F 0 "#PWR014" H 5800 2550 50  0001 C CNN
F 1 "+24V" H 5815 2873 50  0000 C CNN
F 2 "" H 5800 2700 50  0001 C CNN
F 3 "" H 5800 2700 50  0001 C CNN
	1    5800 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 617CF6CF
P 5800 4800
F 0 "#PWR015" H 5800 4550 50  0001 C CNN
F 1 "GND" H 5805 4627 50  0000 C CNN
F 2 "" H 5800 4800 50  0001 C CNN
F 3 "" H 5800 4800 50  0001 C CNN
	1    5800 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3550 2350 3550
Wire Wire Line
	5800 4050 5800 4800
Wire Wire Line
	2350 4300 2350 3550
Wire Wire Line
	3750 4800 3750 3950
Wire Wire Line
	5100 3850 5600 3850
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 618A9854
P 10000 2950
F 0 "J2" H 10080 2942 50  0000 L CNN
F 1 "OUTPUT" H 10080 2851 50  0000 L CNN
F 2 "" H 10000 2950 50  0001 C CNN
F 3 "~" H 10000 2950 50  0001 C CNN
	1    10000 2950
	1    0    0    -1  
$EndComp
Text GLabel 9550 3050 0    50   Input ~ 0
OUT-
$Comp
L Device:R_POT_TRIM RV1
U 1 1 6172ACB7
P 3050 3150
F 0 "RV1" V 2843 3150 50  0000 C CNN
F 1 "10k_TRIM" V 2934 3150 50  0000 C CNN
F 2 "" H 3050 3150 50  0001 C CNN
F 3 "~" H 3050 3150 50  0001 C CNN
	1    3050 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	9800 3050 9550 3050
Text GLabel 9300 2850 2    50   Input ~ 0
OUT+
Wire Wire Line
	3050 3300 3050 3750
Wire Wire Line
	2350 3550 3550 3550
Connection ~ 2350 3550
Wire Wire Line
	5100 3850 5100 4150
Wire Wire Line
	9100 2950 9100 4150
Wire Wire Line
	8000 4800 8000 4650
Wire Wire Line
	6900 4650 6900 4800
Wire Wire Line
	9100 2950 8800 2950
Wire Wire Line
	9100 4650 9100 4800
$Comp
L power:+34V #PWR016
U 1 1 61727A59
P 6850 2700
F 0 "#PWR016" H 6850 2550 50  0001 C CNN
F 1 "+34V" H 6865 2873 50  0000 C CNN
F 2 "" H 6850 2700 50  0001 C CNN
F 3 "" H 6850 2700 50  0001 C CNN
	1    6850 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 61808F09
P 9100 4800
F 0 "#PWR019" H 9100 4550 50  0001 C CNN
F 1 "GND" H 9105 4627 50  0000 C CNN
F 2 "" H 9100 4800 50  0001 C CNN
F 3 "" H 9100 4800 50  0001 C CNN
	1    9100 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 61808A22
P 8000 4800
F 0 "#PWR018" H 8000 4550 50  0001 C CNN
F 1 "GND" H 8005 4627 50  0000 C CNN
F 2 "" H 8000 4800 50  0001 C CNN
F 3 "" H 8000 4800 50  0001 C CNN
	1    8000 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 61808464
P 6900 4800
F 0 "#PWR017" H 6900 4550 50  0001 C CNN
F 1 "GND" H 6905 4627 50  0000 C CNN
F 2 "" H 6900 4800 50  0001 C CNN
F 3 "" H 6900 4800 50  0001 C CNN
	1    6900 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 617EF03A
P 9100 4500
F 0 "C7" H 9215 4546 50  0000 L CNN
F 1 "220n" H 9215 4455 50  0000 L CNN
F 2 "" H 9138 4350 50  0001 C CNN
F 3 "~" H 9100 4500 50  0001 C CNN
	1    9100 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C6
U 1 1 617EE629
P 8000 4500
F 0 "C6" H 8118 4546 50  0000 L CNN
F 1 "4u7" H 8118 4455 50  0000 L CNN
F 2 "" H 8038 4350 50  0001 C CNN
F 3 "~" H 8000 4500 50  0001 C CNN
	1    8000 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C5
U 1 1 617EDA7F
P 6900 4500
F 0 "C5" H 7018 4546 50  0000 L CNN
F 1 "100u" H 7018 4455 50  0000 L CNN
F 2 "" H 6938 4350 50  0001 C CNN
F 3 "~" H 6900 4500 50  0001 C CNN
	1    6900 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 617ECC65
P 8650 2950
F 0 "R5" V 8443 2950 50  0000 C CNN
F 1 "0R22/5W" V 8534 2950 50  0000 C CNN
F 2 "" V 8580 2950 50  0001 C CNN
F 3 "~" H 8650 2950 50  0001 C CNN
	1    8650 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 3250 8000 3350
Wire Wire Line
	7450 3650 7450 3750
$Comp
L Transistor_BJT:TIP41C Q2
U 1 1 617D25AB
P 8000 3050
F 0 "Q2" V 8328 3050 50  0000 C CNN
F 1 "TIP41C" V 8237 3050 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 8250 2975 50  0001 L CIN
F 3 "https://www.centralsemi.com/get_document.php?cmp=1&mergetype=pd&mergepath=pd&pdf_id=tip41.PDF" H 8000 3050 50  0001 L CNN
	1    8000 3050
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:BC337 Q1
U 1 1 617D10FC
P 7450 3450
F 0 "Q1" V 7778 3450 50  0000 C CNN
F 1 "BC337" V 7687 3450 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7650 3375 50  0001 L CIN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/bc337.pdf" H 7450 3450 50  0001 L CNN
	1    7450 3450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7250 3350 6850 3350
Wire Wire Line
	7650 3350 8000 3350
Wire Wire Line
	8200 2950 8500 2950
Wire Wire Line
	7800 2950 6850 2950
Wire Wire Line
	6850 2950 6850 3350
Wire Wire Line
	7450 3750 6200 3750
Wire Wire Line
	5100 4150 6900 4150
Connection ~ 5100 4150
Wire Wire Line
	5100 4150 5100 4300
Connection ~ 9100 4150
Wire Wire Line
	9100 4150 9100 4350
Wire Wire Line
	8000 4350 8000 4150
Connection ~ 8000 4150
Wire Wire Line
	8000 4150 9100 4150
Wire Wire Line
	6900 4350 6900 4150
Connection ~ 6900 4150
Wire Wire Line
	6900 4150 8000 4150
Wire Wire Line
	6850 2700 6850 2950
Connection ~ 6850 2950
Wire Wire Line
	5800 3450 5800 2700
Wire Wire Line
	3750 2700 3750 3350
Wire Wire Line
	3550 3750 3050 3750
Connection ~ 3050 3750
Wire Wire Line
	3050 3750 3050 4300
Wire Wire Line
	3200 3150 4400 3150
Wire Wire Line
	4400 3150 4400 3650
Wire Wire Line
	4400 3650 4150 3650
Wire Wire Line
	9100 2950 9800 2950
Connection ~ 9100 2950
Wire Wire Line
	9300 2850 9100 2850
Wire Wire Line
	9100 2850 9100 2950
Wire Wire Line
	5600 3650 4400 3650
Connection ~ 4400 3650
$EndSCHEMATC
