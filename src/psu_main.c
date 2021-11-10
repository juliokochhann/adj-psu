/*
    Adjustable Power Supply

    MIT License

    Copyright (c) 2021 Julio Cesar Kochhann

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE. 
*/
#include <ctype.h>
#include <string.h>
#include "ffpic.h"  // FFPIC include file
#include "instruments.h"
#include "tm1637.h"

#define _ADC_READS          10
#define _ADC_REF            2.048   // ADC Vref+ voltage
#define _ADC_RES            1024.0  // ADC resolution
#define BUF_SIZE            10
#define POWER_THRESHOLD     10.0

#define KPD_TRIS            &TRISB
#define KPD_PORT            &PORTB
#define TM1_TRIS            &TRISC
#define TM1_PORT            &LATC
#define TM1_SCL_PIN         3
#define TM1_SDA_PIN         4
#define TM2_TRIS            &TRISC
#define TM2_PORT            &LATC
#define TM2_SCL_PIN         5
#define TM2_SDA_PIN         6
#define BUZZER              LATCbits.LATC7

/* Global Variables */
TM1637_t Display1, Display2;

Keypad_t keypad = {KPD_TRIS, KPD_PORT};

Queue_t keystrokes;

uint16_t pwm_period;

void adjust_volt(void);
void beep(uint8_t n);
char getkey(void);
void measure(void);
void menu(void);
bool scan(char* buf);
void print(TM1637_t* tm, char* cp);
uint16_t ADC_Read(uint8_t m);
void PWM_Init(void);
void PWM_Set_Duty(uint16_t t);

/* Interrupt service routine (ISR) */
void ISR(void)
{
    /* TODO Check interrupt flag bits to determine the source of an interrupt */

    if (INTCONbits.IOCIF) {
        pause(50);      // Debounce

        Queue_Insert(&keystrokes, Keypad_Read(&keypad));
        
        IOCBF = 0;
        INTCONbits.IOCIF = 0;
    }
}

/* Setup routine: runs once at startup */
void setup(void)
{
    TRISA = 0xFF;
    ANSELA = 0x03;  // RA0-RA1 pins assigned as analog inputs
    LATA = 0;

    ANSELB = 0;   // Disable analog inputs PORTB
    TRISB = 0x0F;
    LATB = 0;
    PORTB = 0;

    TRISC = 0x1F;
    LATC = 0;

    // Enable PORTB pull-ups and interrupt-on-change
    WPUB = 0xFF;
    OPTION_REGbits.nWPUEN = 0;
    IOCBN = 0x0F;               // Detect falling edge on PORTB pins
    INTCONbits.IOCIE = 1;
    INTCONbits.IOCIF = 0;

    // Configure Fixed Voltage Reference module
    FVRCON |= 0x82; // Enable FVR: 2.048V
    while (!FVRCONbits.FVRRDY); // Wait for the reference and amplifier circuits to stabilize

    // Configure ADC module
    ADCON0 = 0x01;  // Enable ADC and select AN0 channel
    ADCON1 = 0x93;  // AD result right justified; ADC clock = FOSC_PIN/8;
                    // _ADC_REF+ = FVR _ADC_REF- +VMAXS

    beep(1);        // Boot beep

    Queue_Init(&keystrokes);

    PWM_Init();     // Init CCP module

    TM_Init(&Display1, TM1_TRIS, TM1_PORT, TM1_SCL_PIN, TM1_SDA_PIN);

    TM_Init(&Display2, TM2_TRIS, TM2_PORT, TM2_SCL_PIN, TM2_SDA_PIN);
}

/* Infinite loop routine: runs repeatedly */
void loop(void)
{
    int status;

    menu();

    measure();

    print(&Display1, ftoa(voltmeter.val_read));
    print(&Display2, ftoa(ammeter.val_read));
}

void adjust_volt(void)
{
    PWM_Set_Duty(0);

    char str[BUF_SIZE];

    bool cancel = scan(str);

    float value = strtof(str);

    if (cancel || value < 0 || value > voltmeter.range) {
        beep(2);

        return;
    }

    voltmeter.val_set = value;
    pwm_period = (uint16_t)(value * 64);
}

void beep(uint8_t n)
{
    while (n--) {
        BUZZER = HIGH;
        pause(15);
        BUZZER = LOW;
        pause(15);
    }
}

char getkey(void)
{
    char key;

    // Disable interrupts to prevent concurrent access to shared resources
    di();

    // Get a key from the queue (If empty, 0 is returned)
    key = Queue_Retrieve(&keystrokes);

    ei();       // Re-enable interrupts

    pause(100);

    return key;
}

void measure(void)
{
    voltmeter.val_read = (voltmeter.range / _ADC_RES) * ADC_Read(VOLTMETER);  // Measure output voltage

    ammeter.val_read = (ammeter.range / _ADC_RES) * ADC_Read(AMMETER);  // Measure load current

    //wattmeter.val_read = voltmeter.val_read * ammeter.val_read;  // Calculate power
    //thermometer.val_read = (thermometer.range / _ADC_RES) * ADC_Read(THERMOMETER);
}

void menu(void)
{
    switch ( getkey() ) {
        case 'A':
            adjust_volt();
            break;
        case 'B':
            break;
        case 'C':
            beep(1);
            PWM_Set_Duty(pwm_period);
            break;
        case 'D':
            beep(1);
            PWM_Set_Duty(0);
            break;
    }
}

bool scan(char* buf)
{
    char key;
    char* cp = buf;

    buf[0] = '0';
    buf[1] = 0;

    TM_Clear(&Display1);
    TM_Clear(&Display2);

    do {
        print(&Display1, buf);
        
        key = getkey();

        if ((isdigit(key) || key == '.') && (strlen(buf) < BUF_SIZE-1)) {
            if (key == '.') {
                if (strchr(buf, '.') != NULL) continue;
                
                if (!(cp - buf)) ++cp;
            }

            *cp++ = key;
            *cp = 0;
        }else if (key == 'C') {
            buf[0] = '0';
            buf[1] = 0;
            cp = buf;

            TM_Clear(&Display1);
        }
    }while (key != 'A' && key != 'D');

    return key == 'A';
}

void print(TM1637_t* tm, char* cp)
{
    TM_Set_Cursor(tm, 0);
    TM_Print_Str(tm, cp);
}

uint16_t ADC_Read(uint8_t m)
{
    uint16_t read_avg = 0;

    ADCON0 = 1;                 // Clear channel selection

    ADCON0 |= m << 2;           // Select ADC channel 

    pause(1);       // Wait acquisition time (full charge of holding capacitor)

    // Perform n analog reads
    for (uint8_t i = 0; i < _ADC_READS; i++) {
        ADCON0bits.GO = 1;      // Start conversion

        while (ADCON0bits.GO);  // Wait for conversion to complete

        read_avg += ((uint16_t)ADRESH << 8) | ADRESL;
    }

    return read_avg / _ADC_READS;
}

void PWM_Init(void)
{
    pwm_period = 0;             // 0% duty cycle

    // Configure Timer 2 and CCP modules
    PR2 = 0xFF;                 // Period register of timer 2
    CCP1CON = 0x0C;             // CCP mode: PWM; Standard PWM mode

    // PWM period = [(PR2)+1]*4*(1/FOSC)*(TMR2 Prescale Value) = 256us
    PWM_Set_Duty(0);

    CCPTMRS0 = 0x00;            // CCP1 is based off Timer 2 in PWM Mode
    PIR1bits.TMR2IF = 0;        // Clear timer 2 interrupt flag
    T2CON = 0x04;               // Turn ON timer 2; Prescaler 1:1

    // Wait until the Timer 2 overflows in order to send a complete duty cycle
    // and period on the first PWM output
    while (!PIR1bits.TMR2IF);

    TRISCbits.TRISC2 = 0;       // Enable the CCP1 pin output driver
}

void PWM_Set_Duty(uint16_t t)
{
    CCP1CON &= 0xCF;
    CCP1CON |= (t & 0x03) << 4;
    CCPR1L = (uint8_t)(t >> 2);
}