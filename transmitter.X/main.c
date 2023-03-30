
// PIC24FV32KA301 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = OFF               // Boot Segment Write Protect (Disabled)
#pragma config BSS = OFF                // Boot segment Protect (No boot program flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Write Protect (General segment may be written)
#pragma config GSS0 = OFF               // General Segment Code Protect (No Protection)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Select (Fast RC Oscillator (FRC))
#pragma config SOSCSRC = DIG            // SOSC Source Type (Digital Mode for use with external source)
#pragma config LPRCSEL = HP             // LPRC Oscillator Power and Accuracy (High Power, High Accuracy Mode)
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-speed Start-up disabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = ON            // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config POSCFREQ = MS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency between 100kHz and 8MHz)
#pragma config SOSCSEL = SOSCLP         // SOSC Power Selection Configuration bits (Secondary Oscillator configured for low-power operation)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor Selection (Clock Switching is enabled, Fail-safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPS = PS1              // Watchdog Timer Postscale Select bits (1:1)
#pragma config FWPSA = PR32             // WDT Prescaler bit (WDT prescaler ratio of 1:32)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bits (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected(windowed WDT disabled))

// FPOR
#pragma config BOREN = BOR0             // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware, SBOREN bit disabled)
#pragma config LVRCFG = OFF             // Low Voltage Regulator Configuration bit (Low Voltage regulator is not available)
#pragma config PWRTEN = OFF             // Power-up Timer Enable bit (PWRT disabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Use Default SCL1/SDA1 Pins For I2C1)
#pragma config BORV = LPBOR             // Brown-out Reset Voltage bits (Low-power Brown-Out Reset occurs around 2.0V)
#pragma config MCLRE = ON               // MCLR Pin Enable bit (RA5 input pin disabled,MCLR pin enabled)

// FICD
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (EMUC/EMUD share PGC2/PGD2)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

/* ENEL 500
 * 
 * File: main.c 
 * 
 * Authors: 
 */

#include "xc.h"
#include <p24fxxxx.h>
#include <p24fv32ka301.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <libpic30.h>

#include "ChangeClk.h"

////// CONFIGURATION BITS ////
//
//// Code protection 
////#pragma config BSS = OFF // Boot segment code protect disabled
////#pragma config BWRP = OFF // Boot sengment flash write protection off
////#pragma config GCP = OFF // general segment code protecion off
////#pragma config GWRP = OFF
//
//// CLOCK CONTROL 
//#pragma config IESO = OFF    // 2 Speed Startup disabled
//#pragma config FNOSC = FRC  // Start up CLK = 8 MHz
//#pragma config FCKSM = CSECMD // Clock switching is enabled, clock monitor disabled
//#pragma config SOSCSEL = SOSCLP // Secondary oscillator for Low Power Operation
//#pragma config POSCFREQ = MS  //Primary Oscillator/External clk freq betwn 100kHz and 8 MHz. Options: LS, MS, HS
//#pragma config OSCIOFNC = ON  //CLKO output disabled on pin 8, use as IO. 
//#pragma config POSCMOD = NONE  // Primary oscillator mode is disabled
//
//// WDT
//#pragma config FWDTEN = OFF // WDT is off
//#pragma config WINDIS = OFF // STANDARD WDT/. Applicable if WDT is on
//#pragma config FWPSA = PR32 // WDT is selected uses prescaler of 32
//#pragma config WDTPS = PS1 // WDT postscler is 1 if WDT selected
//
//// MCLR/RA5 CONTROL
////#pragma config MCLRE = OFF // RA5 pin configured as input, MCLR reset on RA5 diabled
//
//// BOR  - FPOR Register
//#pragma config BORV = LPBOR // LPBOR value=2V is BOR enabled
//#pragma config BOREN = BOR0 // BOR controlled using SBOREN bit
//#pragma config PWRTEN = OFF // Powerup timer disabled
//#pragma config I2C1SEL = PRI // Default location for SCL1/SDA1 pin

//// JTAG FICD Register
//#pragma config BKBUG = OFF // Background Debugger functions disabled
//#pragma config ICS = PGx2 // PGC2 (pin2) & PGD2 (pin3) are used to connect PICKIT3 debugger
//
//// Deep Sleep RTCC WDT
//#pragma config DSWDTEN = OFF // Deep Sleep WDT is disabled
//#pragma config DSBOREN = OFF // Deep Sleep BOR is disabled
//#pragma config RTCOSC = LPRC// RTCC uses LPRC 32kHz for clock
//#pragma config DSWDTOSC = LPRC // DeepSleep WDT uses Lo Power RC clk
//#pragma config DSWDTPS = DSWDTPS7 // DSWDT postscaler set to 32768 

// MACROS
#define Nop() {__asm__ volatile ("nop");}
#define ClrWdt() {__asm__ volatile ("clrwdt");}
#define Sleep() {__asm__ volatile ("pwrsav #0");}   // set sleep mode
#define Idle() {__asm__ volatile ("pwrsav #1");}
#define dsen() {__asm__ volatile ("BSET DSCON, #15");}

// Definition for the built-in delay function
#define FCY 4000000UL // 8 MHz/2 Instruction cycle frequency, Hz

// Signals
#define LEFT    0xE0E068B7
#define RIGHT   0xE0E068F7 
#define UP      0xE0E0F01F 
#define DOWN    0xE0E0E62F 
#define SAFETY  0xE0E080BF

// Buttons
#define B1 PORTBbits.RB7
#define B2 PORTBbits.RB8
#define B3 PORTBbits.RB9
#define B4 PORTBbits.RB12
#define B5 PORTBbits.RB13

// init buttons

void init_buttons(void) {
    //    CNEN2bits.CN23IE = 1; 
    //    CNEN2bits.CN22IE = 1; 
    //    CNEN2bits.CN21IE = 1;
    //    CNEN1bits.CN8IE  = 1;
    //    CNEN1bits.CN14IE = 1;

    CNPU2bits.CN23PUE = 1;
    CNPU2bits.CN22PUE = 1;
    CNPU2bits.CN21PUE = 1;
    CNPU1bits.CN13PUE = 1;
    CNPU1bits.CN14PUE = 1;

    //    IEC1bits.CNIE = 1;
    //    IPC4bits.CNIP = 0b111;
    TRISBbits.TRISB7 = 1;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB13 = 1;



}

// init timer 1

void init_timer(void) {
    // Timer 1 config bits   
    T1CONbits.TON = 0; // Disable Timer1
    T1CONbits.TCS = 0; // Select internal clock source
    T1CONbits.TCKPS = 0b00; // Set prescaler to 1:1
    T1CONbits.TGATE = 0; // Disable gated time accumulation
    T1CONbits.TSIDL = 0; // Continue operation in idle mode
    T1CONbits.TSYNC = 0; // Do not synchronize external clock input

    IEC0bits.T1IE = 1; // enable interrupt
    IPC0bits.T1IP = 0b110; // set interrupt priority
}

//delay in us

void delay_us(uint16_t time_us) {
    //NewClk(8);
    PR1 = time_us * 4;
    T1CONbits.TON = 1;
    Idle();
}

//delay in ms (max delay: 260 ms)

void delay_ms(uint16_t time_ms) {
    NewClk(500);
    PR1 = time_ms * 250;
    T1CONbits.TON = 1;
    Idle();
    NewClk(8);
}

//sends start bit

void start_bit(void) {
    LATAbits.LATA4 = 1;
    delay_us(4500);
    LATAbits.LATA4 = 0;
    delay_us(4500);
}

//sends a zero bit

void zero_bit(void) {
    LATAbits.LATA4 = 1;
    delay_us(560);
    LATAbits.LATA4 = 0;
    delay_us(560);
}

//sends a 1 bit

void one_bit(void) {
    LATAbits.LATA4 = 1;
    delay_us(560);
    LATAbits.LATA4 = 0;
    delay_us(1690);
}

// Send a signal on the RF transmitter

void send_signal(uint32_t number) {

    uint8_t i = 0; // Aux variable

    start_bit(); // Start bit

    // Loop for sending 
    for (i = 0; i < 32; i++) {
        if (((number >> (31 - i)) & 1) == 1) {
            one_bit();
        } else {
            zero_bit();
        }
    }

    zero_bit(); // end bit
}

int main(void) {

    // Analog ports to digital
  //    AD1PCFGH = 0xFFFF;
//    ANSELA = 0;
//    ANSELB = 0;
    ANSA = 0;
    ANSB = 0;

    // Nesting OFF
    //   INTCON1bits.NSTDIS = 1;

    NewClk(8);

    init_timer();
    init_buttons();

    // set A4 as input for the RF transmitter 
    TRISAbits.TRISA4 = 0;
    
    // DEBUG LED
   // TRISAbits.TRISA4 = 0;
   // LATAbits.LATA4 = 1; // LED ON/Solenoid ON


    while (1) {


        while (B1 == 0 || B2 == 0 || B3 == 0 || B4 == 0 || B5 == 0) {
            if (B1 == 0) {  //if before
                send_signal(LEFT);
                delay_us(5000);

            }

            if (B2 == 0) {
                send_signal(RIGHT);
                delay_us(5000);
            }

            if (B3 == 0) {
                send_signal(UP);
                delay_us(5000);
            }

            if (B4 == 0) {
                send_signal(DOWN);
                delay_us(5000);
            }

            if (B5 == 0) {
                send_signal(SAFETY);
                delay_us(5000);
            }

            delay_us(5000);
        }

    }
    return 0;
}

// timer1 interrupt function

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; //Clear timer 2 interrupt flag
    T1CONbits.TON = 0;
}


