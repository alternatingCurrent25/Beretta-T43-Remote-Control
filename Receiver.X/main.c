/* ENEL 500
 * 
 * File: main.c 
 * 
 * Authors: 
 */

#include "xc.h"
#include <p24fxxxx.h>
#include <p24F16KA101.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <libpic30.h>

#include "ChangeClk.h"
//// CONFIGURATION BITS ////

// Code protection 
#pragma config BSS = OFF // Boot segment code protect disabled
#pragma config BWRP = OFF // Boot sengment flash write protection off
#pragma config GCP = OFF // general segment code protecion off
#pragma config GWRP = OFF

// CLOCK CONTROL 
#pragma config IESO = OFF    // 2 Speed Startup disabled
#pragma config FNOSC = FRC  // Start up CLK = 8 MHz
#pragma config FCKSM = CSECMD // Clock switching is enabled, clock monitor disabled
#pragma config SOSCSEL = SOSCLP // Secondary oscillator for Low Power Operation
#pragma config POSCFREQ = MS  //Primary Oscillator/External clk freq betwn 100kHz and 8 MHz. Options: LS, MS, HS
#pragma config OSCIOFNC = ON  //CLKO output disabled on pin 8, use as IO. 
#pragma config POSCMOD = NONE  // Primary oscillator mode is disabled

// WDT
#pragma config FWDTEN = OFF // WDT is off
#pragma config WINDIS = OFF // STANDARD WDT/. Applicable if WDT is on
#pragma config FWPSA = PR32 // WDT is selected uses prescaler of 32
#pragma config WDTPS = PS1 // WDT postscler is 1 if WDT selected

// MCLR/RA5 CONTROL
#pragma config MCLRE = OFF // RA5 pin configured as input, MCLR reset on RA5 diabled

// BOR  - FPOR Register
#pragma config BORV = LPBOR // LPBOR value=2V is BOR enabled
#pragma config BOREN = BOR0 // BOR controlled using SBOREN bit
#pragma config PWRTEN = OFF // Powerup timer disabled
#pragma config I2C1SEL = PRI // Default location for SCL1/SDA1 pin

// JTAG FICD Register
#pragma config BKBUG = OFF // Background Debugger functions disabled
#pragma config ICS = PGx2 // PGC2 (pin2) & PGD2 (pin3) are used to connect PICKIT3 debugger

// Deep Sleep RTCC WDT
#pragma config DSWDTEN = OFF // Deep Sleep WDT is disabled
#pragma config DSBOREN = OFF // Deep Sleep BOR is disabled
#pragma config RTCOSC = LPRC// RTCC uses LPRC 32kHz for clock
#pragma config DSWDTOSC = LPRC // DeepSleep WDT uses Lo Power RC clk
#pragma config DSWDTPS = DSWDTPS7 // DSWDT postscaler set to 32768 

// MACROS
#define Nop() {__asm__ volatile ("nop");}
#define ClrWdt() {__asm__ volatile ("clrwdt");}
#define Sleep() {__asm__ volatile ("pwrsav #0");}   // set sleep mode
#define Idle() {__asm__ volatile ("pwrsav #1");}
#define dsen() {__asm__ volatile ("BSET DSCON, #15");}

// Definition for the built-in delay function
#define FCY 4000000UL // 8 MHz/2 Instruction cycle frequency, Hz

#define CHNUP 0xE0E048B7 
#define CHNDWN 0xE0E008F7 
#define VOLUP 0xE0E0E01F 
#define VOLDWN 0xE0E0D02F 
#define PWRSW 0xE0E040BF

//typedef union {
//  struct
//  {
//    uint8_t input_level                : 1;
//    uint8_t start                      : 1;
//    uint8_t zero_start                 : 1;
//    uint8_t datalow                    : 1;
//    uint8_t unused                     : 4;
//  };
//  uint8_t byte;
//} Dataflag;
//
//Dataflag dataflag;

uint16_t time; 
uint8_t state = 0; 
uint8_t i = 31;
uint8_t data_complete = 0;
uint32_t data = 0;

void init_timer(void){  
    T1CONbits.TON = 0;   // Disable Timer1
    T1CONbits.TCS = 0;   // Select internal clock source
    T1CONbits.TCKPS = 0b00; // Set prescaler to 1:8
    T1CONbits.TGATE = 0; // Disable gated time accumulation
    T1CONbits.TSIDL = 0; // Continue operation in idle mode
    T1CONbits.TSYNC = 0; // Do not synchronize external clock input
    
    PR1 = 20000;  // interrupt at 5 ms
    IEC0bits.T1IE = 1;  // enable interrupt
    IPC0bits.T1IP = 0b100;  // set interrupt priority
}
    
uint16_t get_time(void){
    return (TMR1 / 4);
}

void set_timer(uint16_t time){
    TMR1 = time;
}

void set_bit(uint8_t i){
    data = data | ((uint32_t)1 << i);
}

void clear_bit(uint8_t i){
    data = data & ~((uint32_t)1 << i);
}
void process_data(void){
    return;
}

int main(void) {
    
    // Analog ports to digital
    AD1PCFG = 0xFFFF; 

    // IO CN config bits
    TRISAbits.TRISA4 = 1;
    TRISBbits.TRISB2 = 1; 
    
    IEC1bits.CNIE = 1;
    CNEN1bits.CN6IE = 1;  
    CNEN1bits.CN0IE = 1;
    IPC4bits.CNIP = 0b001;
     
    TRISBbits.TRISB4 = 0;

    while(1) {
        if (data_complete == 1){
            process_data();
        }
    }
    return 0;
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void){
    
    if(state != 0){
        time = get_time();                         // Store Timer1 value
        set_timer(0);                               // Reset Timer1
    }
    
    IFS1bits.CNIF = 0;
    
    switch(state){
        
        case 0 :                                     // Start receiving IR data (we're at the beginning of 9ms pulse)
            T1CONbits.TON = 1;   // Enable Timer1 module with internal clock source and prescaler = 2
            set_timer(0);                             // Reset Timer1 value
            state = 1;                            
            i = 0;
            return;
            
        case 1 :                                     // End of 9ms pulse
            if((time > 5000) || (time < 4000)){        // Invalid interval ==> stop decoding and reset
                state = 0;                           // Reset decoding process
                T1CONbits.TON = 0;              // Stop Timer1 module
            }
            else{
                state = 2;  // Next state: end of 4.5ms space (start of 560탎 pulse)
            }
            return;
            
        case 2 :                                     // End of 4.5ms space
            if((time > 5000) || (time < 4000)){        // Invalid interval ==> stop decoding and reset
                state = 0;                           // Reset decoding process
                T1CONbits.TON = 0;              // Stop Timer1 module
                return;
            }
            state = 3;                             // Next state: end of 560탎 pulse (start of 560탎 or 1680탎 space)
            return;
            
        case 3 :                                     // End of 560탎 pulse
            if((time > 700) || (time < 400)){          // Invalid interval ==> stop decoding and reset
                state = 0;                           // Reset decoding process
                T1CONbits.TON = 0;              // Disable Timer1 module
            }
            else{
                state = 4;                           // Next state: end of 560탎 or 1680탎 space
            }
            return;
            
        case 4 :                                     // End of 560탎 or 1680탎 space
            if((time > 1800) || (time < 400)){         // Invalid interval ==> stop decoding and reset
                state = 0;                           // Reset decoding process
                T1CONbits.TON = 0;              // Disable Timer1 module
                return;
            }
        
            if( time > 1000){                         // If space width > 1ms (short space)
                set_bit((31 - i));                // Write 1 to bit (31 - i)
            }
            else{                                     // If space width < 1ms (long space)
                clear_bit((31 - i));           // Write 0 to bit (31 - i)
            }
            
            i++;
            
            if(i > 31){                                // If all bits are received
                data_complete = 1;                             // Decoding process OK
            }
            state = 3;                             // Next state: end of 560탎 pulse (start of 560탎 or 1680탎 space)
  }
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    state = 0;
    T1CONbits.TON = 0;
    IFS0bits.T1IF=0; //Clear timer 1 interrupt flag
}