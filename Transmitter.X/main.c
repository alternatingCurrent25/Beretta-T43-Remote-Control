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
#define B4 PORTAbits.RA6
#define B5 PORTBbits.RB12

// init buttons
void init_buttons(void){
//    CNEN2bits.CN23IE = 1; 
//    CNEN2bits.CN22IE = 1; 
//    CNEN2bits.CN21IE = 1;
//    CNEN1bits.CN8IE  = 1;
//    CNEN1bits.CN14IE = 1;
    
    CNPU2bits.CN23PUE = 1;
    CNPU2bits.CN22PUE = 1;
    CNPU2bits.CN21PUE = 1;
    CNPU1bits.CN8PUE = 1;
    CNPU1bits.CN14PUE = 1;
    
//    IEC1bits.CNIE = 1;
//    IPC4bits.CNIP = 0b111;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB4 = 1;
    
 
}

// init timer 1
void init_timer(void){
    // Timer 1 config bits   
    T1CONbits.TON = 0;   // Disable Timer1
    T1CONbits.TCS = 0;   // Select internal clock source
    T1CONbits.TCKPS = 0b00; // Set prescaler to 1:1
    T1CONbits.TGATE = 0; // Disable gated time accumulation
    T1CONbits.TSIDL = 0; // Continue operation in idle mode
    T1CONbits.TSYNC = 0; // Do not synchronize external clock input
    
    IEC0bits.T1IE = 1;  // enable interrupt
    IPC0bits.T1IP = 0b110;  // set interrupt priority
}

//delay in us
void delay_us(uint16_t time_us)
{
    //NewClk(8);
    PR1 = time_us * 4;
    T1CONbits.TON = 1;
    Idle();
}

//delay in ms (max delay: 260 ms)
void delay_ms(uint16_t time_ms){
    NewClk(500);
    PR1 = time_ms * 250;
    T1CONbits.TON = 1;
    Idle();
    NewClk(8);
}

//sends start bit
void start_bit(void){
    LATBbits.LATB4 = 1; 
    delay_us(4500);
    LATBbits.LATB4 = 0;  
    delay_us(4500);
}

//sends a zero bit
void zero_bit(void){
    LATBbits.LATB4 = 1;  
    delay_us(560);
    LATBbits.LATB4 = 0;  
    delay_us(560);
}

//sends a 1 bit
void one_bit(void){
    LATBbits.LATB4 = 1;  
    delay_us(560);
    LATBbits.LATB4 = 0;  
    delay_us(1690);
}

// Send a signal on the RF transmitter
void send_signal(uint32_t number){
   
    uint8_t i = 0;   // Aux variable
    
    start_bit(); // Start bit
    
    // Loop for sending 
    for(i = 0; i < 32; i++){    
        if(((number>>(31-i)) & 1) == 1){
            one_bit();
        }
        else{
            zero_bit();
        }
    }   
    
    zero_bit(); // end bit
}

int main(void) {
    
    // Analog ports to digital
    AD1PCFG = 0xFFFF; 
    
    // Nesting OFF
    INTCON1bits.NSTDIS = 1;
    
    NewClk(8);
      
    init_timer();
    init_buttons();
    
    // set B4 as input for the RF transmitter 
    TRISBbits.TRISB4 = 0;
     
    while(1) {
        
        while(B1 == 0 || B2 == 0 || B3 == 0 || B4 == 0 || B5 == 0)
        {
            if (B1 == 0){
                send_signal(LEFT);
                delay_us(5000);
            }
            
            if (B2 == 0){
                send_signal(RIGHT);
                delay_us(5000);
            }
            
            if (B3 == 0){
                send_signal(UP);
                delay_us(5000);
            }
            
            if (B4 == 0){
                send_signal(DOWN);
                delay_us(5000);
            }
            
            if (B5 == 0){
                send_signal(SAFETY);
                delay_us(5000);
            }
            
            delay_us(5000);
        }
        
    }
    return 0;
}

// timer1 interrupt function
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF=0; //Clear timer 2 interrupt flag
    T1CONbits.TON=0;
}

    
  