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
#define NLEFT   0xE0E066B7
#define RIGHT   0xE0E068F7 
#define NRIGHT  0xE0E066F7
#define UP      0xE0E0F01F 
#define NUP     0xE0E0FF1F 
#define DOWN    0xE0E0E62F 
#define NDOWN   0xE0E0E82F 
#define SAFETY  0xE0E080BF

// LCD Pins
#define RS  LATBbits.LATB7
#define RW  LATBbits.LATB8
#define E   LATBbits.LATB9
#define DB4 LATAbits.LATA6
#define DB5 LATBbits.LATB12
#define DB6 LATBbits.LATB13
#define DB7 LATBbits.LATB14

uint8_t state = 0, i = 31, data_complete = 0;
uint16_t time; 
uint32_t data = 0;

uint8_t up_state = 0;
uint8_t down_state = 0;
uint8_t left_state = 0;
uint8_t right_state = 0;

void init_timer2(void){  
    T2CONbits.TON = 0;   // Disable Timer1
    T2CONbits.TCS = 0;   // Select internal clock source
    T2CONbits.TCKPS = 0b10; // Set prescaler to 1:8
    T2CONbits.TGATE = 0; // Disable gated time accumulation
    T2CONbits.TSIDL = 0; // Continue operation in idle mode
    T2CONbits.T32 = 0; 
  
    IEC0bits.T2IE = 1;  // enable interrupt
    IPC1bits.T2IP = 0b100;  // set interrupt priority
}

//delay in us
volatile unsigned int timer2_count = 0; // volatile variable to store the Timer 2 interrupt count

void delay_us(uint16_t time_us)
{
    timer2_count = 0;
    PR2 = time_us * 4;
    T2CONbits.TON = 1;
    while(timer2_count == 0);
}

 // initialize timer 1
void init_timer1(void){  
    T1CONbits.TON = 0;   // Disable Timer1
    T1CONbits.TCS = 0;   // Select internal clock source
    T1CONbits.TCKPS = 0b00; // Set prescaler to 1:8
    T1CONbits.TGATE = 0; // Disable gated time accumulation
    T1CONbits.TSIDL = 0; // Continue operation in idle mode
    T1CONbits.TSYNC = 0; // Do not synchronize external clock input
    
    PR1 = 20000;  // interrupt at 5 ms
    IEC0bits.T1IE = 1;  // enable interrupt
    IPC0bits.T1IP = 0b101;  // set interrupt priority
}

// returns time of timer 1
uint16_t get_time1(void){
    return (TMR1 / 4);
}

// sets timer 1 
void set_timer1(uint16_t time){
    TMR1 = (time*4);
}

// sets bit i of data to 1
void set_bit(uint8_t i){
    data = data | ((uint32_t)1 << i);
}

// sets bit i of data to 0
void clear_bit(uint8_t i){
    data = data & ~((uint32_t)1 << i);
}

// 
void process_data(void){
    
    if (data == SAFETY){
        left_state = 0;
        right_state = 0;
        up_state = 0;
        down_state = 0;
    }else if (data == LEFT){
        left_state = 1;
    }else if (data == NLEFT)
    {
        left_state = 0;
    }else if (data == RIGHT)
    {
        right_state = 1;
    }else if (data == NRIGHT)
    {
        right_state = 1;
    }else if (data == UP)
    {
        up_state = 1;
    }else if (data == NUP)
    {
        up_state = 0;
    }else if (data == DOWN)
    {
        down_state = 1;
    }else if (data == NDOWN)
    {
        down_state = 0;
    }
    
}

// LCD Functions

void Nybble(void){
    E = 1;
    delay_us(1); //enable pulse width >= 300ns
    E = 0; //Clock enable: falling edge 
}

void put_data(uint8_t i){
    DB7 = i >> 7;
    DB6 = (i >> 6) & 1;
    DB5 = (i >> 5) & 1;
    DB4 = (i >> 4) & 1;
}
void command(char i){
    put_data(i); //put data on output Port
    RS =0; //D/I=LOW : send instruction
    RW =0; //R/W=LOW : Write 
    Nybble(); //Send lower 4 bits
    i = i<<4; //Shift over by 4 bits
    put_data(i); //put data on output Port
    Nybble(); //Send upper 4 bits 
}

void write(char i) {
    put_data(i); //put data on output Port
    RS =1; //D/I=HIGH : send data
    RW =0; //R/W=LOW : Write 
    Nybble(); //Clock lower 4 bits
    i = i<<4; //Shift over by 4 bits
    put_data(i); //put data on output Port
    Nybble(); //Clock upper 4 bits 
}

void write_string(char *str){
    for(int i=0; str[i]!='\0'; i++){
        write(str[i]);
    }
}

void clear_LCD(void){
    command(0x00);
    command(0x01);
}

void init_LCD(void){
    
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;
    TRISAbits.TRISA6 = 0;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB14 = 0;
    
    put_data(0); 
    for (int i = 0; i < 10; i++){  //Wait >40 msec after power is applied
        delay_us(1000);
    }
    put_data(0x30); //put 0x30 on the output port
    delay_us(5000); //must wait 5ms, busy flag not available
    Nybble(); //command 0x30 = Wake up 
    delay_us(160); //must wait 160us, busy flag not available
    Nybble(); //command 0x30 = Wake up #2
    delay_us(160); //must wait 160us, busy flag not available
    Nybble(); //command 0x30 = Wake up #3
    delay_us(160); //can check busy flag now instead of delay
    put_data(0x20); //put 0x20 on the output port
    Nybble(); //Function set: 4-bit interface
    command(0x28); //Function set: 4-bit/2 -line
    command(0x10); //Set cursor
    command(0x0F); //Display ON; Blinking cursor
    command(0x06); //Entry Mode set 
}

int main(void) {
    
    // Analog ports to digital
    AD1PCFG = 0xFFFF; 
    
    NewClk(8);
    // IO CN config bits
    TRISAbits.TRISA4 = 1;
    TRISBbits.TRISB2 = 1; 
    
    IEC1bits.CNIE = 1;
    CNEN1bits.CN6IE = 1;  
    CNEN1bits.CN0IE = 1;
    IPC4bits.CNIP = 0b001;
     
    TRISBbits.TRISB4 = 0;
    init_timer1();
    init_timer2();
    init_LCD();
    

    while(1) {
        
        if (data_complete == 1){
            process_data();
            clear_LCD();
            if (up_state){
                write_string("up ");
            }
            if (down_state){
                write_string("down ");
            }
            if (left_state){
                write_string("left ");
            }
            if (right_state){
                write_string("right ");
            }
            data_complete = 0;
        }
    }
    return 0;
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void){
    
    if(state != 0){
        time = get_time1();                         // Store Timer1 value
        set_timer1(0);                              // Reset Timer1
    }
    
    IFS1bits.CNIF = 0;
    
    switch(state){
        
        case 0 :                 // Start receiving IR data
            T1CONbits.TON = 1;   // start Timer1 
            set_timer1(0);        // Reset Timer1 value
            state = 1;           // go to state 1              
            i = 0;
            return;
            
        case 1 :                                     // expected 4.5ms pulse
            if((time > 5000) || (time < 4000)){      // Invalid interval ==> stop decoding and reset
                state = 0;                      // go back to state 0
                T1CONbits.TON = 0;              // Stop Timer1 
            }
            else{
                state = 2;  // first 4.5ms pulse received, go to state 2
            }
            return;
            
        case 2 :                                     // expected 4.5ms space
            if((time > 5000) || (time < 4000)){      // Invalid interval ==> stop decoding and reset
                state = 0;                      // go back to state 0
                T1CONbits.TON = 0;              // Stop Timer1 
                return;
            }
            state = 3;                             // Second 4.5ms space, go to state 3
            return;
            
        case 3 :                                     // expected 560탎 pulse
            if((time > 700) || (time < 400)){        // Invalid interval ==> stop decoding and reset
                state = 0;                      // go back to state 0
                T1CONbits.TON = 0;              // Disable Timer1 
            }
            else{
                state = 4;                       // 560탎 pulse received, go to state 4
            }
            return;
            
        case 4 :                                     // expected 560탎 or 1680탎 space
            if((time > 1800) || (time < 400)){       // Invalid interval ==> stop decoding and reset
                state = 0;                      // go back to state 0
                T1CONbits.TON = 0;              // Disable Timer1 
                return;
            }
        
            if( time > 1000){                     // If space width > 1ms (1680us space)
                set_bit((31 - i));                // Write 1 to bit (31 - i)
            }
            else{                                 // If space width < 1ms (560us space)
                clear_bit((31 - i));              // Write 0 to bit (31 - i)
            }
            
            i++;    // increment i
            
            if(i > 31){            // if all bits are received, mark data as complete
                data_complete = 1;                    
            }
            
            state = 3;             // go to state 3 to receive next bit
            return;
  }
}

// interrupt happens when timer hits 5ms: receiver is idle, reset state to 0 
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    state = 0;
    T1CONbits.TON = 0;
    IFS0bits.T1IF=0; //Clear timer 1 interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    timer2_count++;
    T2CONbits.TON = 0;
    IFS0bits.T2IF = 0; //Clear timer 2 interrupt flag
}