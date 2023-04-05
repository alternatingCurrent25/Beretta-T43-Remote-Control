
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
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (EMUC/EMUD share PGC1/PGD1)

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

// Solenoid PinOut


// LCD Pins
#define RS  LATBbits.LATB7 //B7
#define RW  LATBbits.LATB8 //B8
#define E   LATBbits.LATB9 //B9
#define DB4 LATBbits.LATB12 // WAS RA6 BEFORE //B12
#define DB5 LATBbits.LATB13 //B13
#define DB6 LATBbits.LATB14 //B14
#define DB7 LATBbits.LATB15 //B15

#define LED1 LATBbits.LATB1 
#define LED2 LATBbits.LATB2 
#define LED3 LATAbits.LATA2
#define LED4 LATBbits.LATB0 
#define LED5 LATBbits.LATB4 




uint8_t state = 0, i = 31, data_complete = 0, count = 0;
uint16_t time;
uint32_t data = 0;

uint8_t up_state = 0;
uint8_t down_state = 0;
uint8_t left_state = 0;
uint8_t right_state = 0;

uint8_t previous_up_state = 0;
uint8_t previous_down_state = 0;
uint8_t previous_left_state = 0;
uint8_t previous_right_state = 0;

uint8_t signal_received = 0;
uint8_t timeout_reached = 0;

void init_timer4(void) {
    T4CONbits.TON = 0; // Disable Timer1
    T4CONbits.TCS = 0; // Select internal clock source
    T4CONbits.TCKPS = 0b10; // Set prescaler to 1:8
    T4CONbits.TGATE = 0; // Disable gated time accumulation
    T4CONbits.TSIDL = 0; // Continue operation in idle mode
    T4CONbits.T32 = 0;

    PR4 = 30000; // interrupt at 10 ms
    IEC1bits.T4IE = 1; // enable interrupt
    IPC6bits.T4IP = 0b110; // set interrupt priority
}

void init_timer2(void) {
    T2CONbits.TON = 0; // Disable Timer1
    T2CONbits.TCS = 0; // Select internal clock source
    T2CONbits.TCKPS = 0b00; // Set prescaler to 1:8
    T2CONbits.TGATE = 0; // Disable gated time accumulation
    T2CONbits.TSIDL = 0; // Continue operation in idle mode
    T2CONbits.T32 = 0;

    PR2 = 20000; // interrupt at 5 ms
    IEC0bits.T2IE = 1; // enable interrupt
    IPC1bits.T2IP = 0b001; // set interrupt priority
}

//delay in us

void delay_us(uint16_t time_us) {
    PR2 = time_us * 4;
    T2CONbits.TON = 1;
    Idle();
}

// initialize timer 1

void init_timer1(void) {
    T1CONbits.TON = 0; // Disable Timer1
    T1CONbits.TCS = 0; // Select internal clock source
    T1CONbits.TCKPS = 0b00; // Set prescaler to 1:8
    T1CONbits.TGATE = 0; // Disable gated time accumulation
    T1CONbits.TSIDL = 0; // Continue operation in idle mode
    T1CONbits.TSYNC = 0; // Do not synchronize external clock input

    PR1 = 20000; // interrupt at 5 ms
    IEC0bits.T1IE = 1; // enable interrupt
    IPC0bits.T1IP = 0b011; // set interrupt priority
}

// returns time of timer 1

uint16_t get_time1(void) {
    return (TMR1 / 4);
}

// sets timer 1 

void set_timer1(uint16_t time) {
    TMR1 = (time * 4);
}

// sets bit i of data to 1

void set_bit(uint8_t i) {
    data = data | ((uint32_t) 1 << i);
}

// sets bit i of data to 0

void clear_bit(uint8_t i) {
    data = data & ~((uint32_t) 1 << i);
}

// LCD Functions:

void Nybble(void) {
    E = 1;
    delay_us(1); //enable pulse width >= 300ns
    E = 0; //Clock enable: falling edge 
}

// puts data in LCD pins

void put_data(uint8_t i) {
    DB7 = i >> 7;
    DB6 = (i >> 6) & 1;
    DB5 = (i >> 5) & 1;
    //    DB4 = (i >> 4) & 1;
}

// sends a command to the LCD

void command(char i) {
    put_data(i); //put data on output Port
    RS = 0; //D/I=LOW : send instruction
    RW = 0; //R/W=LOW : Write 
    Nybble(); //Send lower 4 bits
    i = i << 4; //Shift over by 4 bits
    put_data(i); //put data on output Port
    Nybble(); //Send upper 4 bits 
}

// writes a character to the LCD

void write(char i) {
    put_data(i); //put data on output Port
    RS = 1; //D/I=HIGH : send data
    RW = 0; //R/W=LOW : Write 
    Nybble(); //Clock lower 4 bits
    i = i << 4; //Shift over by 4 bits
    put_data(i); //put data on output Port
    Nybble(); //Clock upper 4 bits 
}

// writes a string to the LCD

void write_string(char *str) {
    for (int i = 0; str[i] != '\0'; i++) {
        write(str[i]);
        delay_us(10);
    }
}

// clears LCD

void clear_LCD(void) {
    command(0x00);
    command(0x01);
}

void init_LCD(void) {
    // set inputs
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB15 = 0;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB14 = 0;

    put_data(0);
    for (int i = 0; i < 10; i++) { //Wait >40 msec after power is applied
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

// RF functions

void process_data(void) {

    // count signals received
    count++;

    if (data == SAFETY) {
        left_state = 0;
        right_state = 0;
        up_state = 0;
        down_state = 0;
        
        // Disable receiving
        // IEC1bits.CNIE = 0;
        
        // clear_LCD();
        
        LED5 = 1; // Safety LED ON
        
        // Switch off LEDs/Solenoids
        LED1 = 0;
        LED2 = 0;
        LED3 = 0;
        LED4 = 0;
    }

    if (data == LEFT) {
        left_state = 1; 
    }

    if (data == RIGHT) {
        right_state = 1;
    }

    if (data == UP) {
        up_state = 1;
    }

    if (data == DOWN) {
        down_state = 1;
    }

    // reset data complete flag
    data_complete = 0;

    // only update after 2 signals received
    if (count == 2) {

        // check if signals changed, if changed update LCD
        if (previous_right_state != right_state) {
            LED2 = right_state;
        }
        if (previous_left_state != left_state) {
            LED1 = left_state;
        }
        if (previous_down_state != down_state) {
            LED4 = down_state;
        }
        if (previous_up_state != up_state) {
            LED3 = up_state;
        }
     

        // update previous states
        previous_right_state = right_state;
        previous_left_state  = left_state;
        previous_up_state    = up_state;
        previous_down_state  = down_state;

        // reset current states
        left_state = 0;
        right_state = 0;
        up_state = 0;
        down_state = 0;

        // reset count
        count = 0;
    }
}

// Process signal received

void process_signal(void) {

    signal_received = 0;

    if (state != 0) {
        time = get_time1(); // Store Timer1 value
        set_timer1(0); // Reset Timer1
    }

    switch (state) {

        case 0: // Start receiving IR data
            T1CONbits.TON = 1; // start Timer1 
            set_timer1(0); // Reset Timer1 value
            state = 1; // go to state 1     
            i = 0;
            return;

        case 1: // expected 4.5ms pulse
            if ((time > 5000) || (time < 4000)) { // Invalid interval ==> stop decoding and reset
                state = 0; // go back to state 0
                T1CONbits.TON = 0; // Stop Timer1 
            } else {
                state = 2; // first 4.5ms pulse received, go to state 2
            }
            return;

        case 2: // expected 4.5ms space
            if ((time > 5000) || (time < 4000)) { // Invalid interval ==> stop decoding and reset
                state = 0; // go back to state 0
                T1CONbits.TON = 0; // Stop Timer1 
                return;
            }
            state = 3; // Second 4.5ms space, go to state 3
            return;

        case 3: // expected 560?s pulse
            if ((time > 700) || (time < 400)) { // Invalid interval ==> stop decoding and reset
                state = 0; // go back to state 0
                T1CONbits.TON = 0; // Disable Timer1 
            } else {
                state = 4; // 560?s pulse received, go to state 4
            }
            return;

        case 4: // expected 560?s or 1680?s space
            if ((time > 1800) || (time < 400)) { // Invalid interval ==> stop decoding and reset
                state = 0; // go back to state 0
                T1CONbits.TON = 0; // Disable Timer1 
                return;
            }

            if (time > 1000) { // If space width > 1ms (1680us space)
                set_bit((31 - i)); // Write 1 to bit (31 - i)
            } else { // If space width < 1ms (560us space)
                clear_bit((31 - i)); // Write 0 to bit (31 - i)
            }

            i++; // increment i

            if (i > 31) { // if all bits are received, mark data as complete      
                data_complete = 1;
            }

            state = 3; // go to state 3 to receive next bit
            return;
    }
}

init_pins(void){
    //Solenoid PinOut LED IN ORDER UP-DOWN
    TRISBbits.TRISB1 = 0; // LED 1
    TRISBbits.TRISB2 = 0; // LED2 
    TRISAbits.TRISA2 = 0; // LED 3
    TRISBbits.TRISB0 = 0; // LED 4
    TRISBbits.TRISB4 = 0; // LED 5
    
    // set RA4 as input for the RF receiver, and enable interrupt
    TRISAbits.TRISA4 = 1;
    IEC1bits.CNIE = 1;
    CNEN1bits.CN0IE = 1;
    IPC4bits.CNIP = 0b111;
}

int main(void) {

    // Analog ports to digital
    ANSA = 0;
    ANSB = 0;
    
    // Nesting OFF
    INTCON1bits.NSTDIS = 1;

    // set clock to 8MHz
    NewClk(8);

    // init timers
    init_timer1();
    init_timer2();
    init_timer4();
    
    init_pins();
    
    // init LCD
    //init_LCD();

    while (1) {

        // process signal
        if (signal_received) {
            process_signal();
        }

        if (data_complete) {
            process_data();

            // start timer for timeout, if no data received before timeout then timeout_reached = 1
            TMR4 = 0;
            T4CONbits.TON = 1;
        }

        // no signal received, timeout reached 
        if (timeout_reached) {
            //clear LCD 
//            clear_LCD();

            // Reset States
            up_state = 0;
            down_state = 0;
            left_state = 0;
            right_state = 0;
            
            LED1 = 0;
            LED2 = 0;
            LED3 = 0;
            LED4 = 0;
     
            previous_right_state = 0;
            previous_left_state = 0;
            previous_up_state = 0;
            previous_down_state = 0;

            T4CONbits.TON = 0;
            timeout_reached = 0;
        }
    }
    return 0;
}

// CN interrupt, signal received on transmitter
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void) {
    signal_received = 1;
    IFS1bits.CNIF = 0;
}


// Timer 1 interrupt, reset state to 0
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    state = 0;
    T1CONbits.TON = 0;
    IFS0bits.T1IF = 0; //Clear timer 1 interrupt flag
}

// timer 2 interrupt, used for delay function
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    T2CONbits.TON = 0;
    IFS0bits.T2IF = 0; //Clear timer 2 interrupt flag
}

// Timer 4 interrupt, no signal received in 5000us timeout reached
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
    timeout_reached = 1;
    TMR4 = 0;
    IFS1bits.T4IF = 0; //Clear timer 4 interrupt flag
}
