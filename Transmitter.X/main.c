/* ENEL 500
 * 
 * File: main.c 
 * 
 * Authors: 
 */

#include <xc.h>
#include <htc.h>
#include <string.h>

#define _XTAL_FREQ 8000000  // Set oscillator frequency to 8 MHz


char data[5];

// Function to send data through RF transmitter
void sendData(char* data) {
    int i;
    for (i = 0; i < strlen(data); i++) {
        TXREG = data[i];  // Load data into transmitter register
        while(!TXIF);  // Wait until transmission is complete
        TXIF = 0;  // Clear transmission complete flag
    }
}

// Interrupt function
void __interrupt() cn_interrupt(void) {
    if(RD6 == 0) {  // Check if button is pressed
        // Code to execute when button is pressed
        sendData(data);
    }
    else {
        // Code to execute when button is released
    }
    
    INTF = 0;  // Clear interrupt flag
}

int main(void) {
    
    // setting up buttons
    TRISDbits.TRISD6 = 1;  // Set PORTD.6 as input
    INTEDG = 0;  // Interrupt on falling edge
    INTE = 1;  // Enable external interrupt
    GIE = 1;
    
    // setting up RF
    TRISCbits.TRISC6 = 0;  // Set PORTC.6 as output for RF transmitter
    SPEN = 1;  // Enable serial port
    SYNC = 0;  // Asynchronous mode
    BRGH = 1;  // High baud rate
    SPBRG = 25;  // Set baud rate to 9600
    CREN = 1;  // Enable receiver
    TXEN = 1;  // Enable transmitter
    
    for (int i = 0; i < 5; i++){
        data[i] = "hello"[i];
    }
    while(1)
    {
        
    }
    return 0;
}


