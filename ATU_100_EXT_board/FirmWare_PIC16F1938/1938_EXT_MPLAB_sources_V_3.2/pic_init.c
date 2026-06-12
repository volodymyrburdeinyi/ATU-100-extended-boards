// pic_init unit for Micro C PRO
// David Fainitski for ATU-100 project
// PIC1938 Microchip

#include "cross_compiler.h"

void pic_init(void)
{
/****************************************************************/
#ifdef MPLAB_COMPILER
  CLRWDT();
  WDTCONbits.WDTPS4 = 0;
  WDTCONbits.WDTPS3 = 1;
  WDTCONbits.WDTPS2 = 0; // 1 sec WDT
  WDTCONbits.WDTPS1 = 1;
  WDTCONbits.WDTPS0 = 0;
  CLRWDT();

  ANSELAbits.ANSELA = 0;
  ANSELAbits.ANSA0 = 1; // analog inputs
  ANSELAbits.ANSA1 = 1;
  ANSELBbits.ANSELB = 0; // all as digital

  CM1CON0bits.C1ON = 0; // Disable comparators
  CM2CON0bits.C2ON = 0;

  OSCCON = 0b01111010; // 16 MHz oscillator, Internal

  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
 /* LATA = 0b01000000;*/  // PORT6 /Tx_req to 1
    LATA = 0b11000000; // A6 and A7 set high
  LATB = 0;
  LATC = 0;
  TRISA = 0b00000011;
  //  If we are using the bypass button for an output for uart
  //  set B1 (auto) to an output
#ifdef UART

#ifdef WA1RCT
  TRISB = 0b00000101;
#else
  TRISB = 0b00000101;   /* RB0=in RB1=out(TX) RB2=in(RX) RB3..RB7=out */
#endif
  UART_OUT_PIN = 1;     /* idle TX line high before any transmission    */

  /* Timer0: internal clock, 1:4 prescaler → 1 µs/tick at 16 MHz CPU.
     Used for UART bit-bang RX timing (104 ticks per bit at 9600 baud).
     TMR0IE is left disabled here; uart.c ISR enables it on start-bit.  */
  OPTION_REGbits.T0CS = 0;    /* internal clock source                  */
  OPTION_REGbits.T0SE = 0;    /* increment on low-to-high (unused)      */
  OPTION_REGbits.PSA  = 0;    /* prescaler assigned to Timer0           */
  OPTION_REGbits.PS2  = 0;    /* PS = 001 = 1:4 prescaler               */
  OPTION_REGbits.PS1  = 0;
  OPTION_REGbits.PS0  = 1;
  TMR0 = 0;
  INTCONbits.TMR0IF = 0;
  INTCONbits.TMR0IE = 0;

  /* IOC on RB2: falling edge detects UART start bit.
     PIC16F1938 uses IOCBN/IOCBP registers for edge selection.          */
  IOCBNbits.IOCBN2 = 1;    /* falling edge on RB2                       */
  IOCBPbits.IOCBP2 = 0;    /* no rising edge on RB2                     */
  INTCONbits.IOCIE = 1;    /* IOC interrupt enable (same as RBIE)       */

#else
  TRISB = 0b00000111;
#endif
  TRISC = 0b00000000; //
  //
  ADC_Init();

  ADCON1bits.ADPREF0 = 1; // ADC with the internal reference
  ADCON1bits.ADPREF1 = 1;

  /* init the ADC here */
  FVRCON = 0x81;        /*enable FVR */
//  ADCON1bits.ADCS0 = 0; /* use Fosc/32 */
//  ADCON1bits.ADCS1 = 1;
//  ADCON1bits.ADCS2 = 0;
//
  ADCON1 = 0xA3;  /* use Fosc/32 */

  ADCON0bits.ADON = 1;
  //
  OPTION_REGbits.nWPUEN = 0;
  WPUBbits.WPUB0 = 1; // PORTB0 Pull-up resistor
  WPUBbits.WPUB1 = 1; // PORTB1 Pull-up resistor
  WPUBbits.WPUB2 = 1; // PORTB2 Pull-up resistor
  //interrupt setting
#ifdef UART
  INTCONbits.GIE = 1;
#else
  INTCONbits.GIE = 0;
#endif
/****************************************************************/
#else
  CLRWDT();
  WDTCON.B5 = 0;
  WDTCON.B4 = 1;
  WDTCON.B3 = 0; // 1 sec WDT
  WDTCON.B2 = 1;
  WDTCON.B1 = 0;
  CLRWDT();

  ANSELA = 0;
  ANSA0_bit = 1; // analog inputs
  ANSA1_bit = 1;
  ANSELB = 0; // all as digital

  C1ON_bit = 0; // Disable comparators
  C2ON_bit = 0;

  OSCCON = 0b01111000; // 16 MHz oscillator

  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  LATA = 0b01000000; // PORT6 /Tx_req to 1
  LATB = 0;
  LATC = 0;
  TRISA = 0b00000011;
  TRISB = 0b00000111;
  TRISC = 0b00000000; //
  //
  ADC_Init();
  //

  ADCON1.B0 = 1; // ADC with the internal reference
  ADCON1.B1 = 1;
  //
  OPTION_REG.B7 = 0;
  WPUB.B0 = 1; // PORTB0 Pull-up resistor
  WPUB.B1 = 1; // PORTB1 Pull-up resistor
  WPUB.B2 = 1; // PORTB2 Pull-up resistor
  //interrupt setting
  GIE_bit = 0;
#endif
}
