/* xc.h — host stub for PIC16F1938 special function registers.
 *
 * Purpose: let the firmware translation units be compiled and LINKED with a
 * host compiler (gcc/clang) so that syntax errors, type errors and — most
 * importantly — undefined symbols across translation units are caught without
 * an MPLAB X / XC8 installation.
 *
 * This is a build check only. The resulting binary is never meant to do
 * anything: SFRs are plain variables, delays are no-ops, and the EEPROM is a
 * RAM array. Peripheral behaviour is deliberately NOT emulated — for algorithm
 * behaviour see tools/atusim.c.
 *
 * Registers are declared here and defined once in host_stubs.c.
 */
#ifndef HOSTBUILD_XC_H
#define HOSTBUILD_XC_H

/* ── XC8 language extensions ── */
#define __interrupt()            /* no-op: "void __interrupt() isr(void)" */
#define __eeprom                 /* no-op: place in normal RAM            */
#define CLRWDT()      ((void)0)
#define _delay(cycles) ((void)(cycles))

/* ── EEPROM access (XC8 builtins) — backed by a RAM array on the host ── */
unsigned char eeprom_read(unsigned char addr);
void          eeprom_write(unsigned char addr, unsigned char value);

/* ── plain byte registers ── */
extern volatile unsigned char PORTA, PORTB, PORTC;
extern volatile unsigned char LATA, LATB, LATC;
extern volatile unsigned char TRISA, TRISB, TRISC;
extern volatile unsigned char ANSELA, ANSELB;
extern volatile unsigned char OSCCON, OPTION_REG, WDTCON;
extern volatile unsigned char TMR0, ADCON0, ADCON1, ADRESL, ADRESH, FVRCON;
extern volatile unsigned char TXREG;

/* ── bit-addressable registers ── */
typedef union { unsigned char byte; struct {
    unsigned RA0:1, RA1:1, RA2:1, RA3:1, RA4:1, RA5:1, RA6:1, RA7:1;
}; } porta_t;
typedef union { unsigned char byte; struct {
    unsigned RB0:1, RB1:1, RB2:1, RB3:1, RB4:1, RB5:1, RB6:1, RB7:1;
}; } portb_t;
typedef union { unsigned char byte; struct {
    unsigned LATA0:1, LATA1:1, LATA2:1, LATA3:1, LATA4:1, LATA5:1, LATA6:1, LATA7:1;
}; } lata_t;
typedef union { unsigned char byte; struct {
    unsigned LATB0:1, LATB1:1, LATB2:1, LATB3:1, LATB4:1, LATB5:1, LATB6:1, LATB7:1;
}; } latb_t;
typedef union { unsigned char byte; struct {
    unsigned LATC0:1, LATC1:1, LATC2:1, LATC3:1, LATC4:1, LATC5:1, LATC6:1, LATC7:1;
}; } latc_t;
typedef union { unsigned char byte; struct {
    unsigned TRISA0:1, TRISA1:1, TRISA2:1, TRISA3:1, TRISA4:1, TRISA5:1, TRISA6:1, TRISA7:1;
}; } trisa_t;
typedef union { unsigned char byte; struct {
    unsigned TRISB0:1, TRISB1:1, TRISB2:1, TRISB3:1, TRISB4:1, TRISB5:1, TRISB6:1, TRISB7:1;
}; } trisb_t;
typedef union { unsigned char byte; struct {
    unsigned ANSA0:1, ANSA1:1, ANSA2:1, ANSA3:1, ANSA4:1, ANSA5:1, pad:2;
}; unsigned char ANSELA; } ansela_t;
typedef union { unsigned char byte; unsigned char ANSELB; } anselb_t;
typedef union { unsigned char byte; struct {
    unsigned IOCIF:1, INTF:1, TMR0IF:1, IOCIE:1, INTE:1, TMR0IE:1, PEIE:1, GIE:1;
}; } intcon_t;
typedef union { unsigned char byte; struct {
    unsigned IOCBF0:1, IOCBF1:1, IOCBF2:1, IOCBF3:1,
             IOCBF4:1, IOCBF5:1, IOCBF6:1, IOCBF7:1;
}; } iocbf_t;
typedef union { unsigned char byte; struct {
    unsigned IOCBN0:1, IOCBN1:1, IOCBN2:1, IOCBN3:1,
             IOCBN4:1, IOCBN5:1, IOCBN6:1, IOCBN7:1;
}; } iocbn_t;
typedef union { unsigned char byte; struct {
    unsigned IOCBP0:1, IOCBP1:1, IOCBP2:1, IOCBP3:1,
             IOCBP4:1, IOCBP5:1, IOCBP6:1, IOCBP7:1;
}; } iocbp_t;
typedef union { unsigned char byte; struct {
    unsigned PS0:1, PS1:1, PS2:1, PSA:1, T0SE:1, T0CS:1, INTEDG:1, nWPUEN:1;
}; } option_t;
typedef union { unsigned char byte; struct {
    unsigned SWDTEN:1, WDTPS0:1, WDTPS1:1, WDTPS2:1, WDTPS3:1, WDTPS4:1, pad:2;
}; } wdtcon_t;
typedef union { unsigned char byte; struct { unsigned C1ON:1, pad:7; }; } cm1con0_t;
typedef union { unsigned char byte; struct { unsigned C2ON:1, pad:7; }; } cm2con0_t;
typedef union { unsigned char byte; struct {
    unsigned FVRS0:1, FVRS1:1, TSRNG:1, TSEN:1, CDAFVR0:1, CDAFVR1:1, FVRRDY:1, FVREN:1;
}; } fvrcon_t;
typedef union { unsigned char byte; struct {
    unsigned ADON:1, ADGO:1, CHS:5, pad:2;
}; } adcon0_t;
typedef union { unsigned char byte; struct {
    unsigned ADPREF0:1, ADPREF1:1, ADNREF:1, pad:1, ADCS0:1, ADCS1:1, ADCS2:1, ADFM:1;
}; } adcon1_t;
typedef union { unsigned char byte; struct {
    unsigned WPUB0:1, WPUB1:1, WPUB2:1, WPUB3:1, WPUB4:1, WPUB5:1, WPUB6:1, WPUB7:1;
}; } wpub_t;
typedef union { unsigned char byte; struct {
    unsigned C:1, DC:1, Z:1, nPD:1, nTO:1, pad:3;
}; } status_t;
typedef union { unsigned char byte; struct {
    unsigned TMR1IF:1, TMR2IF:1, CCP1IF:1, SSP1IF:1, TXIF:1, RCIF:1, ADIF:1, TMR1GIF:1;
}; } pir1_t;
typedef union { unsigned char byte; struct {
    unsigned TX9D:1, TRMT:1, BRGH:1, SENDB:1, SYNC:1, TXEN:1, TX9:1, CSRC:1;
}; } txsta_t;
typedef union { unsigned char byte; struct {
    unsigned RX9D:1, OERR:1, FERR:1, ADDEN:1, CREN:1, SREN:1, RX9:1, SPEN:1;
}; } rcsta_t;

extern volatile porta_t   PORTAbits;
extern volatile portb_t   PORTBbits;
extern volatile lata_t    LATAbits;
extern volatile latb_t    LATBbits;
extern volatile latc_t    LATCbits;
extern volatile trisa_t   TRISAbits;
extern volatile trisb_t   TRISBbits;
extern volatile ansela_t  ANSELAbits;
extern volatile anselb_t  ANSELBbits;
extern volatile intcon_t  INTCONbits;
extern volatile iocbf_t   IOCBFbits;
extern volatile iocbn_t   IOCBNbits;
extern volatile iocbp_t   IOCBPbits;
extern volatile option_t  OPTION_REGbits;
extern volatile wdtcon_t  WDTCONbits;
extern volatile cm1con0_t CM1CON0bits;
extern volatile cm2con0_t CM2CON0bits;
extern volatile fvrcon_t  FVRCONbits;
extern volatile adcon0_t  ADCON0bits;
extern volatile adcon1_t  ADCON1bits;
extern volatile wpub_t    WPUBbits;
extern volatile status_t  STATUSbits;
extern volatile pir1_t    PIR1bits;
extern volatile txsta_t   TXSTAbits;
extern volatile rcsta_t   RCSTAbits;

#endif /* HOSTBUILD_XC_H */
