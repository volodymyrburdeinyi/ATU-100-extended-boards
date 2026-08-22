/* host_stubs.c — storage for the stubbed SFRs plus an entry point.
 *
 * See xc.h. main() here is never the firmware's main(): the firmware's own
 * main() is renamed by the build script so that both can be linked, which is
 * what makes the check a genuine whole-program link.
 */
#include "xc.h"

volatile unsigned char PORTA, PORTB, PORTC;
volatile unsigned char LATA, LATB, LATC;
volatile unsigned char TRISA, TRISB, TRISC;
volatile unsigned char ANSELA, ANSELB;
volatile unsigned char OSCCON, OPTION_REG, WDTCON;
volatile unsigned char TMR0, ADCON0, ADCON1, ADRESL, ADRESH, FVRCON;
volatile unsigned char TXREG;

volatile porta_t   PORTAbits;
volatile portb_t   PORTBbits;
volatile lata_t    LATAbits;
volatile latb_t    LATBbits;
volatile latc_t    LATCbits;
volatile trisa_t   TRISAbits;
volatile trisb_t   TRISBbits;
volatile ansela_t  ANSELAbits;
volatile anselb_t  ANSELBbits;
volatile intcon_t  INTCONbits;
volatile iocbf_t   IOCBFbits;
volatile iocbn_t   IOCBNbits;
volatile iocbp_t   IOCBPbits;
volatile option_t  OPTION_REGbits;
volatile wdtcon_t  WDTCONbits;
volatile cm1con0_t CM1CON0bits;
volatile cm2con0_t CM2CON0bits;
volatile fvrcon_t  FVRCONbits;
volatile adcon0_t  ADCON0bits;
volatile adcon1_t  ADCON1bits;
volatile wpub_t    WPUBbits;
volatile status_t  STATUSbits;
volatile pir1_t    PIR1bits;
volatile txsta_t   TXSTAbits;
volatile rcsta_t   RCSTAbits;

static unsigned char host_eeprom[256];

unsigned char eeprom_read(unsigned char addr)
{
    return host_eeprom[addr];
}

void eeprom_write(unsigned char addr, unsigned char value)
{
    host_eeprom[addr] = value;
}

int main(void)
{
    /* Linking is the test; running the firmware on the host is meaningless. */
    return 0;
}
