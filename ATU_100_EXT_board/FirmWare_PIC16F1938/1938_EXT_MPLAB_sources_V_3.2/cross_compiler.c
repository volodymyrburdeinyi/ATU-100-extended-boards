
#include "cross_compiler.h"

#ifdef MPLAB_COMPILER

#pragma config FOSC = 0x04

#endif


#ifdef SIMULATOR

void init_uart(void) {
    TXSTAbits.TXEN = 1;               // enable transmitter
    RCSTAbits.SPEN = 1;               // enable serial port
}
void putch(unsigned char data) {
    while( ! PIR1bits.TXIF)          // wait until the transmitter is ready
        continue;
    TXREG = data;                     // send one character
}

char firstcall = 0;
/*   make these 'global' since having them as parameters puts a 
     real burden on the stack space */
char tempstring[100];
char terminator = 0;
char stringlength = 0;
char* stringptr = NULL;

void debugprint()
{
   char c;
   if (firstcall == 0)
   {
       init_uart();
       firstcall = 1;
   }
   for (c = 0; c < stringlength; c++)
   {
      putch(stringptr[c]);
   }
   if (terminator)
   {
       putch('\n');
   }
}
char mystring[] = "Delay called with ";
void IntToStr(int number, char *output);
#endif

void Delay_ms(const unsigned int time_in_ms)
{
  unsigned int i = time_in_ms;
  CLRWDT();
  
#ifdef SIMULATOR  
  if (time_in_ms > 99)
  {
    PRINTTEXT(mystring)  
                        
    IntToStr((int)(time_in_ms),&tempstring[0]);
    PRINTTEMPSTRINGLINE(6);
  }
#endif

  while (i > 0)
  {
    /*        _delay((unsigned long)((5)*(_XTAL_FREQ/4000.0)));*/
    _delay((unsigned long)(4000));
    CLRWDT();
    i--;
  }
};

unsigned int ADC_Get_Sample(char channel)
{
  ADCON0bits.CHS = channel;
  ADCON0bits.ADGO = 1; /* start A/D */
  Delay_5_us();
  while (ADCON0bits.ADGO == 1)
    ;
  unsigned char lower = ADRESL;
  unsigned char upper = ADRESH;
  unsigned int result = (unsigned int)(upper << 8) + lower;
  return result;
};

/*  routine to detect button presses and debouncing.  input parms:
 port number, port-pin number  time in ms to delay, and value to expect.
 * returns 1 if the pin held the active state for the whole period, else 0.
 * (it used to return 255 from a signed char, i.e. -1: truthy, but never
 *  equal to the 255 the comment promised.)
 *
 * Note this blocks for the full `time` in ms when the pin IS in the state
 * being tested — callers must short-circuit with && rather than &.
 * we use PORTB, pins 0, 1 and 2 */
unsigned char Button(volatile unsigned char *port, char pin, char time, char active_state)
{
  char loop = time;
  char value;
  while (loop > 0)
  {
    switch (pin)
    {
    case 0:
      value = PORTBbits.RB0;
      break;
    case 1:
      value = PORTBbits.RB1;
      break;
    default:
      value = PORTBbits.RB2;
      break;
    }
    if (value != active_state)
    {
      return 0;
    }
    Delay_ms(1);
    loop--;
  }
  return 1;
};

unsigned char Bcd2Dec(unsigned char bcdnum)
{
  return ((bcdnum / 16 * 10) + (bcdnum % 16));
};

unsigned char Dec2Bcd(unsigned char decnum)
{
  return ((decnum / 10 * 16) + (decnum % 10));
};

void Vdelay_ms(int time_in_ms)
{
  Delay_ms((const unsigned int)(time_in_ms));
};

void ADC_Init(void){};

/*  Write an EEPROM cell only when the value actually changes. An EEPROM write
 *  costs ~4 ms and one of the cell's ~100k endurance cycles whether or not the
 *  content differs, and the LAST_* cells are rewritten after every tune. */
void eeprom_store(unsigned char addr, unsigned char value)
{
  if (eeprom_read(addr) != value)
    eeprom_write(addr, value);
}

/*   Function creates an OUTPUT string out of a signed number (numerical
 * value of int type).  Output string has fixed width of 6 characters;
 * remaining positions on the left (if any ) are filled with blanks.
 *
 * The result is NUL-terminated, so output must have room for 7 bytes.
 * Without the terminator every caller that passed the buffer to a string
 * function walked off the end of it: uart_puts() emitted whatever followed
 * the buffer until it happened to find a zero, corrupting every status and
 * DBG line the tuner sent. */
void IntToStr(int number, char *output)
{
  char *p = output;
  char loopcounter = 0;
  do
  {
    *p = ' '; /* fill with blanks */
    p++;
    loopcounter++;
  } while (loopcounter < 6);
  *p = '\0';
  p = output + 5; /* point to the last digit */
  if (number >= 0)
  {
    do
    {
      *p = '0' + (number % 10);
      p--;
      number /= 10;
      loopcounter++;
    } while (number != 0);
  }
  else
  { /* i < 0 */
    do
    {
      *p = '0' - (number % 10);
      p--;
      number /= 10;
    } while (number != 0);
    *p = '-';
  }
};
