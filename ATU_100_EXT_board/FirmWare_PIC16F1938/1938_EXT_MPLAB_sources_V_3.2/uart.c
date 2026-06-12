#include "cross_compiler.h"

/* Bit-bang TX: transmit one byte, LSB first, 9600 baud 8N1.
   GIE disabled during transmission to protect bit timing.              */
void uart_tx_bit_bang(unsigned char val)
{
    unsigned char i;
    unsigned char l_gie = INTCONbits.GIE;
    INTCONbits.GIE = 0;

    UART_OUT_PIN = 0;                    /* start bit                   */
    Delay_100_us();
    for (i = 8; i != 0; --i)
    {
        UART_OUT_PIN = val & 0x01;
        val >>= 1;
        Delay_100_us();
    }
    UART_OUT_PIN = 1;                    /* stop bit                    */
    Delay_100_us();
    Delay_100_us();

    INTCONbits.GIE = l_gie;
}

void uart_puts(const char *s)
{
    while (*s)
        uart_tx_bit_bang((unsigned char)*s++);
}

/* uart_wr_str: display-protocol wrapper used by existing led_wr_str path */
void uart_wr_str(char posstr[], char str[], char leng)
{
    char i;
    if (posstr[2] == ' ') posstr[2] = '0';
    if (posstr[3] == ' ') posstr[3] = '0';
    if (posstr[4] == ' ') posstr[4] = '0';
    if (posstr[5] == ' ') posstr[5] = '0';
    for (i = 2; i < 6; i++)
        uart_tx_bit_bang(posstr[i]);
    uart_tx_bit_bang(':');
    for (i = 0; i < leng; i++)
    {
        if (str[i] == 0) break;
        uart_tx_bit_bang(str[i]);
    }
    uart_tx_bit_bang(0x0d);
    uart_tx_bit_bang(0x0a);
}

#ifdef UART

/* Bit-bang RX state machine — Timer0 ISR, 9600 baud 8N1.
   Timer0: internal clock, 1:4 prescaler → 1 µs/tick at 16 MHz CPU.
   After start-bit edge: arm for 1.5 bit (156 µs) to land at mid-bit-0.
   Subsequent Timer0 reloads: 1 bit period (104 µs).                    */
#define TMR0_1P5BIT  ((unsigned char)(256u - 156u))  /* 100: first sample */
#define TMR0_1BIT    ((unsigned char)(256u - 104u))  /* 152: per-bit      */

#define UART_RX_BUFSIZE  16u

static volatile unsigned char v_rx_state;   /* 0=idle, 1..8=bits, 9=stop */
static volatile unsigned char v_rx_shift;
static volatile unsigned char v_rx_buf[UART_RX_BUFSIZE];
static volatile unsigned char v_rx_head;
static volatile unsigned char v_rx_tail;

void __interrupt() isr(void)
{
    /* IOC on RB2: detect start bit (falling edge)                       */
    if (INTCONbits.IOCIF) {
        if (IOCBFbits.IOCBF2) {
            if (UART_RX_PIN == 0 && v_rx_state == 0) {
                v_rx_state = 1;
                v_rx_shift = 0;
                TMR0 = TMR0_1P5BIT;
                INTCONbits.TMR0IF = 0;
                INTCONbits.TMR0IE = 1;
                INTCONbits.IOCIE  = 0;   /* disable IOC during byte     */
            }
            IOCBFbits.IOCBF2 = 0;
        }
        INTCONbits.IOCIF = 0;
    }

    /* Timer0: sample each data bit, then re-arm IOC after stop bit      */
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0 = TMR0_1BIT;

        if (v_rx_state >= 1u && v_rx_state <= 8u) {
            unsigned char l_next;
            v_rx_shift >>= 1;
            if (UART_RX_PIN)
                v_rx_shift |= 0x80u;
            if (v_rx_state == 8u) {
                /* all 8 bits — push to ring buffer if space available   */
                l_next = (unsigned char)((v_rx_head + 1u) & (UART_RX_BUFSIZE - 1u));
                if (l_next != v_rx_tail) {
                    v_rx_buf[v_rx_head] = v_rx_shift;
                    v_rx_head = l_next;
                }
            }
            v_rx_state++;
        } else {
            /* stop bit interval — restore IOC for next byte             */
            v_rx_state = 0;
            INTCONbits.TMR0IE = 0;
            IOCBFbits.IOCBF2  = 0;
            INTCONbits.IOCIF  = 0;
            INTCONbits.IOCIE  = 1;
        }
    }
}

unsigned char uart_rx_avail(void)
{
    return v_rx_head != v_rx_tail;
}

unsigned char uart_rx_getc(void)
{
    unsigned char l_c = v_rx_buf[v_rx_tail];
    v_rx_tail = (unsigned char)((v_rx_tail + 1u) & (UART_RX_BUFSIZE - 1u));
    return l_c;
}

#endif /* UART */
