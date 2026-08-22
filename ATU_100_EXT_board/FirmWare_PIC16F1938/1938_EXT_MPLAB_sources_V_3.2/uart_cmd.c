#include "cross_compiler.h"

#ifdef UART

#include "globals.h"
#include "relay.h"
#include "swr.h"
#include "tune_api.h"
#include "uart_cmd.h"

/* ── UART output helpers ── */

static void uart_putuint(int v)
{
    char l_s[7];
    unsigned char l_i = 0;
    IntToStr(v, l_s);
    while (l_i < 5u && l_s[l_i] == ' ') l_i++;
    uart_puts(l_s + l_i);
}

static void uart_puthex8(unsigned char v)
{
    const char l_hex[] = "0123456789abcdef";
    uart_tx_bit_bang(l_hex[(v >> 4) & 0x0f]);
    uart_tx_bit_bang(l_hex[v & 0x0f]);
}

static unsigned char parse_hex8(const char *s)
{
    unsigned char l_hi, l_lo;
    if (s[0] >= '0' && s[0] <= '9')      l_hi = (unsigned char)(s[0] - '0');
    else if (s[0] >= 'a' && s[0] <= 'f') l_hi = (unsigned char)(s[0] - 'a' + 10u);
    else if (s[0] >= 'A' && s[0] <= 'F') l_hi = (unsigned char)(s[0] - 'A' + 10u);
    else return 0;
    if (s[1] >= '0' && s[1] <= '9')      l_lo = (unsigned char)(s[1] - '0');
    else if (s[1] >= 'a' && s[1] <= 'f') l_lo = (unsigned char)(s[1] - 'a' + 10u);
    else if (s[1] >= 'A' && s[1] <= 'F') l_lo = (unsigned char)(s[1] - 'A' + 10u);
    else return 0;
    return (unsigned char)((l_hi << 4) | l_lo);
}

static unsigned int parse_hex16(const char *s)
{
    unsigned int  l_val = 0;
    unsigned char l_i, l_d;
    for (l_i = 0; l_i < 4u; l_i++) {
        if (s[l_i] >= '0' && s[l_i] <= '9')      l_d = (unsigned char)(s[l_i] - '0');
        else if (s[l_i] >= 'a' && s[l_i] <= 'f') l_d = (unsigned char)(s[l_i] - 'a' + 10u);
        else if (s[l_i] >= 'A' && s[l_i] <= 'F') l_d = (unsigned char)(s[l_i] - 'A' + 10u);
        else return 0;
        l_val = (unsigned int)((l_val << 4) | (unsigned int)l_d);
    }
    return l_val;
}

static void uart_puthex16(unsigned int v)
{
    uart_puthex8((unsigned char)(v >> 8));
    uart_puthex8((unsigned char)(v & 0xFFu));
}

void uart_send_status(void)
{
    uart_puts("IND=");   uart_putuint(g_c_ind);
    uart_puts(" CAP=");  uart_putuint(g_c_cap);
    uart_puts(" SW=");   uart_putuint(g_c_SW);
    uart_puts(" SWR=");  uart_putuint(g_i_SWR);
    uart_puts(" AUTO="); uart_putuint(g_b_Auto_mode);
    uart_puts(" EFF=");   uart_putuint(g_char_tune_effort);
    uart_puts(" SLOTS="); uart_putuint(EEPROM_BAND_SLOT_COUNT);
    uart_puts(" SAVED="); uart_putuint(g_b_slot_saved);
    uart_puts(" EXIT=");  uart_putuint(g_c_tune_exit);
    uart_puts("\r\n");
}

/* How far a stored slot may be from the requested frequency and still be worth
   applying. Wider than the ±25 kHz the tune-time probe uses, because a recall
   is only a starting point, but bounded: the nearest slot in a band can be
   hundreds of kHz away, and on a narrow antenna that is a worse starting point
   than the position the tuner is already in. */
#define RECALL_TOL_KHZ 150u

/* Find best-matching sub-slot within the current band for l_freq_kHz.
   If found: apply relays, load stored SWR, return 1. Return 0 if no match.
   No live SWR measurement — safe to call in RX (no TX power present). */
static signed char band_slot_apply_freq(unsigned int l_freq_kHz)
{
    unsigned char l_band, l_sub, l_slot_idx, l_base, l_ind, l_sw_swr;
    unsigned int  l_sf, l_diff, l_best_diff;
    unsigned char l_best_sub;
    if (l_freq_kHz == 0) return 0;
    l_band = freq_to_band_idx(l_freq_kHz);
    if (l_band == 0xFF) return 0;
    l_best_diff = 0xFFFFu;
    l_best_sub  = 0xFF;
    for (l_sub = 0; l_sub < (unsigned char)EEPROM_BAND_SUB_N; l_sub++) {
        l_slot_idx = (unsigned char)(l_band * (unsigned char)EEPROM_BAND_SUB_N + l_sub);
        l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
        l_ind = eeprom_read(l_base + EEPROM_SLOT_IND);
        if (l_ind == 0xFF) continue;
        l_sf = (unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_LO)
             | ((unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_HI) << 8);
        if (l_sf == 0) continue;
        l_diff = (l_sf > l_freq_kHz) ? (unsigned int)(l_sf - l_freq_kHz)
                                      : (unsigned int)(l_freq_kHz - l_sf);
        if (l_diff > RECALL_TOL_KHZ) continue;
        if (l_diff < l_best_diff) { l_best_diff = l_diff; l_best_sub = l_sub; }
    }
    if (l_best_sub == 0xFF) return 0;
    l_slot_idx = (unsigned char)(l_band * (unsigned char)EEPROM_BAND_SUB_N + l_best_sub);
    l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
    l_ind    = eeprom_read(l_base + EEPROM_SLOT_IND);
    l_sw_swr = eeprom_read(l_base + EEPROM_SLOT_SW_SWR);
    g_c_ind = l_ind;
    g_c_cap = eeprom_read(l_base + EEPROM_SLOT_CAP);
    g_c_SW  = (char)((l_sw_swr >> 7) & 1u);
    set_ind(g_c_ind);
    set_cap(g_c_cap);
    set_sw(g_c_SW);
    g_i_SWR = (int)(l_sw_swr & 0x7Fu) * 10;
    return 1;
}

static void uart_exec_cmd(const char *l_cmd, unsigned char l_len)
{
    /* While a tune is running the command parser is being called from inside
       the relay scan loops, with the transmitter keyed. Only abort and status
       are safe there: 'r' would move the relays out from under the scan, 'c'
       would write EEPROM mid-tune, 'm' would hold the line for a second. The
       daemon already restricts itself to 'q', but RB2 doubles as a button pin
       on a 1.5 kW tuner, so RF pickup can synthesise a command byte. */
    if (tune_busy() && l_cmd[0] != 'q' && l_cmd[0] != '?') {
        uart_puts("BUSY\r\n");
        return;
    }

    if (l_cmd[0] == 't') {
        /* "t HHHH" — tune; optional 4-digit kHz freq hint in hex e.g. "t 1BA2".
         * tune_start() only queues TS_INIT; main loop drives tune_tick() and sends
         * uart_send_status() when tune_tick() returns 1 (done). */
        g_i_uart_freq_hint = (l_len >= 6u) ? parse_hex16(l_cmd + 2) : 0;
        tune_start();
    } else if (l_cmd[0] == 'q') {
        /* "q" — abort a running tune; relay state is restored by TS_ABORT */
        g_b_tune_abort = 1;
        uart_puts("OK ABORT\r\n");
    } else if (l_cmd[0] == 'l' && l_len >= 6u) {
        /* "l HHHH" — recall nearest slot for freq in kHz, apply relays, report */
        unsigned int  l_freq_kHz = parse_hex16(l_cmd + 2);
        g_i_uart_freq_hint = l_freq_kHz;
        signed char l_result = band_slot_apply_freq(l_freq_kHz);
        if (l_result == 1) {
            uart_puts("RECALL IND="); uart_putuint(g_c_ind);
            uart_puts(" CAP=");       uart_putuint(g_c_cap);
            uart_puts(" SW=");        uart_putuint(g_c_SW);
            uart_puts(" SWR=");       uart_putuint(g_i_SWR);
            uart_puts("\r\n");
        } else {
            uart_puts("NOMATCH\r\n");
        }
    } else if (l_cmd[0] == 'm') {
        /* "m" — dump all 30 band sub-slots (10 bands × 3) */
        unsigned char l_slot, l_base, l_ind, l_sw_swr;
        unsigned int  l_sf;
        for (l_slot = 0; l_slot < (unsigned char)EEPROM_BAND_SLOT_COUNT; l_slot++) {
            CLRWDT();   /* 30 slots × ~41ms bit-bang = ~1.2s > WDT 1024ms */
            l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
            l_ind = eeprom_read(l_base + EEPROM_SLOT_IND);
            uart_puts("SLOT "); uart_putuint(l_slot);
            if (l_ind == 0xFF) {
                uart_puts(" EMPTY\r\n");
            } else {
                l_sf = (unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_LO)
                     | ((unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_HI) << 8);
                l_sw_swr = eeprom_read(l_base + EEPROM_SLOT_SW_SWR);
                uart_puts(" FREQ=0x"); uart_puthex16(l_sf);
                uart_puts(" IND=");    uart_putuint(l_ind);
                uart_puts(" CAP=");    uart_putuint(eeprom_read(l_base + EEPROM_SLOT_CAP));
                uart_puts(" SW=");     uart_putuint((l_sw_swr >> 7) & 1u);
                uart_puts(" SWR=");    uart_putuint((int)(l_sw_swr & 0x7Fu) * 10);
                uart_puts("\r\n");
            }
        }
    } else if (l_cmd[0] == 'e' && l_len >= 4u) {
        /* "e HH" — read one EEPROM cell */
        unsigned char l_addr = parse_hex8(l_cmd + 2);
        uart_puts("EEPROM[0x"); uart_puthex8(l_addr);
        uart_puts("]=0x");      uart_puthex8(eeprom_read(l_addr));
        uart_puts("\r\n");
    } else if (l_cmd[0] == 'a') {
        g_b_Auto_mode = g_b_Auto_mode ? 0 : 1;
        eeprom_write(EEPROM_AUTOMATIC_MODE, g_b_Auto_mode);
        uart_puts("OK AUTO=");
        uart_putuint(g_b_Auto_mode);
        uart_puts("\r\n");
    } else if (l_cmd[0] == 'r') {
        atu_reset();
        uart_puts("OK RESET\r\n");
    } else if (l_cmd[0] == 'c' && l_len >= 7u) {
        /* "c HH VV" — write EEPROM cell, e.g. "c 36 08"               */
        unsigned char l_addr = parse_hex8(l_cmd + 2);
        unsigned char l_val  = parse_hex8(l_cmd + 5);
        eeprom_write(l_addr, l_val);
        uart_puts("OK EEPROM[0x"); uart_puthex8(l_addr);
        uart_puts("]=0x");         uart_puthex8(l_val);
        uart_puts("\r\n");
    } else if (l_cmd[0] == 'd') {
        g_b_debug_mode = (l_len >= 3u && l_cmd[2] == '1') ? 1 : 0;
        uart_puts("OK DBG=");
        uart_putuint(g_b_debug_mode);
        uart_puts("\r\n");
    } else if (l_cmd[0] == '?') {
        uart_send_status();
    } else {
        uart_puts("ERR\r\n");
    }
}

void uart_cmd_proc(void)
{
    static char l_buf[12];
    static unsigned char l_len = 0;
    unsigned char l_c;

    while (uart_rx_avail()) {
        l_c = uart_rx_getc();
        if (l_c == '\r' || l_c == '\n') {
            if (l_len > 0u) {
                l_buf[l_len] = '\0';
                uart_exec_cmd(l_buf, l_len);
                l_len = 0;
            }
        } else if (l_c >= 0x20u && l_c <= 0x7Eu) {
            if (l_len < 11u)
                l_buf[l_len++] = (char)l_c;
        } else {
            l_len = 0;   /* non-printable = RF noise — discard partial command */
        }
    }
}

#endif /* UART */
