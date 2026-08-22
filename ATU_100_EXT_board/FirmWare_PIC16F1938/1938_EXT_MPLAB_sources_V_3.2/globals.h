/* globals.h — declarations of every global shared between translation units.
 *
 * Definitions live in globals.c, exactly once. Before this file existed the
 * definitions sat in main.h with `static` (internal) linkage while relay.c,
 * swr.c and uart_cmd.c referenced them with hand-written `extern` blocks —
 * which cannot link, and let the declared type drift from the definition.
 *
 * Naming (WA1RCT XC8 port convention):
 *   g_i_ global int    g_c_ global char   g_b_ global boolean (char)
 *   e_i_ / e_c_ / e_c_b_  value loaded from EEPROM at startup by cells_init()
 *
 * XC8 note: char is signed by default. Anything that must hold 0xFF, an EEPROM
 * address, or a relay bit-field is declared unsigned explicitly.
 */
#ifndef GLOBALS_H
#define GLOBALS_H

/* ── relay position ── */
extern unsigned char g_c_ind, g_c_cap;
/* capacitor bank position: 0 = antenna side, 1 = TX side. NOT swr. */
extern char g_c_SW;
extern char g_c_step_cap, g_c_step_ind;
extern char g_c_L_mult, g_c_C_mult;

/* ── measurement ──
 * g_i_SWR is SWR x 100 (100 = 1.0:1, 999 = max). 0 is the TX-inhibit
 * abort sentinel and is checked as such all through the tune state machine. */
extern int g_i_SWR, g_i_PWR, g_i_P_max, g_i_swr_a;
extern char g_b_rready, g_char_p_cnt;
extern char g_b_Overload;
/* set once power exceeds the minimum during a tune; makes a subsequent power
 * drop mean "TX inhibited" rather than "not keyed yet". Reset per tune. */
extern char g_b_tx_seen;
/* count of get_swr() calls this tune session, saturating at 255 */
extern unsigned char g_char_tune_effort;

/* ── operating mode ── */
extern char g_b_Auto_mode;

/* ── EEPROM-loaded settings ── */
extern int  e_i_Cap1, e_i_Cap2, e_i_Cap3, e_i_Cap4, e_i_Cap5, e_i_Cap6, e_i_Cap7;
extern int  e_i_Ind1, e_i_Ind2, e_i_Ind3, e_i_Ind4, e_i_Ind5, e_i_Ind6, e_i_Ind7;
/* are the inductors / caps linearly spaced? */
extern char e_c_b_L_linear, e_c_b_C_linear;
/* number of inductors / caps fitted */
extern char e_c_num_L_q, e_c_num_C_q;
/* correct for diode non-linearity in the power measurement */
extern char e_c_b_D_correction;
/* inductor relays are normally-open */
extern char e_c_b_L_invert;
/* auto-mode retune threshold, in tenths of an SWR point (13 = 1.3:1) */
extern int  e_i_tenths_SWR_Auto_delta;
/* relay settling delay, ms */
extern int  e_i_ms_Rel_Del;
/* power window in which tuning may start, watts */
extern int  e_i_watts_min_for_start, e_i_watts_max_for_start;
/* refuse to tune above this SWR, in tenths. 0 = no check */
extern int  e_i_tenths_init_max_swr;
/* high-power (1500 W) rather than 150 W measurement scaling */
extern char e_c_b_P_High;
/* tandem match turns ratio */
extern char e_c_K_Mult;
/* headless station: loss indication always off, no display to show it on */
extern char e_c_b_Loss_ind;
extern char e_c_b_Relay_off;

#ifdef UART
/* kHz frequency hint from "t HHHH"; consumed once by the tune state machine */
extern unsigned int  g_i_uart_freq_hint;
/* set by band_slot_save() when a slot was written; cleared at tune start */
extern unsigned char g_b_slot_saved;
/* exit-path code from the tune state machine, reported by uart_send_status() */
extern unsigned char g_c_tune_exit;
/* "d 1" enables DBG telemetry lines during a tune */
extern unsigned char g_b_debug_mode;
/* set by the "q" command; polled by the tune state machine and by get_swr() */
extern unsigned char g_b_tune_abort;
#endif

#endif /* GLOBALS_H */
