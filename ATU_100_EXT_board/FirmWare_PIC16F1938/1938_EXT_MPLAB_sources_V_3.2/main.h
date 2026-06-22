
#include "cross_compiler.h"

// Main.h
// David Fainitski
// ATU-100 project 2016

//
static unsigned char g_c_ind = 0, g_c_cap = 0;
static char g_c_SW = 0;
static char g_c_step_cap = 0, g_c_step_ind = 0;
static char g_c_L_mult = 1, g_c_C_mult = 1;
int g_i_SWR, g_i_PWR, g_i_P_max, g_i_swr_a;
static char g_b_rready = 0, g_char_p_cnt = 0;

static char g_b_Overload = 0;
static char g_b_tx_seen = 0;
static unsigned char g_char_tune_effort = 0;

static int e_i_Cap1, e_i_Cap2, e_i_Cap3, e_i_Cap4, e_i_Cap5, e_i_Cap6, e_i_Cap7;
static int e_i_Ind1, e_i_Ind2, e_i_Ind3, e_i_Ind4, e_i_Ind5, e_i_Ind6, e_i_Ind7;
/* booleans indicating whether the inductors and caps are linearly spaced */
static char e_c_b_L_linear = 0, e_c_b_C_linear = 0;
/* number of inductors and caps */
static char e_c_num_L_q = 7, e_c_num_C_q = 7;
/*  boolean indicating whether we should correct for the diode */
static char e_c_b_D_correction = 1;
/*  boolean indicating if the inductors are using relays with normally open contact */
static char e_c_b_L_invert = 0;
/* in automatic mode, threshold triggering the tuning process if the
 * SWR is above this level.   in tenths.   so 13 would be an SWR of 1.3 */
static int e_i_tenths_SWR_Auto_delta;
/* boolean:  if display backlight should be on when any buttons are pressed
 and if RF power is seen */
static char e_c_b_Dysp_delay = 0;
/* feeder loss in tenths of a db.  */
static char e_c_tenths_Fid_loss;
/* delay in ms for relays to settle and to take a measurement */
static int e_i_ms_Rel_Del;
/*  power in watts for starting tuning.  min and max */
static int e_i_watts_min_for_start, e_i_watts_max_for_start;
/*  max SWR for tuning, in tenths.  0 = do not check */
static int e_i_tenths_init_max_swr;
/* boolean support high power measurements */
static char e_c_b_P_High = 0;
/* ratio of turns of the tandem match (default should be 10) */
static char e_c_K_Mult = 32;
/* bool indicating whether to display Power level also */
static char e_c_b_Loss_ind = 0;
/* boolean.  to support Relay off function */
static char e_c_b_Relay_off = 0;

//
void pic_init(void);

void tune_btn_push(void);
void lcd_prep(void);
void lcd_swr(int);
void lcd_pwr(void);
void show_pwr(int, int);
void lcd_ind(void);
void show_reset(void);
void cells_init(void);
void test_init(void);
void button_proc(void);
void button_proc_test(void);
void button_delay(void);
void show_loss(void);
/* relay HAL: set_ind, set_cap, set_sw, atu_reset */
#include "relay.h"
/* SWR/power measurement: correction, get_reverse, get_forward, get_pwr, get_swr */
#include "swr.h"
/* coarse_cap / sharp_cap / sharp_ind / coarse_tune / tune / sub_tune
   are defined in tune_algo.h (single source of truth for firmware + sim) */
//

/* ── frequency measurement ──
 * Returns 0 until the 74HC4060 counter hardware is wired to RD0.
 * When a freq hint arrives via UART ("t HHHH"), tune() uses g_i_uart_freq_hint
 * as a fallback so band-memory probe and slot saves still work headlessly.   */
static unsigned int measure_freq(void) {
   return 0;   /* replaced when 74HC4060 hardware is wired to RD0 */
}

/* ── algorithm: coarse/fine scans, band memory, tune orchestrator ──
 * Single source of truth: also included by tools/atusim.c (simulator).
 * Never edit algorithm logic here — edit tune_algo.h instead.               */
#include "tune_algo.h"
