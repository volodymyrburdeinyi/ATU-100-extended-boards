/* globals.c — the single definition of every cross-module global.
 * Declarations and documentation are in globals.h. */

#include "cross_compiler.h"
#include "globals.h"

unsigned char g_c_ind = 0, g_c_cap = 0;
char g_c_SW = 0;
char g_c_step_cap = 0, g_c_step_ind = 0;
char g_c_L_mult = 1, g_c_C_mult = 1;

int  g_i_SWR = 0, g_i_PWR = 0, g_i_P_max = 0, g_i_swr_a = 0;
char g_b_rready = 0, g_char_p_cnt = 0;
char g_b_Overload = 0;
char g_b_tx_seen = 0;
unsigned char g_char_tune_effort = 0;

char g_b_Auto_mode = 0;

int  e_i_Cap1 = 0, e_i_Cap2 = 0, e_i_Cap3 = 0, e_i_Cap4 = 0,
     e_i_Cap5 = 0, e_i_Cap6 = 0, e_i_Cap7 = 0;
int  e_i_Ind1 = 0, e_i_Ind2 = 0, e_i_Ind3 = 0, e_i_Ind4 = 0,
     e_i_Ind5 = 0, e_i_Ind6 = 0, e_i_Ind7 = 0;
char e_c_b_L_linear = 0, e_c_b_C_linear = 0;
char e_c_num_L_q = 7, e_c_num_C_q = 7;
char e_c_b_D_correction = 1;
char e_c_b_L_invert = 0;
int  e_i_tenths_SWR_Auto_delta = 0;
int  e_i_ms_Rel_Del = 0;
int  e_i_watts_min_for_start = 0, e_i_watts_max_for_start = 0;
int  e_i_tenths_init_max_swr = 0;
char e_c_b_P_High = 0;
char e_c_K_Mult = 32;
char e_c_b_Loss_ind = 0;
char e_c_b_Relay_off = 0;

#ifdef UART
unsigned int  g_i_uart_freq_hint = 0;
unsigned char g_b_slot_saved     = 0;
unsigned char g_c_tune_exit      = 0;
unsigned char g_b_debug_mode     = 0;
unsigned char g_b_tune_abort     = 0;
#endif
