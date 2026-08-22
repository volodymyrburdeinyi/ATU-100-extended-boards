// Main.h
// David Fainitski
// ATU-100 project 2016
//
// Interface of main.c. Shared state lives in globals.h; the tuning algorithm
// is reached through tune_api.h.

#ifndef MAIN_H
#define MAIN_H

#include "cross_compiler.h"
#include "globals.h"
/* relay HAL: set_ind, set_cap, set_sw, atu_reset */
#include "relay.h"
/* SWR/power measurement: correction, get_reverse, get_forward, get_pwr, get_swr */
#include "swr.h"
/* cooperative tune state machine: tune_start, tune_tick, tune_busy */
#include "tune_api.h"

void pic_init(void);

void tune_btn_push(void);
void show_reset(void);
void lcd_ind(void);  /* no-op stub — called by the algorithm; no display hardware */
void cells_init(void);
void Test_init(void);
void button_proc(void);
void button_proc_test(void);
void button_delay(void);

#endif /* MAIN_H */
