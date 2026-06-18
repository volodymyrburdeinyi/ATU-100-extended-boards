
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
//
void atu_reset(void);
unsigned int get_reverse(void);
unsigned int get_forward(void);
int correction(int);
void get_swr(void);
void get_pwr(void);
void set_sw(char);
void coarse_cap();
void sharp_cap();
void sharp_ind();
void coarse_tune();
void tune(void);
void sub_tune(void);
//

int correction(int input)
{
   //
   if (input <= 80)
      return 0;
   if (input <= 171)
      input += 244;
   else if (input <= 328)
      input += 254;
   else if (input <= 582)
      input += 280;
   else if (input <= 820)
      input += 297;
   else if (input <= 1100)
      input += 310;
   else if (input <= 2181)
      input += 430;
   else if (input <= 3322)
      input += 484;
   else if (input <= 4623)
      input += 530;
   else if (input <= 5862)
      input += 648;
   else if (input <= 7146)
      input += 743;
   else if (input <= 8502)
      input += 800;
   else if (input <= 10500)
      input += 840;
   else
      input += 860;
   //
   return input;
}

//

unsigned int get_reverse()
{
   unsigned int returnReverse;
   FVRCON = 0b10000001; // ADC 1024 vmV Vref
   
   WAIT_FOR_FVR
           
   returnReverse = ADC_Get_Sample(0);
   if (returnReverse <= 1000)
      return returnReverse;
   FVRCON = 0b10000010; // ADC 2048 vmV Vref
   
   WAIT_FOR_FVR
           
   returnReverse = ADC_Get_Sample(0);
   if (returnReverse <= 1000)
      return returnReverse * 2;
   FVRCON = 0b10000011; // ADC 4096 vmV Vref
   
   WAIT_FOR_FVR
           
   returnReverse = ADC_Get_Sample(0);
   return returnReverse * 4;
}
//

unsigned int get_forward()
{
   unsigned int returnUIntValue;
   FVRCON = 0b10000001; // ADC 1024 vmV Vref
   
   WAIT_FOR_FVR
           
   returnUIntValue = ADC_Get_Sample(1);
   if (returnUIntValue <= 1000)
   {
      g_b_Overload = 0;
      return returnUIntValue;
   }
   FVRCON = 0b10000010; // ADC 2048 vmV Vref
   
   WAIT_FOR_FVR
           
   returnUIntValue = ADC_Get_Sample(1);
   if (returnUIntValue <= 1000)
   {
      g_b_Overload = 0;
      return returnUIntValue * 2;
   }
   FVRCON = 0b10000011; // ADC 4096 vmV Vref
   
   WAIT_FOR_FVR
           
   returnUIntValue = ADC_Get_Sample(1);
   if (returnUIntValue > 1000)
      g_b_Overload = 1;
   else
      g_b_Overload = 0;
   return returnUIntValue * 4;
}

void get_pwr()
{
   long l_Forward, l_Reverse;
   double l_doub_pwr;
   CLRWDT();
   //
   l_Forward = get_forward();
   l_Reverse = get_reverse();
   if (e_c_b_D_correction == 1)
      l_doub_pwr = correction((int)(l_Forward * 3));
   else
      l_doub_pwr = l_Forward * 3;
   //
   if (l_Reverse >= l_Forward)
      l_Forward = 999;
   else
   {
      l_Forward = ((l_Forward + l_Reverse) * 100) / (l_Forward - l_Reverse);
      if (l_Forward > 999)
         l_Forward = 999;
   }
   //
   l_doub_pwr = l_doub_pwr * e_c_K_Mult / 1000.0; // mV to Volts on Input
   l_doub_pwr = l_doub_pwr / 1.414;
   if (e_c_b_P_High == 1)
      l_doub_pwr = l_doub_pwr * l_doub_pwr / 50; // 0 - 1500 ( 1500 Watts)
   else
      l_doub_pwr = l_doub_pwr * l_doub_pwr / 5; // 0 - 1510 (151.0 Watts)
   l_doub_pwr = l_doub_pwr + 0.5;      // rounding
   //
   g_i_PWR = (int)(l_doub_pwr);
   if (l_Forward < 100)
      g_i_SWR = 999;
   else
      g_i_SWR = (int)(l_Forward);
   return;
}

void get_swr()
{
   get_pwr();
   if (g_char_p_cnt != 100)
   {
      g_char_p_cnt += 1;
      if (g_i_PWR > g_i_P_max)
         g_i_P_max = g_i_PWR;
   }
   if (g_char_tune_effort < 255) g_char_tune_effort++;
   else
   {
      g_char_p_cnt = 0;
      show_pwr(g_i_P_max, g_i_SWR);
      g_i_P_max = 0;
   }
   if (g_i_PWR >= e_i_watts_min_for_start)
      g_b_tx_seen = 1;
   while ((g_i_PWR < e_i_watts_min_for_start) || (g_i_PWR > e_i_watts_max_for_start && e_i_watts_max_for_start > 0))
   { // waiting for good power
      if (g_b_tx_seen == 1)
      { // TX was active, power dropped — QMX+ inhibit, abort gracefully
         g_i_SWR = 0;
         return;
      }
      CLRWDT();
      get_pwr();
      if (g_char_p_cnt != 100)
      {
         g_char_p_cnt += 1;
         if (g_i_PWR > g_i_P_max)
            g_i_P_max = g_i_PWR;
      }
      else
      {
         g_char_p_cnt = 0;
         show_pwr(g_i_P_max, g_i_SWR);
         g_i_P_max = 0;
      }
      //
      if (Button(&PORTB, TUNE_BUTTON, 5, BUTTON_RELEASED))
         g_b_rready = 1;
      if ((g_b_rready == 1) && Button(&PORTB, TUNE_BUTTON, 5, BUTTON_PRESSED))
      { //  press button  Tune
         show_reset();
         g_i_SWR = 0;
         return;
      }
   } //  good power
   return;
}

void set_ind(unsigned char Ind)
{
   charbits Indbits;
   Indbits.bytes = Ind;

   if (e_c_b_L_invert == 0)
   {
      Ind_005 = Indbits.bits.B0;
      Ind_011 = Indbits.bits.B1;
      Ind_022 = Indbits.bits.B2;
      Ind_045 = Indbits.bits.B3;
      Ind_1 = Indbits.bits.B4;
      Ind_22 = Indbits.bits.B5;
      Ind_45 = Indbits.bits.B6;
      //
   }
   else
   {
      Ind_005 = ~Indbits.bits.B0;
      Ind_011 = ~Indbits.bits.B1;
      Ind_022 = ~Indbits.bits.B2;
      Ind_045 = ~Indbits.bits.B3;
      Ind_1 = ~Indbits.bits.B4;
      Ind_22 = ~Indbits.bits.B5;
      Ind_45 = ~Indbits.bits.B6;
      //
   }
   Vdelay_ms(e_i_ms_Rel_Del);
}

void set_cap(unsigned char Cap)
{
   charbits Capbits;
   Capbits.bytes = Cap;

   Cap_10 = Capbits.bits.B0;
   Cap_22 = Capbits.bits.B1;
   Cap_47 = Capbits.bits.B2;
   Cap_100 = Capbits.bits.B3;
   Cap_220 = Capbits.bits.B4;
   Cap_470 = Capbits.bits.B5;
   Cap_1000 = Capbits.bits.B6;
   //
   Vdelay_ms(e_i_ms_Rel_Del);
}

void set_sw(char l_sw)
{ // 0 - IN,  1 - OUT
   Cap_sw = l_sw;
   Vdelay_ms(e_i_ms_Rel_Del);
}

void atu_reset()
{
   g_c_ind = 0;
   g_c_cap = 0;
   set_ind(g_c_ind);
   set_cap(g_c_cap);
   Vdelay_ms(e_i_ms_Rel_Del);
}

void coarse_cap()
{
   unsigned char l_coarse_cap_step = 3;
   unsigned char l_coarse_cap_count;
   int l_coarse_cap_min_swr;

   g_c_cap = 0;
   set_cap(g_c_cap);
   g_c_step_cap = l_coarse_cap_step;
   get_swr();
   if (g_i_SWR == 0)
      return;
   l_coarse_cap_min_swr = g_i_SWR;
   for (l_coarse_cap_count = l_coarse_cap_step; l_coarse_cap_count <= 31;)
   {
      set_cap(l_coarse_cap_count * g_c_C_mult);
      get_swr();
      if (g_i_SWR == 0)
         return;
      if (g_i_SWR < l_coarse_cap_min_swr)
      {
         l_coarse_cap_min_swr = g_i_SWR;
         g_c_cap = l_coarse_cap_count * g_c_C_mult;
         g_c_step_cap = l_coarse_cap_step;
         if (g_i_SWR < 120)
            break;
      }
      l_coarse_cap_count += l_coarse_cap_step;
      if (e_c_b_C_linear == 0 && l_coarse_cap_count == 9)
         l_coarse_cap_count = 8;
      else if (e_c_b_C_linear == 0 && l_coarse_cap_count == 17)
      {
         l_coarse_cap_count = 16;
         l_coarse_cap_step = 4;
      }
   }
   set_cap(g_c_cap);
   return;
}

void coarse_tune()
{
   unsigned char l_coarse_tune_step = 3;
   unsigned char l_coarse_tune_count;
   unsigned char l_coarse_tune_mem_cap, l_coarse_tune_mem_step_cap;
   int l_coarse_tune_min_swr;

   l_coarse_tune_mem_cap = 0;
   g_c_step_ind = l_coarse_tune_step;
   l_coarse_tune_mem_step_cap = 3;
   l_coarse_tune_min_swr = 9999;
   for (l_coarse_tune_count = 0; l_coarse_tune_count <= 31;)
   {
      set_ind(l_coarse_tune_count * g_c_L_mult);
      coarse_cap();
      get_swr();
      if (g_i_SWR == 0)
         return;
      if (g_i_SWR < l_coarse_tune_min_swr)
      {
         l_coarse_tune_min_swr = g_i_SWR;
         g_c_ind = l_coarse_tune_count * g_c_L_mult;
         l_coarse_tune_mem_cap = g_c_cap;
         g_c_step_ind = l_coarse_tune_step;
         l_coarse_tune_mem_step_cap = g_c_step_cap;
         if (g_i_SWR < 120)
            break;
      }
      l_coarse_tune_count += l_coarse_tune_step;
      if (e_c_b_L_linear == 0 && l_coarse_tune_count == 9)
         l_coarse_tune_count = 8;
      else if (e_c_b_L_linear == 0 && l_coarse_tune_count == 17)
      {
         l_coarse_tune_count = 16;
         l_coarse_tune_step = 4;
      }
   }
   g_c_cap = l_coarse_tune_mem_cap;
   set_ind(g_c_ind);
   set_cap(g_c_cap);
   g_c_step_cap = l_coarse_tune_mem_step_cap;
   Delay_ms(10);
   return;
}

void sharp_cap()
{
   unsigned char l_sharp_cap_range, l_sharp_cap_count, l_sharp_cap_max_range, l_sharp_cap_min_range;
   int l_sharp_cap_min_SWR;
   l_sharp_cap_range = g_c_step_cap * g_c_C_mult;
   //
   l_sharp_cap_max_range = g_c_cap + l_sharp_cap_range;
   if (l_sharp_cap_max_range > 32 * g_c_C_mult - 1)
      l_sharp_cap_max_range = 32 * g_c_C_mult - 1;
   if (g_c_cap > l_sharp_cap_range)
      l_sharp_cap_min_range = g_c_cap - l_sharp_cap_range;
   else
      l_sharp_cap_min_range = 0;
   g_c_cap = l_sharp_cap_min_range;
   set_cap(g_c_cap);
   get_swr();
   if (g_i_SWR == 0)
      return;
   l_sharp_cap_min_SWR = g_i_SWR;
   for (l_sharp_cap_count = l_sharp_cap_min_range + g_c_C_mult; l_sharp_cap_count <= l_sharp_cap_max_range; l_sharp_cap_count += g_c_C_mult)
   {
      set_cap(l_sharp_cap_count);
      get_swr();
      if (g_i_SWR == 0)
         return;
      if (g_i_SWR >= l_sharp_cap_min_SWR)
      {
         Delay_ms(10);
         get_swr();
      }
      if (g_i_SWR >= l_sharp_cap_min_SWR)
      {
         Delay_ms(10);
         get_swr();
      }
      if (g_i_SWR < l_sharp_cap_min_SWR)
      {
         l_sharp_cap_min_SWR = g_i_SWR;
         g_c_cap = l_sharp_cap_count;
         if (g_i_SWR < 120)
            break;
      }
      else
         break;
   }
   set_cap(g_c_cap);
   return;
}

void sharp_ind()
{
   unsigned char l_sharp_ind_range, l_sharp_ind_count, l_sharp_ind_max_range, l_sharp_ind_min_range;
   int l_sharp_ind_min_SWR;
   l_sharp_ind_range = g_c_step_ind * g_c_L_mult;
   //
   l_sharp_ind_max_range = g_c_ind + l_sharp_ind_range;
   if (l_sharp_ind_max_range > 32 * g_c_L_mult - 1)
      l_sharp_ind_max_range = 32 * g_c_L_mult - 1;
   if (g_c_ind > l_sharp_ind_range)
      l_sharp_ind_min_range = g_c_ind - l_sharp_ind_range;
   else
      l_sharp_ind_min_range = 0;
   g_c_ind = l_sharp_ind_min_range;
   set_ind(g_c_ind);
   get_swr();
   if (g_i_SWR == 0)
      return;
   l_sharp_ind_min_SWR = g_i_SWR;
   for (l_sharp_ind_count = l_sharp_ind_min_range + g_c_L_mult; l_sharp_ind_count <= l_sharp_ind_max_range; l_sharp_ind_count += g_c_L_mult)
   {
      set_ind(l_sharp_ind_count);
      get_swr();
      if (g_i_SWR == 0)
         return;
      if (g_i_SWR >= l_sharp_ind_min_SWR)
      {
         Delay_ms(10);
         get_swr();
      }
      if (g_i_SWR >= l_sharp_ind_min_SWR)
      {
         Delay_ms(10);
         get_swr();
      }
      if (g_i_SWR < l_sharp_ind_min_SWR)
      {
         l_sharp_ind_min_SWR = g_i_SWR;
         g_c_ind = l_sharp_ind_count;
         if (g_i_SWR < 120)
            break;
      }
      else
         break;
   }
   set_ind(g_c_ind);
   return;
}

void sub_tune()
{
   int l_int_swr_mem;
   unsigned char l_int_ind_mem, l_int_cap_mem;
   //
   l_int_swr_mem = g_i_SWR;
   /* pass 1 — SW at current position */
   coarse_tune();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   sharp_ind();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   sharp_cap();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   //
   if (g_i_SWR < 200 && g_i_SWR < l_int_swr_mem && (l_int_swr_mem - g_i_SWR) > 100)
      return;
   l_int_swr_mem = g_i_SWR;
   l_int_ind_mem = g_c_ind;
   l_int_cap_mem = g_c_cap;
   //
   if (g_c_SW == 1)
      g_c_SW = 0;
   else
      g_c_SW = 1;
   atu_reset();
   set_sw(g_c_SW);
   Delay_ms(50);
   get_swr();
   if (g_i_SWR < 120)
      return;
   /* pass 2 — SW at opposite position */
   coarse_tune();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   sharp_ind();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   sharp_cap();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   //
   if (g_i_SWR > l_int_swr_mem)
   {
      if (g_c_SW == 1)
         g_c_SW = 0;
      else
         g_c_SW = 1;
      set_sw(g_c_SW);
      g_c_ind = l_int_ind_mem;
      g_c_cap = l_int_cap_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      get_swr();
   }
   //
   CLRWDT();
   return;
}

static unsigned int measure_freq(void) {
   return 0;   /* replaced when 74HC4060 hardware is wired to RD0 */
}

/* Band lo/hi in kHz: 160m 80m 40m 30m 20m 17m 15m 12m 10m 6m */
static const unsigned int BAND_LO[10] = {1800, 3500, 7000, 10100, 14000, 18068, 21000, 24890, 28000, 50000};
static const unsigned int BAND_HI[10] = {2000, 4000, 7300, 10150, 14350, 18168, 21450, 24990, 29700, 54000};

static unsigned char freq_to_band_idx(unsigned int kHz)
{
   unsigned char l_b;
   for (l_b = 0; l_b < (unsigned char)EEPROM_BAND_N; l_b++)
      if (kHz >= BAND_LO[l_b] && kHz <= BAND_HI[l_b]) return l_b;
   return 0xFF;
}

static void band_slot_save(char l_probe_matched, unsigned int l_freq_kHz)
{
   unsigned char l_band, l_sub, l_slot_idx, l_base, l_ind, l_sw_swr, l_swr10;
   unsigned int  l_sf, l_diff;
   unsigned char l_existing_sub, l_empty_sub, l_worst_sub, l_worst_swr;
   if (l_probe_matched) return;
   if (g_i_SWR == 0 || g_i_SWR >= 300) return;
   if (l_freq_kHz == 0) return;
   l_band = freq_to_band_idx(l_freq_kHz);
   if (l_band == 0xFF) return;
   l_existing_sub = 0xFF;
   l_empty_sub    = 0xFF;
   l_worst_sub    = 0;
   l_worst_swr    = 0;
   for (l_sub = 0; l_sub < (unsigned char)EEPROM_BAND_SUB_N; l_sub++) {
      l_slot_idx = (unsigned char)(l_band * (unsigned char)EEPROM_BAND_SUB_N + l_sub);
      l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
      l_ind = eeprom_read(l_base + EEPROM_SLOT_IND);
      if (l_ind == 0xFF) { if (l_empty_sub == 0xFF) l_empty_sub = l_sub; continue; }
      l_sf = (unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_LO)
           | ((unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_HI) << 8);
      if (l_sf == 0) { if (l_empty_sub == 0xFF) l_empty_sub = l_sub; continue; }
      l_diff = (l_sf > l_freq_kHz) ? (unsigned int)(l_sf - l_freq_kHz)
                                    : (unsigned int)(l_freq_kHz - l_sf);
      if (l_diff <= (unsigned int)EEPROM_BAND_FREQ_TOL_KHZ) { l_existing_sub = l_sub; break; }
      l_sw_swr = eeprom_read(l_base + EEPROM_SLOT_SW_SWR);
      l_swr10  = (unsigned char)(l_sw_swr & 0x7Fu);
      if (l_swr10 > l_worst_swr) { l_worst_swr = l_swr10; l_worst_sub = l_sub; }
   }
   if (l_existing_sub != 0xFF)   l_sub = l_existing_sub;
   else if (l_empty_sub != 0xFF) l_sub = l_empty_sub;
   else                          l_sub = l_worst_sub;
   l_slot_idx = (unsigned char)(l_band * (unsigned char)EEPROM_BAND_SUB_N + l_sub);
   l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
   l_swr10 = (unsigned char)(g_i_SWR / 10);
   if (l_swr10 > 127) l_swr10 = 127;
   /* write IND last — it doubles as the commit byte (0xFF = empty sentinel).
      If power fails before IND is written, the slot stays empty and is safe to reuse. */
   eeprom_write(l_base + EEPROM_SLOT_FREQ_LO, (unsigned char)(l_freq_kHz & 0xFFu));
   eeprom_write(l_base + EEPROM_SLOT_FREQ_HI, (unsigned char)(l_freq_kHz >> 8));
   eeprom_write(l_base + EEPROM_SLOT_CAP,     (unsigned char)g_c_cap);
   eeprom_write(l_base + EEPROM_SLOT_SW_SWR,  (unsigned char)(((unsigned char)(g_c_SW & 1u) << 7) | l_swr10));
   eeprom_write(l_base + EEPROM_SLOT_IND,     (unsigned char)g_c_ind);
   g_b_slot_saved = 1;
}

void tune()
{
   unsigned char l_tune_ind_mem, l_tune_cap_mem;
   char l_tune_sw_mem;
   unsigned int l_freq_kHz;
   char l_probe_matched = 0;
   CLRWDT();
   //
   g_char_p_cnt = 0;
   g_i_P_max = 0;
   g_char_tune_effort = 0;
   g_b_slot_saved = 0;
   g_c_tune_exit  = 0;
   //
   g_b_rready = 0;
   g_b_tx_seen = 0;
   l_tune_ind_mem = g_c_ind;
   l_tune_cap_mem = g_c_cap;
   l_tune_sw_mem  = g_c_SW;
   l_freq_kHz = measure_freq();
#ifdef UART
   if (l_freq_kHz == 0 && g_i_uart_freq_hint != 0) { l_freq_kHz = g_i_uart_freq_hint; g_i_uart_freq_hint = 0; }
#endif
   get_swr();
   if (g_i_SWR < 110)
   {
      g_c_tune_exit = 1;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }
   /* probe band memory: search the 3 sub-slots for the current band */
   if (l_freq_kHz > 0)
   {
      unsigned char l_band_idx = freq_to_band_idx(l_freq_kHz);
      if (l_band_idx != 0xFF)
      {
         unsigned char l_sub, l_slot_idx, l_base, l_ind, l_sw_swr;
         unsigned int  l_sf, l_diff;
         for (l_sub = 0; l_sub < (unsigned char)EEPROM_BAND_SUB_N; l_sub++)
         {
            l_slot_idx = (unsigned char)(l_band_idx * (unsigned char)EEPROM_BAND_SUB_N + l_sub);
            l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
            l_ind = eeprom_read(l_base + EEPROM_SLOT_IND);
            if (l_ind == 0xFF) continue;
            l_sf = (unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_LO)
                 | ((unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_HI) << 8);
            if (l_sf == 0) continue;
            l_diff = (l_sf > l_freq_kHz) ? (unsigned int)(l_sf - l_freq_kHz)
                                          : (unsigned int)(l_freq_kHz - l_sf);
            if (l_diff > (unsigned int)EEPROM_BAND_FREQ_TOL_KHZ) continue;
            g_c_ind = l_ind;
            g_c_cap = eeprom_read(l_base + EEPROM_SLOT_CAP);
            l_sw_swr = eeprom_read(l_base + EEPROM_SLOT_SW_SWR);
            g_c_SW = (char)((l_sw_swr >> 7) & 1u);
            set_ind(g_c_ind);
            set_cap(g_c_cap);
            set_sw(g_c_SW);
            get_swr();
            if (g_i_SWR == 0)
            {
               g_c_tune_exit = 2;
               g_c_ind = l_tune_ind_mem; g_c_cap = l_tune_cap_mem; g_c_SW = l_tune_sw_mem;
               set_ind(g_c_ind); set_cap(g_c_cap); set_sw(g_c_SW);
               return;
            }
            if (g_i_SWR < 150)
            {
               /* upsert: overwrite stored SWR only when current measurement improved */
               unsigned char l_stored_swr10 = l_sw_swr & 0x7Fu;
               g_c_tune_exit = 3;
               if (g_i_SWR > 0 && (unsigned char)((unsigned int)g_i_SWR / 10u) < l_stored_swr10)
                  band_slot_save(0, l_freq_kHz);
               else
                  l_probe_matched = 1;
               return;
            }
         }
         /* no sub-slot matched — reset SW before full tune */
         g_c_SW = 0;
         set_sw(g_c_SW);
      }
   }
   g_char_tune_effort = 0;
   atu_reset();
   if (e_c_b_Loss_ind == 0)
      lcd_ind();
   Delay_ms(50);
   get_swr();
   g_i_swr_a = g_i_SWR;
   if (g_i_SWR == 0)
   {
      g_c_tune_exit = 4;
      g_c_ind = l_tune_ind_mem;
      g_c_cap = l_tune_cap_mem;
      g_c_SW  = l_tune_sw_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
      return;
   }
   if (g_i_SWR < 110)
   {
      g_c_tune_exit = 5;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }
   if (e_i_tenths_init_max_swr > 110 && g_i_SWR > e_i_tenths_init_max_swr)
   { g_c_tune_exit = 6; return; }
   //
   sub_tune();
   if (g_i_SWR == 0)
   {
      g_c_tune_exit = 7;
      g_c_ind = l_tune_ind_mem;
      g_c_cap = l_tune_cap_mem;
      g_c_SW  = l_tune_sw_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
      return;
   }
   if (g_i_SWR < 120)
   {
      g_c_tune_exit = 8;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }
   if (e_c_num_C_q == 5 && e_c_num_L_q == 5)
   {
      g_c_tune_exit = 9;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }

   if (e_c_num_L_q > 5)
   {
      g_c_step_ind = g_c_L_mult;
      g_c_L_mult = 1;
      sharp_ind();
   }
   if (g_i_SWR == 0)
   {
      g_c_tune_exit = 10;
      g_c_ind = l_tune_ind_mem;
      g_c_cap = l_tune_cap_mem;
      g_c_SW  = l_tune_sw_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
      return;
   }
   if (g_i_SWR < 120)
   {
      g_c_tune_exit = 11;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }
   if (e_c_num_C_q > 5)
   {
      g_c_step_cap = g_c_C_mult; // = g_c_C_mult
      g_c_C_mult = 1;
      sharp_cap();
   }
   if (g_i_SWR == 0)
   {
      g_c_tune_exit = 12;
      g_c_ind = l_tune_ind_mem;
      g_c_cap = l_tune_cap_mem;
      g_c_SW  = l_tune_sw_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
      return;
   }
   if (e_c_num_L_q == 5)
      g_c_L_mult = 1;
   else if (e_c_num_L_q == 6)
      g_c_L_mult = 2;
   else if (e_c_num_L_q == 7)
      g_c_L_mult = 4;
   if (e_c_num_C_q == 5)
      g_c_C_mult = 1;
   else if (e_c_num_C_q == 6)
      g_c_C_mult = 2;
   else if (e_c_num_C_q == 7)
      g_c_C_mult = 4;
   get_swr();
   g_c_tune_exit = 13;
   band_slot_save(l_probe_matched, l_freq_kHz);
   CLRWDT();
   return;
}
