#include "cross_compiler.h"
#include "globals.h"
#include "swr.h"

/* show_reset() defined in main.c — called from get_swr() button-abort path */
void show_reset(void);

/* Upper bound on how long get_swr() will sit waiting for the transmitter to
 * key up. Each pass is one ADC set plus a 5 ms button debounce, ~8 ms, so
 * this is roughly ten seconds. Without it a tune requested while the radio
 * stays silent never returns, and since the main loop is what services UART,
 * the tuner cannot even be told to stop. */
#define PWR_WAIT_MAX_PASSES 1200u

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

/* Rolling peak-power window: sample until the window is full, then start over.
 * Kept in one place so the two call sites below cannot drift apart. */
static void track_peak_power(void)
{
   if (g_char_p_cnt != 100)
   {
      g_char_p_cnt += 1;
      if (g_i_PWR > g_i_P_max)
         g_i_P_max = g_i_PWR;
   }
   else
   {
      g_char_p_cnt = 0;
      g_i_P_max = 0;
   }
}

void get_swr()
{
   unsigned int l_wait_passes = 0;

   get_pwr();
   track_peak_power();
   if (g_char_tune_effort < 255)
      g_char_tune_effort++;
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
#ifdef UART
      /* This loop, not the relay scans, is where a tune actually sits waiting.
         The main loop is blocked while we are here, so poll the UART directly
         or 'q' cannot be acted on until the radio keys — which may be never. */
      uart_cmd_proc();
      if (g_b_tune_abort)
      {
         g_i_SWR = 0;
         return;
      }
#endif
      if (++l_wait_passes >= PWR_WAIT_MAX_PASSES)
      { // transmitter never keyed — give up rather than hang the main loop
         g_i_SWR = 0;
         return;
      }
      get_pwr();
      track_peak_power();
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
