//   ATU-100 project
//   David Fainitski
//   2016

#include "cross_compiler.h"

#include "main.h"
#include "uart_cmd.h"

// Variables
char g_b_Auto_mode = 0;

char g_b_Bypas_mode = 0;
char g_c_cap_mem = 0, g_c_ind_mem = 0, g_c_SW_mem = 0, g_c_Auto_mem = 0;

char g_b_Restart = 0;
char g_b_Test_mode = 0;
char g_b_L = 1, g_b_but = 0;

char g_b_tune_btn_released;

/*  initial eeprom values*/
__eeprom unsigned char initial_eeprom[256] = {
    0x78,0x05,0x01,0x15,0x13,0x01,0x00,0x00,0x02,0x00,0x07,0x00,0x07,0x00,0x01,0x00,
    0x00,0x50,0x01,0x10,0x02,0x20,0x04,0x50,0x10,0x00,0x22,0x00,0x45,0x00,0xff,0xff,
    0x00,0x10,0x00,0x22,0x00,0x47,0x01,0x00,0x02,0x20,0x04,0x70,0x10,0x00,0xff,0xff,
    0x00,0x10,0x00,0x00,0x00,0x00,0x02,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x01,0x00,0x00,
};

/* ISR is in uart.c when UART is defined */

void main()
{
#ifdef SIMULATOR
    /*  print in debug mode */
    char mystart[] = "Starting!";
    PRINTLINE(mystart)
    eeprom_write(EEPROM_DISPLAY_I2C_ADDR,0x4E);
    eeprom_write(EEPROM_DISPLAY_TYPE,4);
    eeprom_write(EEPROM_AUTOMATIC_MODE,0);
    eeprom_write(EEPROM_TIMEOUT_TIME,0x15);
    eeprom_write(EEPROM_SWR_THRESHOLD,0x13);
    eeprom_write(EEPROM_MIN_POWER,5);
    eeprom_write(EEPROM_MAX_POWER,0);
    eeprom_write(EEPROM_DISPLAY_OFFSET_DOWN,2);
    eeprom_write(EEPROM_DISPLAY_OFFSET_LEFT,3);
    eeprom_write(EEPROM_MAX_INIT_SWR,0);
    eeprom_write(EEPROM_NUMBER_INDS,7);
    eeprom_write(EEPROM_IND_LINEAR_PITCH,0);
    eeprom_write(EEPROM_NUMBER_CAPS,7);
    eeprom_write(EEPROM_CAP_LINEAR_PITCH,0);
    eeprom_write(EEPROM_ENABLE_NONLINEAR_DIODE,1);
    eeprom_write(EEPROM_INVERSE_INDUCTANCE_RELAY,0);

    eeprom_write(0x10,0);
    eeprom_write(0x11,0x50);
    eeprom_write(0x12,1);
    eeprom_write(0x13,0x10);
    eeprom_write(0x14,2);
    eeprom_write(0x15,0x20);
    eeprom_write(0x16,4);
    eeprom_write(0x17,0x50);
    eeprom_write(0x18,0x10);
    eeprom_write(0x19,0);
    eeprom_write(0x1a,0x22);
    eeprom_write(0x1b,0);
    eeprom_write(0x1c,0x45);
    eeprom_write(0x1d,0);

    eeprom_write(0x20,0);
    eeprom_write(0x21,0x10);
    eeprom_write(0x22,0);
    eeprom_write(0x23,0x22);
    eeprom_write(0x24,0);
    eeprom_write(0x25,0x47);
    eeprom_write(0x26,1);
    eeprom_write(0x27,0x00);
    eeprom_write(0x28,2);
    eeprom_write(0x29,0x20);
    eeprom_write(0x2a,4);
    eeprom_write(0x2b,0x70);
    eeprom_write(0x2c,0x10);
    eeprom_write(0x2d,0);


    eeprom_write(EEPROM_POWER_MEASURE_LEVEL,0);
    eeprom_write(EEPROM_TANDEM_MATCH,10);
    eeprom_write(EEPROM_DISPLAY_OFF_TIMER,0);
    eeprom_write(EEPROM_ADDITIONAL_INDICATION,1);
    eeprom_write(EEPROM_FEEDER_LOSS,0x12);
    eeprom_write(EEPROM_DISABLE_RELAYS,0x0);
    eeprom_write(EEPROM_LAST_SWR_L,0x18);
    eeprom_write(EEPROM_LAST_SWR_H,0);
    eeprom_write(EEPROM_LAST_SW,0);
    eeprom_write(EEPROM_LAST_IND,0);
    eeprom_write(EEPROM_LAST_CAP,0);

 #endif

   if (STATUSbits.nTO == 0)
      g_b_Restart = 1;
   pic_init();
   //
   Delay_ms(300);
   CLRWDT();
   cells_init();
   //
   Delay_ms(300);
   CLRWDT();

   //
   /*  test mode?   enter step by step adjustments */
   if (PORTB_AUTO_BUTTON == BUTTON_PRESSED & PORTB_BYPASS_BUTTON == BUTTON_PRESSED)
   { // g_b_Test_mode mode
      g_b_Test_mode = 1;
      g_b_Auto_mode = 0;
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

   Delay_ms(300);
   CLRWDT();
   Delay_ms(300);
   CLRWDT();
   Delay_ms(300);
   CLRWDT();
   Delay_ms(300);
   CLRWDT();
   Delay_ms(300);
   CLRWDT();

   /*   if FAST TEST mode, then turn on all relays, and loop here forever */
   if ((PORTB_AUTO_BUTTON == BUTTON_PRESSED) &&
           (PORTB_BYPASS_BUTTON == BUTTON_PRESSED) &&
           (PORTB_TUNE_BUTTON == BUTTON_PRESSED))
   { // Fast g_b_Test_mode mode (loop)
      set_cap(255);
      if (e_c_b_L_invert == 0)
         set_ind(255);
      else
         set_ind(0);
      set_sw(1);
      CLRWDT();
      while (1)
      {
         Delay_ms(500);
         CLRWDT();
      }
   }
   /*   end of FAST TEST code */
   CLRWDT();
   //
   if (g_b_Test_mode == 0)
   {
      g_c_cap = eeprom_read(EEPROM_LAST_CAP);
      g_c_ind = eeprom_read(EEPROM_LAST_IND);
      g_c_SW = eeprom_read(EEPROM_LAST_SW);
      g_i_swr_a = eeprom_read(EEPROM_LAST_SWR_H) * 256;
      g_i_swr_a += eeprom_read(EEPROM_LAST_SWR_L);
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
   }
   else
      Test_init();

   g_b_tune_btn_released = 1;

   //*******************************

   while (1)
   {
      CLRWDT();
      if (g_b_Test_mode == 0)
         button_proc();
      else
         button_proc_test();
#ifdef UART
      uart_cmd_proc();
#endif
   }
}

//***************** Routines *****************

void button_proc_test(void)
{
   if (Button(&PORTB, TUNE_BUTTON, 50, BUTTON_PRESSED))
   { // Tune btn
      Delay_ms(250);
      CLRWDT();
      if (PORTB_TUNE_BUTTON == BUTTON_RELEASED)
      { // short press button
         if (g_c_SW == 0)
            g_c_SW = 1;
         else
            g_c_SW = 0;
         set_sw(g_c_SW);
      }
      else
      { // long press button
         if (g_b_L == 1)
            g_b_L = 0;
         else
            g_b_L = 1;
      }
      while (Button(&PORTB, TUNE_BUTTON, 50, BUTTON_PRESSED))
      {
         CLRWDT();
      }
   } // END Tune btn
   //
   if (Button(&PORTB, BYPASS_BUTTON, 50, BUTTON_PRESSED))
   { // BYP button
      CLRWDT();
      while (PORTB_BYPASS_BUTTON == BUTTON_PRESSED)
      {
         if (g_b_L & (g_c_ind < 32 * g_c_L_mult - 1))
         {
            g_c_ind++;
            set_ind(g_c_ind);
         }
         else if (!g_b_L & (g_c_cap < 32 * g_c_L_mult - 1))
         {
            g_c_cap++;
            set_cap(g_c_cap);
         }
         Delay_ms(30);
         CLRWDT();
      }
   } // end of BYP button
   //
   if (Button(&PORTB, AUTO_BUTTON, 50, BUTTON_PRESSED) & (g_b_Bypas_mode == 0))
   { // g_b_Auto_mode button
      CLRWDT();
      while (PORTB_AUTO_BUTTON == BUTTON_PRESSED)
      {
         if (g_b_L & (g_c_ind > 0))
         {
            g_c_ind--;
            set_ind(g_c_ind);
         }
         else if (!g_b_L & (g_c_cap > 0))
         {
            g_c_cap--;
            set_cap(g_c_cap);
         }
         Delay_ms(30);
         CLRWDT();
      }
   }
   return;
}

void button_proc(void)
{
   if ((g_b_tune_btn_released == 0) & Button(&PORTB, TUNE_BUTTON, 50, BUTTON_RELEASED))
      g_b_tune_btn_released = 1;
   if (Button(&PORTB, TUNE_BUTTON, 50, BUTTON_PRESSED) & g_b_tune_btn_released)
   {
      Delay_ms(250);
      CLRWDT();
      if (PORTB_TUNE_BUTTON == BUTTON_RELEASED)
      { // short press button — relay reset
         atu_reset();
         g_c_SW = 1;
         set_sw(g_c_SW);
         eeprom_write(EEPROM_LAST_CAP, 0);
         eeprom_write(EEPROM_LAST_IND, 0);
         eeprom_write(EEPROM_LAST_SW, 1);
         eeprom_write(EEPROM_LAST_SWR_H, 0);
         eeprom_write(EEPROM_LAST_SWR_L, 0);
         p_Tx = 0;
         n_Tx = 1;
         g_i_SWR = 0;
         g_i_PWR = 0;
         g_b_Bypas_mode = 0;
      }
      else
      {                 // long press button
         p_Tx = 1;      //
         n_Tx = 0;      // TX request
         Delay_ms(250); //
         tune_btn_push();
         g_b_Bypas_mode = 0;
         g_b_tune_btn_released = 0;
      }
   }
   //
   if (Button(&PORTB, BYPASS_BUTTON, 50, BUTTON_PRESSED))
   { // BYP button
      CLRWDT();
      if (g_b_Bypas_mode == 0)
      {
         g_b_Bypas_mode = 1;
         g_c_cap_mem = g_c_cap;
         g_c_ind_mem = g_c_ind;
         g_c_SW_mem = g_c_SW;
         g_c_cap = 0;
         if (e_c_b_L_invert)
            g_c_ind = 255;
         else
            g_c_ind = 0;
         g_c_SW = 1;
         set_ind(g_c_ind);
         set_cap(g_c_cap);
         set_sw(g_c_SW);
         g_c_Auto_mem = g_b_Auto_mode;
         g_b_Auto_mode = 0;
      }
      else
      {
         g_b_Bypas_mode = 0;
         g_c_cap = g_c_cap_mem;
         g_c_ind = g_c_ind_mem;
         g_c_SW = g_c_SW_mem;
         set_cap(g_c_cap);
         set_ind(g_c_ind);
         set_sw(g_c_SW);
         g_b_Auto_mode = g_c_Auto_mem;
      }
      CLRWDT();
      while (Button(&PORTB, BYPASS_BUTTON, 50, BUTTON_PRESSED))
      {
         CLRWDT();
      }
   }
   //
   if (Button(&PORTB, AUTO_BUTTON, 50, BUTTON_PRESSED) & (g_b_Bypas_mode == 0))
   { // g_b_Auto_mode button
      CLRWDT();
      if (g_b_Auto_mode == 0)
         g_b_Auto_mode = 1;
      else
         g_b_Auto_mode = 0;
      eeprom_write(EEPROM_AUTOMATIC_MODE, g_b_Auto_mode);
      CLRWDT();
      while (Button(&PORTB, AUTO_BUTTON, 50, BUTTON_PRESSED))
      {
         CLRWDT();
      }
   }
   return;
}

void show_reset()
{
   atu_reset();
   g_c_SW = 1;
   set_sw(g_c_SW);
   eeprom_write(EEPROM_LAST_CAP, 0);
   eeprom_write(EEPROM_LAST_IND, 0);
   eeprom_write(EEPROM_LAST_SW, 1);
   eeprom_write(EEPROM_LAST_SWR_H, 0);
   eeprom_write(EEPROM_LAST_SWR_L, 0);
   p_Tx = 0;
   n_Tx = 1;
   g_i_SWR = 0;
   g_i_PWR = 0;
   return;
}

void tune_btn_push()
{
   CLRWDT();
   tune();
   eeprom_write(EEPROM_LAST_CAP, g_c_cap);
   eeprom_write(EEPROM_LAST_IND, g_c_ind);
   eeprom_write(EEPROM_LAST_SW, g_c_SW);
   eeprom_write(EEPROM_LAST_SWR_H, (char)(g_i_swr_a / 256));
   eeprom_write(EEPROM_LAST_SWR_L, (char)(g_i_swr_a % 256));
   p_Tx = 0;
   n_Tx = 1;
   CLRWDT();
   return;
}

/* lcd_ind: no-op stub — tune_algo.h calls this after relay changes;
 * headless station has no display so the call is retained for linker
 * compatibility without pulling in any display driver. */
void lcd_ind(void)
{
   return;
}

void button_delay()
{
   if ((Button(&PORTB, TUNE_BUTTON, 25, BUTTON_PRESSED)) | (Button(&PORTB, AUTO_BUTTON, 25, BUTTON_PRESSED)) | (Button(&PORTB, BYPASS_BUTTON, 25, BUTTON_PRESSED)))
   {
      g_b_but = 1;
   }
   return;
}

void Test_init(void)
{
   CLRWDT();
   atu_reset();
   g_c_SW = 1;
   set_sw(g_c_SW);
   eeprom_write(EEPROM_LAST_CAP, g_c_cap);
   eeprom_write(EEPROM_LAST_IND, g_c_ind);
   eeprom_write(EEPROM_LAST_SW, g_c_SW);
   return;
}

void cells_init(void)
{
   // Cells init
   CLRWDT();
   if (eeprom_read(EEPROM_AUTOMATIC_MODE) == 1)
      g_b_Auto_mode = 1;
   e_i_ms_Rel_Del = Bcd2Dec(eeprom_read(EEPROM_TIMEOUT_TIME));            // Relay's Delay
   e_i_tenths_SWR_Auto_delta = Bcd2Dec(eeprom_read(EEPROM_SWR_THRESHOLD)) * 10;    // e_i_tenths_SWR_Auto_delta
   e_i_watts_min_for_start = Bcd2Dec(eeprom_read(EEPROM_MIN_POWER)) * 10; // P_min_for_start
   e_i_watts_max_for_start = Bcd2Dec(eeprom_read(EEPROM_MAX_POWER)) * 10; // P_max_for_start
   // 7  - shift down
   // 8 - shift left
   e_i_tenths_init_max_swr = Bcd2Dec(eeprom_read(EEPROM_MAX_INIT_SWR)) * 10; // Max g_i_SWR
   e_c_num_L_q = eeprom_read(EEPROM_NUMBER_INDS);
   e_c_b_L_linear = eeprom_read(EEPROM_IND_LINEAR_PITCH);
   e_c_num_C_q = eeprom_read(EEPROM_NUMBER_CAPS);
   e_c_b_C_linear = eeprom_read(EEPROM_CAP_LINEAR_PITCH);
   e_c_b_D_correction = eeprom_read(EEPROM_ENABLE_NONLINEAR_DIODE);
   e_c_b_L_invert = eeprom_read(EEPROM_INVERSE_INDUCTANCE_RELAY);
   //
   CLRWDT();
   e_i_Ind1 = Bcd2Dec(eeprom_read(16)) * 100 + Bcd2Dec(eeprom_read(17)); // e_i_Ind1
   e_i_Ind2 = Bcd2Dec(eeprom_read(18)) * 100 + Bcd2Dec(eeprom_read(19)); // e_i_Ind2
   e_i_Ind3 = Bcd2Dec(eeprom_read(20)) * 100 + Bcd2Dec(eeprom_read(21)); // e_i_Ind3
   e_i_Ind4 = Bcd2Dec(eeprom_read(22)) * 100 + Bcd2Dec(eeprom_read(23)); // e_i_Ind4
   e_i_Ind5 = Bcd2Dec(eeprom_read(24)) * 100 + Bcd2Dec(eeprom_read(25)); // e_i_Ind5
   e_i_Ind6 = Bcd2Dec(eeprom_read(26)) * 100 + Bcd2Dec(eeprom_read(27)); // e_i_Ind6
   e_i_Ind7 = Bcd2Dec(eeprom_read(28)) * 100 + Bcd2Dec(eeprom_read(29)); // e_i_Ind7
   //
   e_i_Cap1 = Bcd2Dec(eeprom_read(32)) * 100 + Bcd2Dec(eeprom_read(33)); // e_i_Cap1
   e_i_Cap2 = Bcd2Dec(eeprom_read(34)) * 100 + Bcd2Dec(eeprom_read(35)); // e_i_Cap2
   e_i_Cap3 = Bcd2Dec(eeprom_read(36)) * 100 + Bcd2Dec(eeprom_read(37)); // e_i_Cap3
   e_i_Cap4 = Bcd2Dec(eeprom_read(38)) * 100 + Bcd2Dec(eeprom_read(39)); // e_i_Cap4
   e_i_Cap5 = Bcd2Dec(eeprom_read(40)) * 100 + Bcd2Dec(eeprom_read(41)); // e_i_Cap5
   e_i_Cap6 = Bcd2Dec(eeprom_read(42)) * 100 + Bcd2Dec(eeprom_read(43)); // e_i_Cap6
   e_i_Cap7 = Bcd2Dec(eeprom_read(44)) * 100 + Bcd2Dec(eeprom_read(45)); // e_i_Cap7
   //
   e_c_b_P_High = eeprom_read(EEPROM_POWER_MEASURE_LEVEL);              // High power
   e_c_K_Mult = Bcd2Dec(eeprom_read(EEPROM_TANDEM_MATCH));     // Tandem Match rate
   e_c_b_Relay_off = Bcd2Dec(eeprom_read(EEPROM_DISABLE_RELAYS));
   /* wipe band slots on format change (new firmware, new slot layout) */
   if (eeprom_read(EEPROM_BAND_FORMAT_CELL) != (unsigned char)EEPROM_BAND_FORMAT_VER)
   {
      unsigned char l_fi, l_fbase;
      for (l_fi = 0; l_fi < (unsigned char)EEPROM_BAND_SLOT_COUNT; l_fi++) {
         CLRWDT();
         l_fbase = EEPROM_BAND_SLOT_0 + (unsigned char)(l_fi * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
         eeprom_write(l_fbase,                              0xFF);
         eeprom_write((unsigned char)(l_fbase + 1),         0xFF);
         eeprom_write((unsigned char)(l_fbase + 2),         0xFF);
         eeprom_write((unsigned char)(l_fbase + 3),         0xFF);
         eeprom_write((unsigned char)(l_fbase + 4),         0xFF);
      }
      eeprom_write(EEPROM_BAND_FORMAT_CELL, (unsigned char)EEPROM_BAND_FORMAT_VER);
   }
   CLRWDT();
   return;
}

#ifdef UART

unsigned int  g_i_uart_freq_hint = 0;
unsigned char g_b_slot_saved     = 0;
unsigned char g_c_tune_exit      = 0;   /* set to exit-path code by tune() for diagnostics */

#endif /* UART */
