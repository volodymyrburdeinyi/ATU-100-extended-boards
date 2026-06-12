//   ATU-100 project
//   David Fainitski
//   2016

#include "cross_compiler.h"
#include "oled_control.h"

#include "main.h"

/*  a few constants */

#define DEFAULT_INITIAL_OLD_VALUE  10000

#define DYSP_CNT_MULT 2.3

// Variables
int g_i_SWR_fixed_old = 0;
char g_work_str[7], g_work_str_2[7];
int g_i_Power = 0, g_i_Power_old = DEFAULT_INITIAL_OLD_VALUE;
int g_i_SWR_old = DEFAULT_INITIAL_OLD_VALUE;
char e_c_ledtype, g_b_Soft_tune = 0;
char g_b_Auto_mode = 0;

char g_b_Bypas_mode = 0;
char g_c_cap_mem = 0, g_c_ind_mem = 0, g_c_SW_mem = 0, g_c_Auto_mem = 0;

char g_b_Restart = 0;
char g_b_Test_mode = 0;
char g_b_lcd_prep_short = 0;
char g_b_L = 1, g_b_but = 0;





int g_i_dysp_delay_counter = 0;
int c_int_dysp_delay_time = 0;

char g_b_Loss_mode = 0;
char g_b_display_onoff = 1;
char g_b_tune_btn_released;

/*  initial eeprom values*/
__eeprom unsigned char initial_eeprom[256] = {
    0x78,0x05,0x01,0x15,0x13,0x01,0x00,0x00,0x02,0x00,0x07,0x00,0x07,0x00,0x01,0x00,
    0x00,0x50,0x01,0x10,0x02,0x20,0x04,0x50,0x10,0x00,0x22,0x00,0x45,0x00,0xff,0xff,
    0x00,0x10,0x00,0x22,0x00,0x47,0x01,0x00,0x02,0x20,0x04,0x70,0x10,0x00,0xff,0xff,
    0x00,0x10,0x00,0x00,0x00,0x00,0x08,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0x01,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0x01,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
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
   Soft_I2C_Init();
   if (e_c_ledtype == 0)
   { // 2-colors led  reset  turn both off
      GREEN_LED = LED_OFF;
      RED_LED = LED_OFF;
   }
   g_i_dysp_delay_counter = c_int_dysp_delay_time;
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

   led_init();

   /*   if FAST TEST mode, then turn on all relays, and loop here forever */
   if ((PORTB_AUTO_BUTTON == BUTTON_PRESSED) && 
           (PORTB_BYPASS_BUTTON == BUTTON_PRESSED) && 
           (PORTB_TUNE_BUTTON == BUTTON_PRESSED))
   { // Fast g_b_Test_mode mode (loop)
      if (e_c_ledtype == 4 | e_c_ledtype == 5)
         led_wr_str(0, 12, "FAST TEST", 9); // 128*64
      else if (e_c_ledtype != 0)
         led_wr_str(0, 3, "FAST TEST", 9); // 1602 | 128*32
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
   /*  feeder loss mode */
   if (Button(&PORTB, TUNE_BUTTON, 100, BUTTON_PRESSED))
   { //  Fider loss input
      if (e_c_ledtype == 4 | e_c_ledtype == 5)
      {
         led_wr_str(0, 6, "Fider Loss", 10); // 128*64
         led_wr_str(2, 6, "input", 5);
         led_wr_str(4, 6 + 3 * 12, "dB", 2);
      }
      else if (e_c_ledtype != 0)
      {
         led_wr_str(0, 0, "Fider Loss input", 16); // 1602 | 128*32
         led_wr_str(1, 3, "dB", 2);
      }
      e_c_tenths_Fid_loss = Bcd2Dec(eeprom_read(EEPROM_FEEDER_LOSS));
      show_loss();
      //
      while (1)
      {
         if (Button(&PORTB, BYPASS_BUTTON, 50, BUTTON_PRESSED))
         { // BYP button
            if (e_c_tenths_Fid_loss < 99)
            {
               e_c_tenths_Fid_loss++;
               show_loss();
               eeprom_write(EEPROM_FEEDER_LOSS, Dec2Bcd(e_c_tenths_Fid_loss));
            }
            while (Button(&PORTB, BYPASS_BUTTON, 50, BUTTON_PRESSED))
               CLRWDT();
         }
         //
         if (Button(&PORTB, AUTO_BUTTON, 50, BUTTON_PRESSED))
         { // AUTO button
            if (e_c_tenths_Fid_loss > 0)
            {
               e_c_tenths_Fid_loss--;
               show_loss();
               eeprom_write(EEPROM_FEEDER_LOSS, Dec2Bcd(e_c_tenths_Fid_loss));
            }
            while (Button(&PORTB, AUTO_BUTTON, 50, BUTTON_PRESSED))
               CLRWDT();
         }
         CLRWDT();
      } // while
   }    //  Fider loss input
   /*  end of feeder input loss code */
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
      if (g_b_Restart == 1)
         g_b_lcd_prep_short = 1;
      lcd_prep();
   }
   else
      Test_init();

   lcd_ind();
   g_b_tune_btn_released = 1;

   //*******************************

   while (1)
   {
      CLRWDT();
      lcd_pwr();
      //
      if (g_b_Test_mode == 0)
         button_proc();
      else
         button_proc_test();
#ifdef UART
      uart_cmd_proc();
#endif
      //
      if (g_i_dysp_delay_counter != 0)
         g_i_dysp_delay_counter--;
      else if (g_b_Test_mode == 0 & e_c_b_Dysp_delay != 0 & g_b_display_onoff == 1)
      {
         if (e_c_b_Relay_off)
         {
            set_ind(0);
            set_cap(0);
            set_sw(0);
         }
         g_b_display_onoff = 0;
         dysp_off();
      }

      // next While code
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
         lcd_ind();
      }
      else
      { // long press button
         if (g_b_L == 1)
            g_b_L = 0;
         else
            g_b_L = 1;
         if (g_b_L == 1)
         {
            if (e_c_ledtype == 4 | e_c_ledtype == 5) // 128*64 OLED
               led_wr_str(0, 16 + 12 * 8, "l", 1);
            else if (e_c_ledtype != 0) // 1602 LCD & 128*32 OLED
               led_wr_str(0, 8, "l", 1);
         }
         else
         {
            if (e_c_ledtype == 4 | e_c_ledtype == 5) // 128*64 OLED
               led_wr_str(0, 16 + 12 * 8, "c", 1);
            else if (e_c_ledtype != 0) // 1602 LCD & 128*32 OLED
               led_wr_str(0, 8, "c", 1);
         }
      }
      while (Button(&PORTB, TUNE_BUTTON, 50, BUTTON_PRESSED))
      {
         lcd_pwr();
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
         lcd_ind();
         lcd_pwr();
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
         lcd_ind();
         lcd_pwr();
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
   if ((g_b_tune_btn_released == 0) & Button(&PORTB, TUNE_BUTTON, 50, BUTTON_PRESSED))
      g_i_dysp_delay_counter = c_int_dysp_delay_time;
   if ((Button(&PORTB, TUNE_BUTTON, 50, BUTTON_PRESSED) & g_b_tune_btn_released) | g_b_Soft_tune)
   {
      if (g_b_display_onoff == 0)
      { // dysplay ON
         if (e_c_b_Relay_off == 1)
         {
            set_ind(g_c_ind);
            set_cap(g_c_cap);
            set_sw(g_c_SW);
         }
         g_b_display_onoff = 1;
         dysp_on();
         g_i_dysp_delay_counter = c_int_dysp_delay_time;
         return;
      }
      g_i_dysp_delay_counter = c_int_dysp_delay_time;
      Delay_ms(250);
      CLRWDT();
      if (g_b_Soft_tune == 0 & PORTB_TUNE_BUTTON == BUTTON_RELEASED)
      { // short press button
         show_reset();
         g_b_Bypas_mode = 0;
      }
      else
      {                 // long press button
         p_Tx = 1;      //
         n_Tx = 0;      // TX request
         Delay_ms(250); //
         tune_btn_push();
         g_b_Bypas_mode = 0;
         g_b_Soft_tune = 0;
         g_b_tune_btn_released = 0;
      }
   }
   //
   if (Button(&PORTB, BYPASS_BUTTON, 50, BUTTON_PRESSED))
   { // BYP button
      if (g_b_display_onoff == 0)
      { // dysplay ON
         if (e_c_b_Relay_off == 1)
         {
            set_ind(g_c_ind);
            set_cap(g_c_cap);
            set_sw(g_c_SW);
         }
         g_b_display_onoff = 1;
         dysp_on();
         g_i_dysp_delay_counter = c_int_dysp_delay_time;
         return;
      }
      g_i_dysp_delay_counter = c_int_dysp_delay_time;
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
         if (g_b_Loss_mode == 0)
            lcd_ind();
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
         if (g_b_Loss_mode == 0)
            lcd_ind();
         g_b_Auto_mode = g_c_Auto_mem;
      }
      if (e_c_ledtype == 4 | e_c_ledtype == 5)
      { // 128*64 OLED
         if (g_b_Auto_mode & !g_b_Bypas_mode)
            led_wr_str(0, 16 + 8 * 12, ".", 1);
         else if ((!g_b_Auto_mode) & g_b_Bypas_mode)
            led_wr_str(0, 16 + 8 * 12, "_", 1);
         else
            led_wr_str(0, 16 + 8 * 12, " ", 1);
      }
      else if (e_c_ledtype != 0)
      { //  1602 LCD  or 128*32 OLED
         if (g_b_Auto_mode & !g_b_Bypas_mode)
            led_wr_str(0, 8, ".", 1);
         else if ((!g_b_Auto_mode) & g_b_Bypas_mode)
            led_wr_str(0, 8, "_", 1);
         else
            led_wr_str(0, 8, " ", 1);
      }
      CLRWDT();
      while (Button(&PORTB, BYPASS_BUTTON, 50, BUTTON_PRESSED))
      {
         lcd_pwr();
         CLRWDT();
      }
   }
   //
   if (Button(&PORTB, AUTO_BUTTON, 50, BUTTON_PRESSED) & (g_b_Bypas_mode == 0))
   { // g_b_Auto_mode button
      if (g_b_display_onoff == 0)
      { // dysplay ON
         if (e_c_b_Relay_off == 1)
         {
            set_ind(g_c_ind);
            set_cap(g_c_cap);
            set_sw(g_c_SW);
         }
         g_b_display_onoff = 1;
         dysp_on();
         g_i_dysp_delay_counter = c_int_dysp_delay_time;
         return;
      }
      g_i_dysp_delay_counter = c_int_dysp_delay_time;
      CLRWDT();
      if (g_b_Auto_mode == 0)
         g_b_Auto_mode = 1;
      else
         g_b_Auto_mode = 0;
      eeprom_write(EEPROM_AUTOMATIC_MODE, g_b_Auto_mode);
      if (e_c_ledtype == 4 | e_c_ledtype == 5)
      { // 128*64 OLED
         if (g_b_Auto_mode & !g_b_Bypas_mode)
            led_wr_str(0, 16 + 8 * 12, ".", 1);
         else if ((!g_b_Auto_mode) & g_b_Bypas_mode)
            led_wr_str(0, 16 + 8 * 12, "_", 1);
         else
            led_wr_str(0, 16 + 8 * 12, " ", 1);
      }
      else if (e_c_ledtype != 0)
      { //  1602 LCD  or 128*32 OLED
         if (g_b_Auto_mode & !g_b_Bypas_mode)
            led_wr_str(0, 8, ".", 1);
         else if ((!g_b_Auto_mode) & g_b_Bypas_mode)
            led_wr_str(0, 8, "_", 1);
         else
            led_wr_str(0, 8, " ", 1);
      }
      CLRWDT();
      while (Button(&PORTB, AUTO_BUTTON, 50, BUTTON_PRESSED))
      {
         lcd_pwr();
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
   lcd_ind();
   g_b_Loss_mode = 0;
   p_Tx = 0;
   n_Tx = 1;
   g_i_SWR = 0;
   g_i_PWR = 0;
   g_i_SWR_fixed_old = 0;
   if (e_c_ledtype == 4 | e_c_ledtype == 5)
   { // 128*64 OLED
      led_wr_str(2, 16, "RESET   ", 8);
      CLRWDT();
      Delay_ms(600);
      led_wr_str(2, 16, "SWR=0.00", 8);
      CLRWDT();
   }
   else if (e_c_ledtype != 0)
   { // 1602 LCD & 128*32 OLED
      led_wr_str(1, 0, "RESET   ", 8);
      CLRWDT();
      Delay_ms(600);
      led_wr_str(1, 0, "SWR=0.00", 8);
      CLRWDT();
   }
   else
   {
      GREEN_LED = LED_OFF;   /* turn off both LED's */
      RED_LED = LED_OFF;
   }
   g_i_SWR_old = DEFAULT_INITIAL_OLD_VALUE;
   g_i_Power_old = DEFAULT_INITIAL_OLD_VALUE;
   lcd_pwr();
   return;
}

void tune_btn_push()
{
   CLRWDT();
   if (e_c_ledtype == 4 | e_c_ledtype == 5)
   { // 128*64 OLED
      led_wr_str(2, 16 + 12 * 4, "TUNE", 4);
   }
   else if (e_c_ledtype != 0)
   { // 1602 LCD & 128*32 OLED
      led_wr_str(1, 4, "TUNE", 4);
   }
   else
   {
      GREEN_LED = LED_OFF;  /* turn off both LED's */
      RED_LED = LED_OFF;
   }
   tune();
   if (e_c_ledtype == 0)
   { // real-time 2-colors led work
      if (g_i_SWR <= 150)
      {
         GREEN_LED = LED_ON;  /* Is SWR < 1.5 then turn on green */
         RED_LED = LED_OFF;
      } // Green
      else if (g_i_SWR <= 250)
      {
         GREEN_LED = LED_ON;  /* Is SWR < 2.5 then turn on both = orange */
         RED_LED = LED_ON;
      } // Orange
      else
      {
         GREEN_LED = LED_OFF;
         RED_LED = LED_ON;  /* Is SWR > 2.5 then turn on red */
      } // Red
   }
   else if (g_b_Loss_mode == 0 | e_c_b_Loss_ind == 0)
      lcd_ind();
   eeprom_write(EEPROM_LAST_CAP, g_c_cap);
   eeprom_write(EEPROM_LAST_IND, g_c_ind);
   eeprom_write(EEPROM_LAST_SW, g_c_SW);
   eeprom_write(EEPROM_LAST_SWR_H, (char)(g_i_swr_a / 256));
   eeprom_write(EEPROM_LAST_SWR_L, (char)(g_i_swr_a % 256));
   g_i_SWR_old = DEFAULT_INITIAL_OLD_VALUE;
   g_i_Power_old = DEFAULT_INITIAL_OLD_VALUE;
   lcd_pwr();
   g_i_SWR_fixed_old = g_i_SWR;
   p_Tx = 0;
   n_Tx = 1;
   CLRWDT();
   return;
}

void lcd_prep()
{
   CLRWDT();
   if (e_c_ledtype == 4 | e_c_ledtype == 5)
   { // 128*64 OLED
      if (g_b_lcd_prep_short == 0)
      {
//  it appears the uart needs some chars to get synced up
#ifdef UART
         led_wr_str(0, 22, "       ", 7);
#endif
         led_wr_str(0, 22, "ATU-100", 7);
         led_wr_str(2, 6, "EXT board", 9);
         led_wr_str(4, 16, "by N7DDC", 8);
         led_wr_str(6, 4, "FW ver 3.2", 10);
         CLRWDT();
         Delay_ms(600);
         CLRWDT();
         Delay_ms(500);
         CLRWDT();
         led_wr_str(0, 16, "        ", 8);
         led_wr_str(2, 4, "          ", 10);
         led_wr_str(4, 16, "        ", 8);
         led_wr_str(6, 4, "          ", 10);
      }
      Delay_ms(150);
      if (e_c_b_P_High == 1)
         led_wr_str(0, 16, "PWR=  0W", 8);
      else
         led_wr_str(0, 16, "PWR=0.0W", 8);
      led_wr_str(2, 16, "SWR=0.00", 8);
      if (g_b_Auto_mode)
         led_wr_str(0, 16 + 8 * 12, ".", 1);
   }
   else if (e_c_ledtype != 0)
   { // 1602 LCD & 128*32 OLED
      if (g_b_lcd_prep_short == 0)
      {
         led_wr_str(0, 4, "ATU-100", 7);
         led_wr_str(1, 3, "EXT board", 9);
         CLRWDT();
         Delay_ms(700);
         CLRWDT();
         Delay_ms(500);
         CLRWDT();
         led_wr_str(0, 4, "by N7DDC", 8);
         led_wr_str(1, 3, "FW ver 3.2", 10);
         CLRWDT();
         Delay_ms(600);
         CLRWDT();
         Delay_ms(500);
         CLRWDT();
         led_wr_str(0, 4, "        ", 8);
         led_wr_str(1, 3, "          ", 10);
      }
      Delay_ms(150);
      if (e_c_b_P_High == 1)
         led_wr_str(0, 0, "PWR=  0W", 8);
      else
         led_wr_str(0, 0, "PWR=0.0W", 8);
      led_wr_str(1, 0, "SWR=0.00", 8);
      if (g_b_Auto_mode)
         led_wr_str(0, 8, ".", 1);
   }
   CLRWDT();
   lcd_ind();
   return;
}

void lcd_swr(int swr)
{
   CLRWDT();
   if (swr != g_i_SWR_old)
   {
      g_i_SWR_old = swr;
      if (swr == 0)
      { // Low power
         if (e_c_ledtype == 4 | e_c_ledtype == 5)
            led_wr_str(2, 16 + 4 * 12, "0.00", 4); // 128*64 OLED
         else if (e_c_ledtype != 0)
            led_wr_str(1, 4, "0.00", 4); // 1602  & 128*32 OLED
         else if (e_c_ledtype == 0)
         { // real-time 2-colors led work
            GREEN_LED = LED_OFF;  /* turn off both leds */
            RED_LED = LED_OFF;
         }
         g_i_SWR_old = 0;
      }
      else
      {
         IntToStr(swr, g_work_str);
         g_work_str_2[0] = g_work_str[3];
         g_work_str_2[1] = '.';
         g_work_str_2[2] = g_work_str[4];
         g_work_str_2[3] = g_work_str[5];
         if (e_c_ledtype == 4 | e_c_ledtype == 5)
            led_wr_str(2, 16 + 4 * 12, g_work_str_2, 4); // 128*64 OLED
         else if (e_c_ledtype != 0)
            led_wr_str(1, 4, g_work_str_2, 4); // 1602  & 128*32
         else if (e_c_ledtype == 0)
         { // real-time 2-colors led work
            if (swr <= 150)
            {
               GREEN_LED = LED_ON;  /* Is SWR < 1.5 then turn on green */
              RED_LED = LED_OFF;
            } // Green
            else if (swr <= 250)
            {
               GREEN_LED = LED_ON;/* Is SWR < 2.5 then turn on both */
               RED_LED = LED_ON;
            } // Orange
            else
            {
               GREEN_LED = LED_OFF;  /* Is SWR > 2.5 then turn on red */
               RED_LED = LED_ON;
            } // Red
         }
      }
   }
   CLRWDT();
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

void show_pwr(int parm_Power, int parm_SWR)
{
   int p_ant, eff;
   double a, b;
   a = 100;
   CLRWDT();
   //
   if (g_b_Test_mode == 0 & e_c_b_Loss_ind == 1 & parm_Power >= 10)
   {
      if (g_b_Loss_mode == 0)
      { // prepare
         if (e_c_ledtype == 4 | e_c_ledtype == 5)
         { // 128*64 OLED
            if (e_c_b_P_High == 1)
               led_wr_str(4, 16, "ANT=  0W", 8);
            else
               led_wr_str(4, 16, "ANT=0.0W", 8);
            led_wr_str(6, 16, "EFF=  0%", 8);
         }
         else if (e_c_ledtype == 2 | e_c_ledtype == 3)
         { // 128*32 OLED
            if (e_c_b_P_High == 1)
               led_wr_str(0, 9, "ANT=  0W", 8);
            else
               led_wr_str(0, 9, "ANT=0.0W", 8);
            led_wr_str(1, 9, "EFF=  0%", 8);
         }
         else if (e_c_ledtype == 1)
         { // 1602 LCD
            if (e_c_b_P_High == 1)
               led_wr_str(0, 9, "AN=  0W", 7);
            else
               led_wr_str(0, 9, "AN=0.0W", 7);
            led_wr_str(1, 9, "EFF= 0%", 7);
         }
      }
      g_b_Loss_mode = 1;
   }
   else
   {
      if (g_b_Loss_mode == 1)
         lcd_ind();
      g_b_Loss_mode = 0;
   }
   CLRWDT();
   if (parm_Power != g_i_Power_old)
   {
      g_i_Power_old = parm_Power;
      //
      if (e_c_b_P_High == 0)
      {
         if (parm_Power >= 100)
         {              // > 10 W
            parm_Power += 5; // rounding to 1 W
            IntToStr(parm_Power, g_work_str);
            g_work_str_2[0] = g_work_str[2];
            g_work_str_2[1] = g_work_str[3];
            g_work_str_2[2] = g_work_str[4];
            g_work_str_2[3] = 'W';
         }
         else
         {
            IntToStr(parm_Power, g_work_str);
            if (g_work_str[4] != ' ')
               g_work_str_2[0] = g_work_str[4];
            else
               g_work_str_2[0] = '0';
            g_work_str_2[1] = '.';
            if (g_work_str[5] != ' ')
               g_work_str_2[2] = g_work_str[5];
            else
               g_work_str_2[2] = '0';
            g_work_str_2[3] = 'W';
         }
      }
      else
      { // High g_i_Power
         if (parm_Power < 999)
         { // 0 - 999 Watt
            IntToStr(parm_Power, g_work_str);
            g_work_str_2[0] = g_work_str[3];
            g_work_str_2[1] = g_work_str[4];
            g_work_str_2[2] = g_work_str[5];
            g_work_str_2[3] = 'W';
         }
         else
         {
            IntToStr(parm_Power, g_work_str);
            g_work_str_2[0] = g_work_str[2];
            g_work_str_2[1] = g_work_str[3];
            g_work_str_2[2] = g_work_str[4];
            g_work_str_2[3] = g_work_str[5];
         }
      }
      if (e_c_ledtype == 4 | e_c_ledtype == 5)
         led_wr_str(0, 16 + 4 * 12, g_work_str_2, 4); // 128*64 OLED
      else if (e_c_ledtype != 0)
         led_wr_str(0, 4, g_work_str_2, 4); // 1602  & 128*32
      //
      CLRWDT();
      //  Loss indication
      if (g_b_Loss_mode == 1)
      {
         if (g_c_ind == 0 & g_c_cap == 0)
            g_i_swr_a = parm_SWR;
         a = 1.0 / ((g_i_swr_a / 100.0 + 100.0 / g_i_swr_a) * e_c_tenths_Fid_loss / 10.0 * 0.115 + 1.0); // Fider loss
         b = 4.0 / (2.0 + parm_SWR / 100.0 + 100.0 / parm_SWR);                         // parm_SWR loss
         if (a >= 1.0)
            a = 1.0;
         if (b >= 1.0)
            b = 1.0;
         p_ant = (int)(parm_Power * a * b);
         eff = (int)(a * b * 100);
         if (eff >= 100)
            eff = 99;
         //
         if (e_c_b_P_High == 0)
         {
            if (p_ant >= 100)
            {              // > 10 W
               p_ant += 5; // rounding to 1 W
               IntToStr(p_ant, g_work_str);
               g_work_str_2[0] = g_work_str[2];
               g_work_str_2[1] = g_work_str[3];
               g_work_str_2[2] = g_work_str[4];
               g_work_str_2[3] = 'W';
            }
            else
            {
               IntToStr(p_ant, g_work_str);
               if (g_work_str[4] != ' ')
                  g_work_str_2[0] = g_work_str[4];
               else
                  g_work_str_2[0] = '0';
               g_work_str_2[1] = '.';
               if (g_work_str[5] != ' ')
                  g_work_str_2[2] = g_work_str[5];
               else
                  g_work_str_2[2] = '0';
               g_work_str_2[3] = 'W';
            }
         }
         else
         { // High g_i_Power
            if (p_ant < 999)
            { // 0 - 1500 Watts
               IntToStr(p_ant, g_work_str);
               g_work_str_2[0] = g_work_str[3];
               g_work_str_2[1] = g_work_str[4];
               g_work_str_2[2] = g_work_str[5];
               g_work_str_2[3] = 'W';
            }
            else
            {
               IntToStr(p_ant, g_work_str);
               g_work_str_2[0] = g_work_str[2];
               g_work_str_2[1] = g_work_str[3];
               g_work_str_2[2] = g_work_str[4];
               g_work_str_2[3] = g_work_str[5];
            }
         }
         if (e_c_ledtype == 4 | e_c_ledtype == 5)
            led_wr_str(4, 16 + 4 * 12, g_work_str_2, 4); // 128*64 OLED
         else if (e_c_ledtype == 2 | e_c_ledtype == 3)
            led_wr_str(0, 13, g_work_str_2, 4); // 128*32
         else if (e_c_ledtype != 0)
            led_wr_str(0, 12, g_work_str_2, 4); // 1602
         //
         IntToStr(eff, g_work_str);
         g_work_str_2[0] = g_work_str[4];
         g_work_str_2[1] = g_work_str[5];
         if (e_c_ledtype == 4 | e_c_ledtype == 5)
            led_wr_str(6, 16 + 5 * 12, g_work_str_2, 2);
         else if (e_c_ledtype == 2 | e_c_ledtype == 3)
            led_wr_str(1, 14, g_work_str_2, 2);
         else if (e_c_ledtype == 1)
            led_wr_str(1, 13, g_work_str_2, 2);
      }
   }
   CLRWDT();
   return;
}

void lcd_pwr()
{
   int p = 0;
   char peak_cnt;
   int delta = e_i_tenths_SWR_Auto_delta - 100;
   char cnt;
   int SWR_fixed;
   g_i_PWR = 0;
   CLRWDT();
   //
   if (g_b_display_onoff == 0 & e_c_b_Relay_off == 1)
   {
      get_pwr();
      if (g_i_PWR >= 10)
      {
         set_ind(g_c_ind);
         set_cap(g_c_cap);
         set_sw(g_c_SW);
         //
         g_b_display_onoff = 1;
         dysp_on(); // dysplay ON
         g_i_dysp_delay_counter = c_int_dysp_delay_time;
      }
      return;
   }
   // peak detector
   cnt = 120;
   for (peak_cnt = 0; peak_cnt < cnt; peak_cnt++)
   {
      if ((PORTB_AUTO_BUTTON == BUTTON_PRESSED) | (PORTB_BYPASS_BUTTON == BUTTON_PRESSED) | ((PORTB_TUNE_BUTTON == BUTTON_PRESSED) & g_b_tune_btn_released))
      {
         button_delay();
         if (g_b_but == 1)
         {
            g_b_but = 0;
            return;
         }
      }
      get_pwr();
      if (g_i_PWR > p)
      {
         p = g_i_PWR;
         SWR_fixed = g_i_SWR;
      }
      Delay_ms(3);
   }
   CLRWDT();
   if (p >= 100)
   {
      p = (p + 5) / 10;
      p *= 10;
   } // round to 1 W if more then 100 W
   g_i_Power = p;
   if (g_i_Power < 10)
      SWR_fixed = 0;
   lcd_swr(SWR_fixed);
   if (g_i_Power >= 10 & e_c_b_Dysp_delay > 0)
   {
      g_i_dysp_delay_counter = c_int_dysp_delay_time;
   }
   //
   if (g_b_Auto_mode & (SWR_fixed >= e_i_tenths_SWR_Auto_delta) & ((SWR_fixed > g_i_SWR_fixed_old & (SWR_fixed - g_i_SWR_fixed_old) > delta) | (SWR_fixed<g_i_SWR_fixed_old &(g_i_SWR_fixed_old - SWR_fixed)> delta) | g_i_SWR_fixed_old == 999))
      g_b_Soft_tune = 1;
   //
   if ((PORTB_AUTO_BUTTON == BUTTON_PRESSED) | (PORTB_BYPASS_BUTTON == BUTTON_PRESSED) | ((PORTB_TUNE_BUTTON == BUTTON_PRESSED) & g_b_tune_btn_released))
   {
      button_delay();
      if (g_b_but == 1)
      {
         g_b_but = 0;
         return;
      }
   } // Fast return if button pressed
   //
   show_pwr(g_i_Power, SWR_fixed);
   //
   if ((PORTB_AUTO_BUTTON == BUTTON_PRESSED) | (PORTB_BYPASS_BUTTON == BUTTON_PRESSED) |
           ((PORTB_TUNE_BUTTON == BUTTON_PRESSED) & g_b_tune_btn_released))
   {
      button_delay();
      if (g_b_but == 1)
      {
         g_b_but = 0;
         return;
      }
   }
   CLRWDT();
   if (g_b_Overload == 1)
   {
      if (e_c_ledtype == 4 | e_c_ledtype == 5)
      { // 128*64 OLED
         led_wr_str(2, 16, "        ", 8);
         Delay_ms(100);
         led_wr_str(2, 16, "OVERLOAD", 8);
         Delay_ms(500);
         CLRWDT();
         led_wr_str(2, 16, "        ", 8);
         Delay_ms(300);
         CLRWDT();
         led_wr_str(2, 16, "OVERLOAD", 8);
         Delay_ms(500);
         CLRWDT();
         led_wr_str(2, 16, "        ", 8);
         Delay_ms(300);
         CLRWDT();
         led_wr_str(2, 16, "OVERLOAD", 8);
         Delay_ms(500);
         CLRWDT();
         led_wr_str(2, 16, "        ", 8);
         Delay_ms(100);
         led_wr_str(2, 16, "SWR=    ", 8);
      }
      else if (e_c_ledtype != 0)
      { // 1602  & 128*32
         led_wr_str(1, 0, "        ", 8);
         Delay_ms(100);
         led_wr_str(1, 0, "OVERLOAD", 8);
         Delay_ms(500);
         CLRWDT();
         led_wr_str(1, 0, "        ", 8);
         Delay_ms(300);
         CLRWDT();
         led_wr_str(1, 0, "OVERLOAD", 8);
         Delay_ms(500);
         CLRWDT();
         led_wr_str(1, 0, "        ", 8);
         Delay_ms(300);
         CLRWDT();
         led_wr_str(1, 0, "OVERLOAD", 8);
         Delay_ms(500);
         CLRWDT();
         led_wr_str(1, 0, "        ", 8);
         Delay_ms(100);
         led_wr_str(1, 0, "SWR=    ", 8);
      }
      CLRWDT();
      g_i_SWR_old = DEFAULT_INITIAL_OLD_VALUE;
      lcd_swr(SWR_fixed);
   }
   return;
}

void lcd_ind()
{
   char l_column;
   int l_work_int;
   CLRWDT();
 /*  if (1)
   {  */
      charbits indbits;
      indbits.bytes = g_c_ind;
      l_work_int = 0;
      if (indbits.bits.B0)
         l_work_int += e_i_Ind1;
      if (indbits.bits.B1)
         l_work_int += e_i_Ind2;
      if (indbits.bits.B2)
         l_work_int += e_i_Ind3;
      if (indbits.bits.B3)
         l_work_int += e_i_Ind4;
      if (indbits.bits.B4)
         l_work_int += e_i_Ind5;
      if (indbits.bits.B5)
         l_work_int += e_i_Ind6;
      if (indbits.bits.B6)
         l_work_int += e_i_Ind7;
      if (l_work_int > 9999)
      {                  // more then 9999 nH
         l_work_int += 50; // round
         IntToStr(l_work_int, g_work_str);
         g_work_str_2[0] = g_work_str[1];
         g_work_str_2[1] = g_work_str[2];
         g_work_str_2[2] = '.';
         g_work_str_2[3] = g_work_str[3];
      }
      else
      {
         IntToStr(l_work_int, g_work_str);
         if (g_work_str[2] != ' ')
            g_work_str_2[0] = g_work_str[2];
         else
            g_work_str_2[0] = '0';
         g_work_str_2[1] = '.';
         if (g_work_str[3] != ' ')
            g_work_str_2[2] = g_work_str[3];
         else
            g_work_str_2[2] = '0';
         if (g_work_str[4] != ' ')
            g_work_str_2[3] = g_work_str[4];
         else
            g_work_str_2[3] = '0';
      }
      if (e_c_ledtype == 4 | e_c_ledtype == 5)
      { // 128*64 OLED
         if (g_c_SW == 1)
            l_column = 4;
         else
            l_column = 6;
         led_wr_str(l_column, 16, "L=", 2);
         led_wr_str(l_column, 16 + 6 * 12, "uH", 2);
         led_wr_str(l_column, 16 + 2 * 12, g_work_str_2, 4);
      }
      else if (e_c_ledtype == 2 | e_c_ledtype == 3)
      { // 128*32 OLED
         if (g_c_SW == 1)
            l_column = 0;
         else
            l_column = 1;
         led_wr_str(l_column, 9, "L=", 2);
         led_wr_str(l_column, 15, "uH", 2);
         led_wr_str(l_column, 11, g_work_str_2, 4);
      }
      else if (e_c_ledtype == 1)
      { //  1602 LCD
         if (g_c_SW == 1)
            l_column = 0;
         else
            l_column = 1;
         led_wr_str(l_column, 9, "L=", 2);
         led_wr_str(l_column, 15, "u", 1);
         led_wr_str(l_column, 11, g_work_str_2, 4);
      }
 /*  }  */
   CLRWDT();
 /*  if (1)
   {  */
      l_work_int = 0;
      charbits capbits;
      capbits.bytes = g_c_cap;
      if (capbits.bits.B0)
         l_work_int += e_i_Cap1;
      if (capbits.bits.B1)
         l_work_int += e_i_Cap2;
      if (capbits.bits.B2)
         l_work_int += e_i_Cap3;
      if (capbits.bits.B3)
         l_work_int += e_i_Cap4;
      if (capbits.bits.B4)
         l_work_int += e_i_Cap5;
      if (capbits.bits.B5)
         l_work_int += e_i_Cap6;
      if (capbits.bits.B6)
         l_work_int += e_i_Cap7;
      IntToStr(l_work_int, g_work_str);
      g_work_str_2[0] = g_work_str[2];
      g_work_str_2[1] = g_work_str[3];
      g_work_str_2[2] = g_work_str[4];
      g_work_str_2[3] = g_work_str[5];
      //
      if (e_c_ledtype == 4 | e_c_ledtype == 5)
      { // 128*64 OLED
         if (g_c_SW == 1)
            l_column = 6;
         else
            l_column = 4;
         led_wr_str(l_column, 16, "C=", 2);
         led_wr_str(l_column, 16 + 6 * 12, "pF", 2);
         led_wr_str(l_column, 16 + 2 * 12, g_work_str_2, 4);
      }
      else if (e_c_ledtype == 2 | e_c_ledtype == 3)
      { // 128*32 OLED
         if (g_c_SW == 1)
            l_column = 1;
         else
            l_column = 0;
         led_wr_str(l_column, 9, "C=", 2);
         led_wr_str(l_column, 15, "pF", 2);
         led_wr_str(l_column, 11, g_work_str_2, 4);
      }
      else if (e_c_ledtype == 1)
      { // 1602 LCD
         if (g_c_SW == 1)
            l_column = 1;
         else
            l_column = 0;
         led_wr_str(l_column, 9, "C=", 2);
         led_wr_str(l_column, 15, "p", 1);
         led_wr_str(l_column, 11, g_work_str_2, 4);
      }
/*   }  */
   CLRWDT();
   return;
}

void Test_init(void)
{                             // g_b_Test_mode mode
   if (e_c_ledtype == 4 | e_c_ledtype == 5) // 128*64 OLED
      led_wr_str(0, 10, "TEST MODE", 9);
   else if (e_c_ledtype != 0) // 1602 LCD  or 128*32 OLED
      led_wr_str(0, 3, "TEST MODE", 9);
   CLRWDT();
   Delay_ms(500);
   CLRWDT();
   Delay_ms(500);
   CLRWDT();
   Delay_ms(500);
   CLRWDT();
   Delay_ms(500);
   CLRWDT();
   if (e_c_ledtype == 4 | e_c_ledtype == 5) // 128*64 OLED
      led_wr_str(0, 10, "         ", 9);
   else if (e_c_ledtype != 0) // 1602 LCD  or 128*32 OLED
      led_wr_str(0, 3, "         ", 9);
   atu_reset();
   g_c_SW = 1; // C to OUT
   set_sw(g_c_SW);
   eeprom_write(EEPROM_LAST_CAP, g_c_cap);
   eeprom_write(EEPROM_LAST_IND, g_c_ind);
   eeprom_write(EEPROM_LAST_SW, g_c_SW);
   //
   if (e_c_ledtype == 4 | e_c_ledtype == 5) // 128*64 OLED
      led_wr_str(0, 16 + 12 * 8, "l", 1);
   else if (e_c_ledtype != 0) // 1602 LCD or 128*32 OLED
      led_wr_str(0, 8, "l", 1);
   //
   g_b_lcd_prep_short = 1;
   lcd_prep();
   return;
}

void cells_init(void)
{
   // Cells init
   CLRWDT();
   //oled_addr = eeprom_read(0); // address
   e_c_ledtype = eeprom_read(EEPROM_DISPLAY_TYPE); // e_c_ledtype of display
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
   e_c_b_Dysp_delay = Bcd2Dec(eeprom_read(EEPROM_DISPLAY_OFF_TIMER)); // Dysplay ON delay
   
   c_int_dysp_delay_time = (int)(e_c_b_Dysp_delay * DYSP_CNT_MULT);
     
   e_c_b_Loss_ind = eeprom_read(EEPROM_ADDITIONAL_INDICATION);
   e_c_tenths_Fid_loss = Bcd2Dec(eeprom_read(EEPROM_FEEDER_LOSS));
   e_c_b_Relay_off = Bcd2Dec(eeprom_read(EEPROM_DISABLE_RELAYS));
   /* guard against old 4-byte slot format after firmware upgrade without EEPROM reprogram */
   if (eeprom_read(EEPROM_FORMAT_VERSION) != 0x01)
   {
      unsigned char l_fi, l_fbase;
      for (l_fi = 0; l_fi < EEPROM_BAND_SLOT_COUNT; l_fi++) {
         l_fbase = (unsigned char)(EEPROM_BAND_SLOT_0 + l_fi * EEPROM_BAND_SLOT_STRIDE);
         eeprom_write((unsigned char)(l_fbase + 1), 0xFF);
      }
      eeprom_write(EEPROM_BAND_PTR, 0xFF);
      if (eeprom_read(EEPROM_BAND_COUNT) < 1 ||
          eeprom_read(EEPROM_BAND_COUNT) > EEPROM_BAND_SLOT_COUNT)
         eeprom_write(EEPROM_BAND_COUNT, EEPROM_BAND_SLOT_COUNT);
      eeprom_write(EEPROM_FORMAT_VERSION, 0x01);
   }
   CLRWDT();
   return;
}

void show_loss(void)
{
   IntToStr(e_c_tenths_Fid_loss, g_work_str);
   if (e_c_tenths_Fid_loss >= 10)
      g_work_str_2[0] = g_work_str[4];
   else
      g_work_str_2[0] = '0';
   g_work_str_2[1] = '.';
   g_work_str_2[2] = g_work_str[5];
   if (e_c_ledtype == 4 | e_c_ledtype == 5)
      led_wr_str(4, 6, g_work_str_2, 3); // 128*64
   else if (e_c_ledtype != 0)
      led_wr_str(1, 0, g_work_str_2, 3); // 1602 | 128*32
   return;
}

#ifdef UART

unsigned char g_c_uart_freq_hint = 0;

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

static void uart_send_status(void)
{
    uart_puts("IND=");   uart_putuint(g_c_ind);
    uart_puts(" CAP=");  uart_putuint(g_c_cap);
    uart_puts(" SW=");   uart_putuint(g_c_SW);
    uart_puts(" SWR=");  uart_putuint(g_i_SWR);
    uart_puts(" AUTO="); uart_putuint(g_b_Auto_mode);
    uart_puts(" SLOTS=");uart_putuint(eeprom_read(EEPROM_BAND_COUNT));
    uart_puts("\r\n");
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

/* Find a freq-tagged slot within EEPROM_BAND_FREQ_TOL of l_freq.
   If found: apply relays, measure SWR.
   Returns  1 if SWR < 150 (match good).
   Returns  0 if no matching slot.
   Returns -1 if SWR == 0 (TX absent — relays left at last-tried position, caller restores). */
static signed char band_slot_apply_freq(unsigned char l_freq)
{
    unsigned char l_slot, l_base, l_slot_ind, l_count, l_slot_freq, l_diff;
    if (l_freq == 0) return 0;
    l_count = eeprom_read(EEPROM_BAND_COUNT);
    if (l_count < 1 || l_count > EEPROM_BAND_SLOT_COUNT) return 0;
    for (l_slot = 0; l_slot < l_count; l_slot++) {
        l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot * EEPROM_BAND_SLOT_STRIDE);
        l_slot_ind = eeprom_read(l_base + 1);
        if (l_slot_ind == 0xFF) continue;
        l_slot_freq = eeprom_read(l_base);
        if (l_slot_freq == 0) continue;
        l_diff = (l_slot_freq > l_freq) ? (unsigned char)(l_slot_freq - l_freq)
                                        : (unsigned char)(l_freq - l_slot_freq);
        if (l_diff > EEPROM_BAND_FREQ_TOL) continue;
        g_c_ind = l_slot_ind;
        g_c_cap = eeprom_read(l_base + 2);
        g_c_SW  = (char)(eeprom_read(l_base + 3) & 1u);
        set_ind(g_c_ind);
        set_cap(g_c_cap);
        set_sw(g_c_SW);
        get_swr();
        if (g_i_SWR == 0) return -1;
        if (g_i_SWR < 150) return 1;
    }
    return 0;
}

static void uart_exec_cmd(const char *l_cmd, unsigned char l_len)
{
    if (l_cmd[0] == 't') {
        if (l_len >= 4u) g_c_uart_freq_hint = parse_hex8(l_cmd + 2);
        tune();
        uart_send_status();
    } else if (l_cmd[0] == 'l' && l_len >= 4u) {
        /* "l HH" — recall slot by freq_enc, apply relays, report result */
        unsigned char l_freq = parse_hex8(l_cmd + 2);
        char l_save_ind = g_c_ind, l_save_cap = g_c_cap, l_save_sw = g_c_SW;
        signed char l_result = band_slot_apply_freq(l_freq);
        if (l_result == 1) {
            uart_puts("RECALL IND="); uart_putuint(g_c_ind);
            uart_puts(" CAP=");       uart_putuint(g_c_cap);
            uart_puts(" SW=");        uart_putuint(g_c_SW);
            uart_puts(" SWR=");       uart_putuint(g_i_SWR);
            uart_puts("\r\n");
        } else if (l_result == 0) {
            uart_puts("NOMATCH\r\n");
        } else {
            g_c_ind = l_save_ind; g_c_cap = l_save_cap; g_c_SW = l_save_sw;
            set_ind(g_c_ind); set_cap(g_c_cap); set_sw(g_c_SW);
            uart_puts("ERR ABORT\r\n");
        }
    } else if (l_cmd[0] == 'm') {
        /* "m" — dump all band slots */
        unsigned char l_slot, l_base, l_ind, l_count;
        l_count = eeprom_read(EEPROM_BAND_COUNT);
        if (l_count < 1 || l_count > EEPROM_BAND_SLOT_COUNT) {
            uart_puts("SLOTS=0\r\n");
        } else {
            for (l_slot = 0; l_slot < l_count; l_slot++) {
                l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot * EEPROM_BAND_SLOT_STRIDE);
                l_ind = eeprom_read(l_base + 1);
                uart_puts("SLOT "); uart_putuint(l_slot);
                if (l_ind == 0xFF) {
                    uart_puts(" EMPTY\r\n");
                } else {
                    uart_puts(" FREQ=0x"); uart_puthex8(eeprom_read(l_base));
                    uart_puts(" IND=");    uart_putuint(l_ind);
                    uart_puts(" CAP=");    uart_putuint(eeprom_read(l_base + 2));
                    uart_puts(" SW=");     uart_putuint(eeprom_read(l_base + 3) & 1u);
                    uart_puts(" SWR=");    uart_putuint((int)eeprom_read(l_base + 4) * 10);
                    uart_puts("\r\n");
                }
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
        } else if (l_len < 11u) {
            l_buf[l_len++] = (char)l_c;
        }
    }
}

#endif /* UART */

