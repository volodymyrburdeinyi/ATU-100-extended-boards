#include "cross_compiler.h"
#include "globals.h"
#include "relay.h"

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
