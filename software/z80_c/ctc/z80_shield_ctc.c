/* 
 * Z80 Shield support for Z80 CTC
 *
 */

#include "z80_shield_ctc.h"

// Initialise a CTC

void z80_shield_ctc_channel_init(int ctc_channel, int control_word, int time_constant)
{
  volatile unsigned char *p_ctc = &IO_CTC1;
  
  switch(ctc_channel)
    {
    case 0:
      IO_CTC0 = control_word;
      if ( control_word & 0x04)
	{
	  IO_CTC0 = time_constant;
	}
      
      break;

    case 1:
      IO_CTC1 = control_word;
      if ( control_word & 0x04)
	{
	  IO_CTC1 = time_constant;
	}
      
      break;

    case 2:
      IO_CTC2 = control_word;
      if ( control_word & 0x04)
	{
	  IO_CTC2 = time_constant;
	}
      
      break;

    case 3:
      IO_CTC3 = control_word;
      if ( control_word & 0x04)
	{
	  IO_CTC3 = time_constant;
	}
      
      break;
      
    }
}

int z80_shield_ctc_read(int ctc)
{
  unsigned char time_constant = 0;
  
  switch(ctc)
    {
    case 0:
      time_constant = IO_CTC0;
      break;
    case 1:
      time_constant = IO_CTC1;
      break;
    case 2:
      time_constant = IO_CTC2;
      break;
    case 3:
      time_constant = IO_CTC3;
      break;
    }

  return((int)time_constant);
}
