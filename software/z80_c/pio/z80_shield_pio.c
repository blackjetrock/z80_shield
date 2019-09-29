/* 
 * Z80 Shield support for Z80 PIO
 *
 */

#include "z80_shield_pio.h"

// Initialise a PIO

void z80_shield_pio_init(int pio, int a_nb, int data, int io_pattern)
{
  switch(pio)
    {
    case 0:
      // Mode 3
      if( a_nb )
	{
	  IO_PIO0_A_CONTROL = 0xcf;
	  IO_PIO0_A_CONTROL = io_pattern;
	  IO_PIO0_A_DATA, data;
	}
      else
	{
	  IO_PIO0_B_CONTROL = 0xcf;
	  IO_PIO0_B_CONTROL = io_pattern;
	  IO_PIO0_B_DATA, data;
	}
      break;

    case 1:
      // Mode 3
      if( a_nb )
	{
	  IO_PIO1_A_CONTROL = 0xcf;
	  IO_PIO1_A_CONTROL = io_pattern;
	  IO_PIO1_A_DATA, data;
	}
      else
	{
	  IO_PIO1_B_CONTROL = 0xcf;
	  IO_PIO1_B_CONTROL = io_pattern;
	  IO_PIO1_B_DATA, data;
	}
      break;
      
    }

}
