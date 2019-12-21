// Short program to display a string on an LCD
//
//

#include "../z80_shield/z80_shield.h"
#include "../ctc/z80_shield_ctc.h"
#include "../lcd/z80_shield_lcd.h"
#include <stdio.h>


int main(void)
{
  char x[20];
  int y = 9038;
  
  // Enable interrupts
  __asm
    im 1
    di
    __endasm;
  
  // Set up CTC as a timer

  // Prescaler 16
  //z80_shield_ctc_channel_init(0, 0x05, 0x20);
  // Prescaler 256
  z80_shield_ctc_channel_init(0, 0x25, 0x50);
  
  lcd_initialise();
  lcd_display("Hello World ");

  // Loop reading CTC time constant
  while(1)
    {
      lcd_cursor_line_col(1,0);
      sprintf(x, "%03d", z80_shield_ctc_read(0));
      //sprintf(x, "%03d", y++);
      lcd_display(x);
      
    }
  
  // Sit in loop when we are done
  while(1)
    {
    }
}
	 
