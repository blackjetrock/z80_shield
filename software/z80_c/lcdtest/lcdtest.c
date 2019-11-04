// Short program to display a string on an LCD
//
//

#include "../z80_shield/z80_shield.h"
#include "../lcd/z80_shield_lcd.h"
#include <stdio.h>


int main(void)
{
  char x[20];
  int y = 9038;


  
  lcd_initialise();
  lcd_display("Hello World ");

  while(1)
    {
      lcd_cursor_line_col(1,0);
      sprintf(x, "%d", y);
      lcd_display(x);
      y++;
    }
  
  // Sit in loop when we are done
  while(1)
    {
    }
}
	 
