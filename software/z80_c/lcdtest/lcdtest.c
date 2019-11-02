// Short program to display a string on an LCD
//
//

#include "../z80_shield/z80_shield.h"
#include "../lcd/z80_shield_lcd.h"


int main(void)
{
  lcd_initialise();
  lcd_display("Hello World");

  // Sit in loop when we are done
  while(1)
    {
    }
}
	 
