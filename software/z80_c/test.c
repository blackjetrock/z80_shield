/*include <stdio.h>*/
#include <stdlib.h>
#include "z80_shield_pio.h"
#include "z80_shield_lcd.h"

void fputc_cons_native(int c);
void lcd_display(char *string);

int main()
{
#asm
  ld sp, 9000H
#endasm
    
    /* Initialise PIOs */
    z80_shield_pio_init(IO_ADDR_PIO1, PIO_A_PORT, 0x00, 0x0f);
  z80_shield_pio_init(IO_ADDR_PIO1, PIO_B_PORT, 0x00, 0xfc);
  
  //    z80_shield_pio_init(IO_ADDR_PIO1, PIO_B_PORT, 0x00, 0xfc);
  lcd_display("Hello World!");
  return 0;
}


void fputc_cons_native(int c)
{
}


