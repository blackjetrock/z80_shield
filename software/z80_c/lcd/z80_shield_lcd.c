
#include "../pio/z80_shield_pio.h"

#define FULL_SPEED 1

void lcd_delay(void);

int lcd_a_shadow = 0;
int lcd_b_shadow = 0;


void lcd_rs_low(void)
{
  lcd_b_shadow &= 0xfe;
  IO_PIO1_B_DATA = lcd_b_shadow;
}

void lcd_e_low(void)
{
  lcd_b_shadow &= 0xfd;
  IO_PIO1_B_DATA = lcd_b_shadow;
}

void lcd_rs_high(void)
{
  lcd_b_shadow |= 0x01;
  IO_PIO1_B_DATA = lcd_b_shadow;
}

void lcd_e_high(void)
{
  lcd_b_shadow |= 0x02;
  IO_PIO1_B_DATA = lcd_b_shadow;
}

void lcd_send_s_data_4bits(char data)
{
  lcd_rs_low();
  lcd_e_high();
#if FULL_SPEED
  lcd_delay();
#endif
  
  IO_PIO1_A_DATA = (data << 4) & 0xf0;

#if FULL_SPEED
  lcd_delay();
#endif

  lcd_e_low();
  lcd_rs_high();

#if FULL_SPEED
  lcd_delay();
#endif
}

void lcd_send_s_data_8bits(char data)
{
  lcd_rs_low();
  lcd_e_high();
#if FULL_SPEED
  lcd_delay();
#endif

  IO_PIO1_A_DATA =  data;
#if FULL_SPEED
  lcd_delay();
#endif
  lcd_e_low();
  lcd_e_high();
#if FULL_SPEED
  lcd_delay();
#endif
  
  IO_PIO1_A_DATA = (data << 4) & 0xf0;
#if FULL_SPEED
  lcd_delay();
#endif

  lcd_e_low();
  lcd_rs_high();

#if FULL_SPEED
  lcd_delay();
#endif
}

void lcd_send_d_data_8bits(char data)
{
  lcd_rs_high();
  lcd_e_high();
#if FULL_SPEED
  lcd_delay();
#endif

  IO_PIO1_A_DATA =  data;
#if FULL_SPEED
  lcd_delay();
#endif
  lcd_e_low();
  lcd_e_high();
#if FULL_SPEED
  lcd_delay();
#endif
  
  IO_PIO1_A_DATA =(data << 4) & 0xf0;
#if FULL_SPEED
  lcd_delay();
#endif
  lcd_e_low();
  lcd_rs_high();
#if FULL_SPEED
  lcd_delay();
#endif
}


void lcd_delay(void)
{
  volatile int i,j;

  // Uncomment for running under Mega clocking
  
#if FULL_SPEED
#else
  
  return;
#endif
  
  for(j=0;j<50;j++)
    {
    }
}

void lcd_initialise(void)
{
  // We have to initialise the PIOs
  z80_shield_pio_init(PIO1, PIO_PORT_A, 0x00, 0x0F);
  z80_shield_pio_init(PIO1, PIO_PORT_B, 0x00, 0xFC);
  z80_shield_pio_init(PIO0, PIO_PORT_A, 0x01, 0xFB);  //AD_CS
  
  lcd_rs_low();
  lcd_e_low();
  lcd_delay();
  
  lcd_send_s_data_4bits(0x03);
  lcd_send_s_data_4bits(0x03);
  lcd_send_s_data_4bits(0x03);

  lcd_send_s_data_4bits(0x02);
  lcd_send_s_data_8bits(0x0e);
  lcd_send_s_data_8bits(0x06);

  lcd_send_s_data_8bits(0x01);

}

void lcd_cursor_line_col(int line, int col)
{
  lcd_send_s_data_8bits(0x80 + 0x40*line + col);
}

void lcd_display(char *text)
{
    while(*text != '\0' )
    {
      lcd_send_d_data_8bits(*text++);
    }
}
