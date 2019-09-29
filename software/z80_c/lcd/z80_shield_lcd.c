#include "z80_shield_pio.h"

int lcd_a_shadow = 0;
int lcd_b_shadow = 0;


void lcd_rs_low(void)
{
  lcd_b_shadow &= 0xfe;
  outp(IO_ADDR_PIO1+PIO_B_DATA, lcd_b_shadow);
}

void lcd_e_low(void)
{
  lcd_b_shadow &= 0xfd;
  outp(IO_ADDR_PIO1+PIO_B_DATA, lcd_b_shadow);
}

void lcd_rs_high(void)
{
  lcd_b_shadow |= 0x01;
  outp(IO_ADDR_PIO1+PIO_B_DATA, lcd_b_shadow);
}

void lcd_e_high(void)
{
  lcd_b_shadow |= 0x02;
  outp(IO_ADDR_PIO1+PIO_B_DATA, lcd_b_shadow);
}

void lcd_send_s_data_4bits(int data)
{
  lcd_rs_low();
  lcd_e_high();
  outp(IO_ADDR_PIO1+PIO_A_DATA, (data << 4) & 0xf0);
  lcd_e_low();
  lcd_rs_high();
}

void lcd_send_s_data_8bits(int data)
{
  lcd_rs_low();
  lcd_e_high();

  outp(IO_ADDR_PIO1+PIO_A_DATA,  data & 0x0f);
  lcd_e_low();
  lcd_e_high();
  
  outp(IO_ADDR_PIO1+PIO_A_DATA, (data << 4) & 0xf0);
  lcd_e_low();
  lcd_rs_high();
}

void lcd_send_d_data_8bits(int data)
{
  lcd_rs_high();
  lcd_e_high();

  outp(IO_ADDR_PIO1+PIO_A_DATA,  data & 0x0f);
  lcd_e_low();
  lcd_e_high();
  
  outp(IO_ADDR_PIO1+PIO_A_DATA, (data << 4) & 0xf0);
  lcd_e_low();
  lcd_rs_high();
}


void lcd_delay(int delay)
{
  int i,j;

  for(i=0;i<delay;i++)
    {
        for(j=0;j<10000;j++)
	  {
	  }
    }
  
}

void lcd_initialise(void)
{
  lcd_rs_low();
  lcd_e_low();
  lcd_delay(1000);
  
  lcd_send_s_data_4bits(0x03);
  lcd_send_s_data_4bits(0x03);
  lcd_send_s_data_4bits(0x03);

  lcd_send_s_data_4bits(0x02);
  lcd_send_s_data_8bits(0x0e);
  lcd_send_s_data_8bits(0x06);

  lcd_send_s_data_8bits(0x01);

}

void lcd_display(char *text)
{
  while(*text != '\0' )
    {
      lcd_send_d_data_8bits(*(text++));
    }
  
}
