/* 
 * Z80 Shield support for Z80 PIO
 *
 */

__sfr __at 0x00 IO_PIO0_A_DATA;  
__sfr __at 0x01 IO_PIO0_B_DATA;  
__sfr __at 0x02 IO_PIO0_A_CONTROL;
__sfr __at 0x03 IO_PIO0_B_CONTROL;

__sfr __at 0x80 IO_PIO1_A_DATA;  
__sfr __at 0x81 IO_PIO1_B_DATA;  
__sfr __at 0x82 IO_PIO1_A_CONTROL; 
__sfr __at 0x83 IO_PIO1_B_CONTROL; 


#define PIO0 0
#define PIO1 1
#define PIO_PORT_A 1
#define PIO_PORT_B 0

void z80_shield_pio_init(int pio, int a_nb, int data, int io_pattern);
