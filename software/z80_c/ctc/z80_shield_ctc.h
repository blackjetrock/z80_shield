/* 
 * Z80 Shield support for Z80 CTC
 *
 */

__sfr __at 0x40 IO_CTC0;
__sfr __at 0x41 IO_CTC1;
__sfr __at 0x42 IO_CTC2;
__sfr __at 0x43 IO_CTC3;


void z80_shield_ctc_channel_init(int ctc_channel, int control_word, int time_constant);
int z80_shield_ctc_read(int ctc);

