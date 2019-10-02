/*

Compile with:

zcc +z80 -v -SO3 --max-allocs-per-node200000 -compiler sdcc -clib=sdcc_iy -startup=1 --c-code-in-asm -no-cleanup --list test_embedded.c -o test_embedded --std-c11 -create-app -Cz"--ihex --romsize=0x8000"

Inspect with:

z80dasm --origin 0 --address --labels test_embedded.rom | less

Convert to C with:

xxd -i test_embedded.rom > c_array.txt

*/

#pragma output CRT_ORG_CODE          = 0x0000
#pragma output CRT_ORG_DATA          = 0x8000
#pragma output CRT_ORG_BSS           = -1
#pragma output REGISTER_SP           = 0xFFFF
#pragma output CRT_STACK_SIZE        = 512
#pragma output CLIB_MALLOC_HEAP_SIZE = -1

#include <stdint.h>
#include <string.h>

static unsigned char data_str[] = "Z80 Shield";

void main()
{
  uint8_t i;
  uint8_t *ram_start = (uint8_t*)0x8100;

  for( i=0; i<2; i++ )
  {
    strncpy( ram_start, data_str, strlen(data_str)+1 );
    ram_start += strlen(data_str)+1;
  }

  while(1);
}
