/* Test C file */

#pragma output CRT_ORG_CODE          = 0x0000
#pragma output CRT_ORG_DATA          = 0x8000
#pragma output CRT_ORG_BSS           = -1
#pragma output REGISTER_SP           = 0xFFFF
#pragma output CRT_STACK_SIZE        = 512
#pragma output CLIB_MALLOC_HEAP_SIZE = -1

char x[10];
char y ='p';

int main(void)
{
volatile  int a;
volatile  int b;

 a = 25;
 b = 4;
 
   a = b*10;


}
