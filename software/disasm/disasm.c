#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define boolean int
#define false 0
#define true (!false)

#define ENDIAN_SWAP(x) (((x&0xff)<<8)+((x&0xff00)>>8))

char *desc[] =
  {

    // 8 bit load
    "01rrrsss :LD r,s:",
    "00rrr110 nn :LD r, n:",
    "01rrr110 :LD r, (HL):",
    "11y11101 01rrr110 dd :LD r, (y+d):",
    "01110rrr :LD (HL), r:",
    "11y11101 01110rrr dd :LD (y+d), r:",

    "00pp0001 nn nn :LD p, n:",
    "2A nn nn :LD HL,(nn):",
    "00110110 nn :LD (HL), n:",    
    "11y11101 00110110 dd nn :LD (y+d), n:",
    "02 :LD (BC),A:",
    "12 :LD (DE),A:",
    "3A nn nn :LD A, (nn):",
    "1A :LD A,(DE):",
    "0A :LD A,(BC):",
    "32 nn nn :LD (nn), A:",
    
    "ED 57 :LD A,I:",
    "ED 5F :LD, A,R:",
    "ED 47 :LD I,A:",
    "ED 4F :LD R,A:",

    // 16 bit load
    "00pp0001 nn nn :LD p, nn:",
    "11y11101 21 nn nn :LD y, nn:",
    "2A nn nn :LD HL, (nn):",
    "ED 01pp1011 nn nn :LD p, (nn):",
    "11y11101 2A nn nn :LD y, (nn):",
    "22 nn nn :LD (nn), HL:",
    "ED 01pp0011 nn nn :LD (nn), p:",
    "11y11101 22 nn nn :LD (nn), y:",
    "F9 :LD SP, HL:",
    "11y11101 :LD SP, y:",
    "11qq0101 :PUSH q:",
    "11qq0001 :POP q:",
    "11y11101 E5 :PUSH y:",
    "11y11101 E1 :POP  y:",

    // Exchange, block transfer and search
    "EB :EX DE, HL:",
    "08 :EX AF, AF':",
    "D9 :EXX:",
    "E3 :EX (SP), HL:",
    "11y11101 E3 :EX(y), HL:",
    "ED A0 :LDI:",
    "ED B0 :LDIR:",
    "ED A8 :LDD:",
    "ED B8 :LDDR:",
    "ED A1 :CPI:",
    "ED B1 :CPIR:",
    "ED A9 :CPD:",
    "ED B9 :CPDR:",

    // 8 bit arithmetic
    "10000rrr :ADD A, r:",
    "11000110 nn :ADD A, n:",
    "86 :ADD A, (HL):",
    "11y11101 86 dd :ADD A, (y+d):",

    "10001rrr :ADC A, r:",
    "11001110 nn :ADC A, n:",
    "11y11101 8E dd :ADC A, (y+d):",

    "10010rrr :SUB r:",
    "11010110 nn :SUB n:",
    "11y11101 96 dd :SUB (y+d):",

    "10011rrr :SBC A, r:",
    "11011110 nn :SBC A, n:",
    "11y11101 9E dd :SBC A, (y+d):",

    "10100rrr :AND r:",
    "11100110 nn :AND n:",
    "11y11101 A6 dd :AND (y+d):",
    
    "10110rrr :OR r:",
    "11110110 nn :OR n:",
    "11y11101 B6 dd :OR (y+d):",

    "10101rrr :XOR r:",
    "11101110 nn :XOR n:",
    "11y11101 AE dd :XOR (y+d):",

    "10111rrr :CP r:",
    "11111110 nn :CP n:",
    "11y11101 BE dd :CP (y+d):",

    "00rrr100 :INC r:",
    "11y11101 34 dd :INC (y+d):",
    
    "00rrr101 :DEC r:",
    "11y11101 35 dd :DEC (y+d):",

    // General purpose arithmetic and CPU control
    "27 :DAA:",
    "2F :CPL:",
    "ED 44 :NEG:",
    "3F :CCF:",
    "37 :SCF:",
    "00 :NOP:",
    "76 :HALT:"
    "F3 :DI:",
    "FB :EI:",
    "ED 46 :IM 0:",
    "ED 56 :IM 1:",
    "ED 5E :IM 2:",

    // 16 bit arithmetic
    "00pp1001 :ADD HL, p:",
    "ED 01pp1010 :ADC HL, p:",
    "ED 01pp0010 :SBC HL, p:",
    "DD 00pp1001 :SBC HL, px:",
    "FD 00pp1001 :SBC HL, py:",
    "00pp0011 :INC p:",
    "11y11101 23 :INC y:",
    "00pp1011 :DEC p:",
    "11y11101 2B :DEC y:",

    // Rotate and shift
    "07 :RLCA:",
    "17 :RLA:",
    "0F :RRCA:",
    "1F :RRA:",
    "CB 00000rrr :RLC r:",
    "11y11101 CB dd 06 :RLC (y+d):",

    "CB 00010rrr :RL r:",
    "11y11101 CB dd 16 :RL (y+d):",

    "CB 00001rrr :RRC r:",
    "11y11101 CB dd 0E :RRC (y+d):",
    
    "CB 00011rrr :RR r:",
    "11y11101 CB dd 1E :RR (y+d):",

    "CB 00100rrr :SLA r:",
    "11y11101 CB dd 26 :SLA (y+d):",

    "CB 00101rrr :SRA r:",
    "11y11101 CB dd 2E :SRA (y+d):",

    "CB 001!1rrr :SRL r:",
    "11y11101 CB dd 3E :SRL (y+d):",

    "ED 6F :RLD:",
    "ED 67 :RRD:",

    // Bit set, reset and test
    "CB 01bbbrrr :BIT b, r:",
    "11y11101 CB dd 01bbb110 :BIT b, (y+d):",
    "CB 11bbbrrr :SET b, r:",
    "11y11101 CB dd 01bbb110 :SET b, (y+d):",
    "CB 10bbbrrr :RES b, r:",
    "11y11101 CB dd 10bbb110 :RES b, (y+d):",

    // Jump
    "C3 nn nn :JP nn:",
    "11ccc010 nn nn :JP c, nn:",
    "18 ee :JR e:",
    "38 ee :JR C, e:",
    "30 ee :JR NC, e:",
    "28 ee :JR Z, e:",
    "20 ee :JR NZ, e:",
    "E9 :JP (HL):",
    "11y11101 E9 :JP(y):",
    "10 ee :DJNZ e:",
    "38 ee :JR C, e:",

    // Call and return
    "CD nn nn :CALL nn:",
    "11ccc100 nn nn :CALL cc, nn:",
    "C9 :RET:",
    "11ccc000 :RET c:",
    "ED 4D :RETI:",
    "ED 45 :RETN:",
    "11ttt111 :RST t:",

    // Input and output
    "DB nn :IN A, (n):",
    "ED 01rrr000 :IN r, (C):",
    "ED A2 :INI:",
    "ED B2 :INIR:",
    "ED AA :IND:",
    "ED BA :INDR:",
    "D3 nn :OUT (n), A:",
    "ED 01rrr001 :OUT (C), r:",
    "ED A3 :OUTI:",
    "ED B3 :OTIR:",
    "ED AB :OUTD:",
    "ED BB :OTDR:",

    NULL,
  };

#define NUM_DESC (sizeof(desc))

unsigned char bytes[] = {
  0xdd, 0xcb, 0x23, 0x6e,
  0xff, 0xc7,
  0xfd, 0xcb, 0x23, 0x6e,
  0xF5, 0xE5, 0xC5, 0xD5, 0xDD, 0xE5, 0xFD, 0xE5,
  0xFD, 0xE1, 0xDD, 0xE1, 0xD1, 0xC1, 0xE1, 0xF1, 
  0x08, 0xD9, 0xF5, 0xE5, 0xC5, 0xD5, 0xD1, 0xC1,
  0xE1, 0xF1, 0xD9, 0x08, 0x18, 0xE2,
  0x31, 0x00, 0x90, 0x0E, 0x82, 0x16, 0x80, 0x26, 0x0F, 0x2E, 0x00, 0xCD, 0x80, 0x01, 0x0E, 0x83, 
  0x16, 0x81, 0x26, 0xFC, 0x2E, 0x00, 0xCD, 0x80, 0x01, 0x0E, 0x02, 0x16, 0x00, 0x26, 0xF8, 0x2E,
  0x01, 0xCD, 0x80, 0x01, 0x3E, 0x01, 0x32, 0x02, 0xA0, 0x3E, 0x00, 0x32, 0x00, 0xA0, 0x3E, 0x00, 
  0x32, 0x01, 0xA0, 0xCD, 0x8C, 0x01, 0xCD, 0xAC, 0x01, 0xCD, 0x75, 0x01, 0x16, 0x03, 0xCD, 0x16, 
  0x02, 0xCD, 0x75, 0x01, 0x16, 0x03, 0xCD, 0x16, 0x02, 0xCD, 0x75, 0x01, 0x16, 0x03, 0xCD, 0x16, 
  0x02, 0xCD, 0x75, 0x01, 0x16, 0x02, 0xCD, 0x16, 0x02, 0xCD, 0x75, 0x01, 0x16, 0x0E, 0xCD, 0xCC, 
  0x01, 0xCD, 0x75, 0x01, 0x16, 0x06, 0xCD, 0xCC, 0x01, 0xCD, 0x75, 0x01, 0x16, 0x01, 0xCD, 0xCC, 
  0x01, 0xCD, 0x75, 0x01, 0x16, 0x5A, 0xCD, 0xF1, 0x01, 0xCD, 0x75, 0x01, 0x16, 0x38, 0xCD, 0xF1, 
  0x01, 0xCD, 0x75, 0x01, 0x16, 0x30, 0xCD, 0xF1, 0x01, 0xCD, 0x75, 0x01, 0x16, 0x20, 0xCD, 0xF1, 
  0x01, 0xCD, 0x75, 0x01, 0x16, 0x53, 0xCD, 0xF1, 0x01, 0xCD, 0x75, 0x01, 0x16, 0x68, 0xCD, 0xF1, 
  0x01, 0xCD, 0x75, 0x01, 0x16, 0x69, 0xCD, 0xF1, 0x01, 0xCD, 0x75, 0x01, 0x16, 0x65, 0xCD, 0xF1,
  0x01, 0xCD, 0x75, 0x01, 0x16, 0x6C, 0xCD, 0xF1, 0x01, 0xCD, 0x75, 0x01, 0x16, 0x64, 0xCD, 0xF1,
  0x01, 0xCD, 0x75, 0x01, 0x16, 0x02, 0xCD, 0xCC, 0x01, 0xCD, 0xE9, 0x00, 0xED, 0x43, 0x00, 0xB0,
  0x3E, 0x41, 0x81, 0x57, 0xCD, 0xF1, 0x01, 0x18, 0xEB, 0x18, 0xFE, 0x7E, 0xFE, 0x00, 0x20, 0x01,
  0xC9, 0x7E, 0x57, 0xCD, 0xF1, 0x01, 0x23, 0x18, 0xF2, 0xCD, 0x60, 0x02, 0xCD, 0x30, 0x02, 0xCD,
  0x50, 0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x60,
  0x02, 0xCD, 0x50, 0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02,
  0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x80, 0x02, 0xCD,
  0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x80, 0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x70,
  0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x70, 0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02,
  0xCD, 0x70, 0x02, 0xCD, 0x60, 0x02, 0xCD, 0x50, 0x02, 0xCD, 0x70, 0x02, 0xCD, 0x60, 0x02, 0xCD,
  0x50, 0x02, 0xCD, 0x70, 0x02, 0xCD, 0x60, 0x02, 0x16, 0x0A, 0x01, 0x00, 0x00, 0xCD, 0x60, 0x02,
  0xCD, 0x90, 0x02, 0xCB, 0x21, 0xCB, 0x10, 0xE6, 0x01, 0xB1, 0x4F, 0xCD, 0x50, 0x02, 0x15, 0x20,
  0xEC, 0xCD, 0x40, 0x02, 0xC9, 0x26, 0x02, 0x2E, 0xFF, 0x2D, 0x20, 0xFD, 0x25, 0x20, 0xF8, 0xC9,
  0x3E, 0xCF, 0xED, 0x79, 0x7C, 0xED, 0x79, 0x7D, 0x4A, 0xED, 0x79, 0xC9, 0xC5, 0xD5, 0x0E, 0x81,
  0x21, 0x01, 0xA0, 0x7E, 0xCB, 0x87, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9, 0xC5, 0xD5, 0x0E, 0x81,
  0x21, 0x01, 0xA0, 0x7E, 0xCB, 0xC7, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9, 0xC5, 0xD5, 0x0E, 0x81,
  0x21, 0x01, 0xA0, 0x7E, 0xCB, 0x8F, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9, 0xC5, 0xD5, 0x0E, 0x81,
  0x21, 0x01, 0xA0, 0x7E, 0xCB, 0xCF, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9, 0x0E, 0x80, 0xCD, 0x8C,
  0x01, 0xCD, 0xBC, 0x01, 0x7A, 0x0E, 0x80, 0xED, 0x79, 0xCD, 0xAC, 0x01, 0xCD, 0xBC, 0x01, 0x7A,
  0xCB, 0x27, 0xCB, 0x27, 0xCB, 0x27, 0xCB, 0x27, 0xED, 0x79, 0xCD, 0xAC, 0x01, 0xCD, 0x9C, 0x01,
  0xC9, 0x0E, 0x80, 0xCD, 0x9C, 0x01, 0xCD, 0xBC, 0x01, 0x7A, 0x0E, 0x80, 0xED, 0x79, 0xCD, 0xAC,
  0x01, 0xCD, 0xBC, 0x01, 0x7A, 0xCB, 0x27, 0xCB, 0x27, 0xCB, 0x27, 0xCB, 0x27, 0xED, 0x79, 0xCD,
  0xAC, 0x01, 0xCD, 0x9C, 0x01, 0xC9, 0xCD, 0x8C, 0x01, 0xCD, 0xBC, 0x01, 0x7A, 0xCB, 0x27, 0xCB,
  0x27, 0xCB, 0x27, 0xCB, 0x27, 0x0E, 0x80, 0xED, 0x79, 0xCD, 0xAC, 0x01, 0xCD, 0x9C, 0x01, 0xC9,
  0xC5, 0xD5, 0x0E, 0x00, 0x21, 0x02, 0xA0, 0x7E, 0xCB, 0x87, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9,
  0xC5, 0xD5, 0x0E, 0x00, 0x21, 0x02, 0xA0, 0x7E, 0xCB, 0xC7, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9,
  0xC5, 0xD5, 0x0E, 0x00, 0x21, 0x02, 0xA0, 0x7E, 0xCB, 0x8F, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9,
  0xC5, 0xD5, 0x0E, 0x00, 0x21, 0x02, 0xA0, 0x7E, 0xCB, 0xCF, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9,
  0xC5, 0xD5, 0x0E, 0x00, 0x21, 0x02, 0xA0, 0x7E, 0xCB, 0x97, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9,
  0xC5, 0xD5, 0x0E, 0x00, 0x21, 0x02, 0xA0, 0x7E, 0xCB, 0xD7, 0x77, 0xED, 0x79, 0xD1, 0xC1, 0xC9,
  0xC5, 0xD5, 0x0E, 0x00, 0xED, 0x78, 0xE6, 0x08, 0xCB, 0x3F, 0xCB, 0x3F, 0xCB, 0x3F, 0xD1, 0xC1,

};

boolean word_literal(char *word)
{
  boolean is_literal = true;
  char *original_word = word;
  
  // A word is literal if it has no lower case characters
  while(*word != '\0')
    {
      if ( islower(*(word++)) )
	{
	  is_literal = false;
	}
      word++;
    }

  printf("\n%s %s literal", original_word, is_literal?"is":"is not");
  return(is_literal);
}

// Field values are global
typedef struct _field
{
  char name;
  int value;
} FIELD;

int num_fields;
FIELD fields[8];

void dump_fields(void)
{
  int i;

  printf("\nFields\n");
  
  for(i=0; i< num_fields; i++)
    {
      printf("\n%d : %c = %d", i, fields[i].name, fields[i].value);
    }

  printf("\n");
}

int find_field(char name)
{
  int i;
  
  for(i=0; i<num_fields; i++)
    {
      if( fields[i].name == name )
	{
	  return(i);
	}
    }
  return(-1);
}

void add_to_value(char name, int value, int radix)
{
  int index = find_field(name);

  if( index == -1 )
    {
      index = num_fields++;
      fields[index].name = name;
      fields[index].value = 0;
    }
  
  fields[index].value *= radix;
  fields[index].value += value;
  printf("\nField %d: Add %d(%d) : %c     -> %d", index, value, radix, fields[index].name, fields[index].value);
}

void set_field_value(char name, int value)
{
  int i;
  
  for(i=0; i<num_fields; i++)
    {
      if( fields[i].name == name )
	{
	  fields[i].value = value;
	  return;
	}
    }

  printf("\nNew field set field %c to %d", name, value);
  fields[num_fields].name = name;
  fields[num_fields].value = value;
  num_fields++;
  printf("\n Now %d fields", num_fields);
}

// Initialise any fields that are present in the word
void init_field_values(char *word)
{
  num_fields = 0;
  
  while(*word != '\0')
    {
      if( islower(*word) )
	{
	  // Got a field character, do we already have a field for this?
	  // Zero field
	  set_field_value(*word, 0);
	}
      word++;
    }
}

boolean match_word(char *word, unsigned char byte)
{
  int vword;
  boolean matches = false;
  char b[8];
  int i;
  int char_i;
  
  printf("\n%s", __FUNCTION__);
  
  // See if the word is a literal value
  if( word_literal(word))
    {
      // If the word is two characters then it matches the byte as hex
      // If 8 characters then it matches as binary
      
      switch(strlen(word))
	{
	case 2:
	  // Get value if it is a literal hex byte
	  sscanf(word, "%02X", &vword);

	  // Do we have a match?
	  if( vword == byte)
	    {
	      matches = true;	      
	    }
	  break;
	  
	case 8:

	  // Convert literal binary to value
	  vword = 0;
	  
	  for(i=0; i<8; i++)
	    {
	      vword *= 2;
	      vword+= word[i]=='0'?0:1;
	    }
	  
	  // Do we have a match?
	  if( vword == byte)
	    {
	      matches = true;	      
	    }
	  
	  break;
	  
	default:
	  // Bad
	  break;
	}
    }
  else
    {
      int nibble_mask;
      int bit_mask;
      int charval;
      int wordval;

      matches = true;
      
      // Word has fields in it
      
      // Build up the field values and names
      switch(strlen(word))
	{
	case 2:
	  printf("\n>>> Hex ");
	  
	  // Hex digits represent nibbles
	  nibble_mask = 0xf0;
	  char_i = strlen(word);
	  
	  while(*word != '\0' )
	    {
	      switch(*word)
		{
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
		  // See if the nibble matches
		  charval = (byte & nibble_mask) >> (4*(char_i-1));
		  sscanf(word, "%1X", &wordval);

		  if( charval != wordval )
		    {
		      matches = false;
		    }
		  break;

		default:
		  printf("\n>>>>%d %c", char_i, *word);
		  
		  // process nibble
		  charval = (byte & nibble_mask) >> (4*(char_i-1));
		  
		  printf("\n>>>>%d %d %d", charval, nibble_mask, byte);
		  
		  // Add this nibble to the field value
		  add_to_value(*word, charval, 16);
		  break;
		}
	      
	      word++;
	      nibble_mask>>=4;
	      char_i--;
	      
	    }
	  
	  break;

	case 8:
	  printf("\n>>> Binary");


	  // Binary digits represent bits
	  bit_mask = 0x80;
	  char_i = strlen(word);
	  
	  while(*word != '\0' )
	    {
	      switch(*word)
		{
		case '0':
		case '1':
		  // See if the nibble matches
		  charval = (byte & bit_mask) >> (1*(char_i-1));
		  sscanf(word, "%1d", &wordval);

		  if( charval != wordval )
		    {
		      matches = false;
		    }
		  break;

		  
		default:
		  printf("\n>>>>%d %c", char_i, *word);
		  
		  // process bit
		  charval = (byte & bit_mask) >> (1*(char_i-1));
		  
		  printf("\n>>>>%d %d %d", charval, bit_mask, byte);
		  
		  // Add this bit to the field value
		  add_to_value(*word, charval, 2);
		  break;

		  
		}
	      word++;
	      bit_mask>>=1;
	      char_i--;
	    }
	  break;

	default:
	  // Bad
	  break;
	}
      

    }
  
  printf("\nWord:'%s' Byte:%02X", word, byte);
  printf("\nWord value    :%02X", vword);
  printf("\nWord and byte %s", matches?"match":"do not match");
  printf("\n");
  return(matches);
}

// Returns number of matching bytes

boolean desc_match(char *desc, unsigned char *bytes)
{
  int di;    // desc string index
  int bi;    // byte list index
  boolean all_match = true;
  boolean done = false;
  int num_words = 0;
  
  char word[20];
  char c[2];

  printf("\nDESC:'%s'", desc);

  // The desc has words that encode the instruction, process them in order
  // one word per byte
  di = 0;
  word[0] = '\0';

  // Get words and process them
  while ( !done && (strlen(desc) > 0) )
    {
      printf("\ndoing '%c'", *desc);
      
      switch(*desc)
	{

	case ':':
	  // If we get the string marker then all bytes have to have matched
	  if( !all_match )
	    {
	      // Not a match
	      return(0);
	    }

	  // We have a match
	  return(num_words);
	  break;
	  
	  // Start of new word, process previous one
	case '\0':
	case ' ':

	  // Process word
	  if( !match_word(word, *(bytes++)) )
	    {
	      all_match = false;
	    }
	  num_words++;
	  
	  word[0] = '\0';
	  break;

	default:
	  c[0] = *desc;
	  c[1] = '\0';
	  strcat(word, c);

	  if( strlen(desc) == 1 )
	    {
	      if( !match_word(word, *(bytes++)) )
		{
		  all_match = false;
		}

	      num_words++;
	    }
	  break;
	}

     
      desc++;
    }
  
  // If the number of matches equals the length then the next word should be the
  // format string
  if( all_match )
    {
      printf("\nBytes match ");
      return(num_words);
    }
  else
    {
        return(0);
    }
}

// We have a desc and some bytes, we now have to decode the desc and create the
// disassembled instruction

char decode_string[40];

char * desc_decode(char *desc, unsigned char *bytes)
{

  int i;
  char fmt_str[40];
  char field_val[10];
  int field_i;
  
  dump_fields();
  
  printf("\nDECODE:%s", desc);
  boolean done = false;
  decode_string[0] = '\0';
  
  fmt_str[0] = '\0';
  
  for(i=0; (i<strlen(desc)) && !done; i++)
    {
      switch(*(desc+i))
	{
	case ':':
	  // Get the format string
	  strcpy(fmt_str, desc+i+1);
	  fmt_str[strlen(desc+i)-2] = '\0';
	  done = true;
	  break;
	  
	default:
	  break;
	}
    }
  
  // Build the result, substituting fields as needed
  for(i=0; i<strlen(fmt_str); i++ )
    {
      if( islower(fmt_str[i]) )
	{
	  field_i = find_field(fmt_str[i]);
	  
	  if ( field_i == -1 )
	    {
	      strcat(decode_string, "?");
	    }
	  else
	    {
	      switch(fields[field_i].name)
		{
		  
		case 'b':
		  sprintf(field_val, "%d", fields[field_i].value);
		  strcat(decode_string, field_val);
		  break;
		  
		case 'e':
		  sprintf(field_val, "%02XH", fields[field_i].value);
		  strcat(decode_string, field_val);
		  break;

		case 't':
		  sprintf(field_val, "%02XH", fields[field_i].value*8);
		  strcat(decode_string, field_val);
		  break;

		case 'c':
		  // c or cc show condition code
		  if( fmt_str[i+1] == 'c' )
		    {
		      i++;
		    }
		  switch(fields[field_i].value)
		    {
		    case 0:
		      strcat(decode_string, "NZ");
		      break;
		    case 1:
		      strcat(decode_string, "Z");
		      break;
		    case 2:
		      strcat(decode_string, "NC");
		      break;
		    case 3:
		      strcat(decode_string, "C");
		      break;
		    case 4:
		      strcat(decode_string, "PO");
		      break;
		    case 5:
		      strcat(decode_string, "PE");
		      break;
		    case 6:
		      strcat(decode_string, "P");
		      break;
		    case 7:
		      strcat(decode_string, "M");
		      break;
		    }
		  

		  break;

		case 'n':
		case 'd':
		  // n is 8 bit
		  // nn is 16 bit
		  // dn shouldn't be used, but would be 16 bit
		  if( fmt_str[i+1] == 'n' )
		    {
		      // Endian swap
		      
		      sprintf(field_val, "%04XH", ENDIAN_SWAP(fields[field_i].value));
		      strcat(decode_string, field_val);
		      i++;
		    }
		  else
		    {
		      sprintf(field_val, "%02XH", fields[field_i].value);
		      strcat(decode_string, field_val);
		    }
		  break;

		case 'q':
		  switch(fields[field_i].value)
		    {
		    case 0:
		      strcat(decode_string, "BC");
		      break;
		    case 1:
		      strcat(decode_string, "DE");
		      break;
		    case 2:
		      strcat(decode_string, "HL");
		      break;
		    case 3:
		      strcat(decode_string, "AF");
		      break;
		    }
		  break;

		case 'r':
		case 's':
		  switch(fields[field_i].value)
		    {
		    case 0:
		      strcat(decode_string, "B");
		      break;
		    case 1:
		      strcat(decode_string, "C");
		      break;
		    case 2:
		      strcat(decode_string, "D");
		      break;
		    case 3:
		      strcat(decode_string, "E");
		      break;
		    case 4:
		      strcat(decode_string, "H");
		      break;
		    case 5:
		      strcat(decode_string, "L");
		      break;
		    case 6:
		      strcat(decode_string, "(HL)");
		      break;
		    case 7:
		      strcat(decode_string, "A");
		      break;
		    }
		  break;

		  // IY is 1 bit

		case 'y':
		  switch(fields[field_i].value)
		    {
		    case 0:
		      strcat(decode_string, "IX");
		      break;
		    case 1:
		      strcat(decode_string, "IY");
		      break;
		    }
		  break;
		  
		case 'p':
		  switch(fields[field_i].value)
		    {
		    case 0:
		      strcat(decode_string, "BC");
		      break;
		    case 1:
		      strcat(decode_string, "DE");
		      break;
		    case 2:
// If next char is x or y then modify
		      switch( fmt_str[i+1] )
			{
			case 'x':
			  strcat(decode_string, "IX");
			  i++;
			  break;

			case 'y':
			  strcat(decode_string, "IY");
			  i++;
			  break;

			default:
			  strcat(decode_string, "HL");
			  break;
			}

		      break;
		    case 3:
		      strcat(decode_string, "SP");
		      break;
		    }
		  break;
		}
	      
	    }
	  
	}
      else
	{
	  sprintf(field_val, "%c", fmt_str[i]);
	  strcat(decode_string, field_val);
	  
	}
      
    }
  
  printf("\nFMT str:'%s'", fmt_str);
  printf("\n                                               %s", decode_string);
  
}

char disasm_str[10000];

// Length prefixed bytes
void disasm(unsigned char *bytes, int num_bytes)
{
  int i, j;
  int num_match = 0;
  int total_matched = 0;
  unsigned char *start = bytes;
  boolean any_match;
  char byte_str[3] = "..";
  
  disasm_str[0] = '\0';
  
  while( bytes < (start+num_bytes) )
    {
      any_match = false;
      for(i=0; i< NUM_DESC; i++)
	{
	  // Reset field values
	  num_fields = 0;

	  // End of desc list
	  if( desc[i] == NULL )
	    {
	      break;
	    }

	  // Does this one match?
	  if ( (num_match = desc_match(desc[i], bytes)) > 0 )
	    {
	      any_match = true;
	      
	      // Yes, get decoded instruction
	      desc_decode(desc[i], bytes);
	      
	      total_matched += num_match;

	      strcat(disasm_str, "\n");
	      
	      for(j=0; j<num_match; j++)
		{
		  sprintf(byte_str, "%02X", *(bytes+j));
		  strcat(disasm_str, " ");
		  strcat(disasm_str, byte_str);
		}
	      
	      for(; j<6; j++)
		{
		  strcat(disasm_str, "   ");
		}
	      
	      bytes += num_match;	      
	      strcat(disasm_str, decode_string);
	      break;
	    }
	  
	  dump_fields();
	}
      
      if( !any_match )
	{
	  printf("\nNo match at:");
	  for(i=0; i<10; i++)
	    {
	      printf("\n%02X", *(bytes+i));
	    }
	  break;
	}
    }
}

int main(void)
{

  disasm(bytes, sizeof(bytes));

  printf("\n%s\n", disasm_str);
}
