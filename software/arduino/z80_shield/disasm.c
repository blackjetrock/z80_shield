#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define boolean int
#define false 0
#define true (!false)

char *desc[] =
  {
    "2A nn nn 'LD HL,(n)'",
    "FD 21 nn nn 'LD IY, n'",
    "D9 'EXX'",
    "DD 11001011 dd 01bbb110 'BIT b, (IX+d)'",
    "CD 11001011 dd 01bbb110 'BIT b, (IY+d)'",
    "11qq0101 'PUSH q'",
    "11qq0001 'POP q'",
    "DD E5 'PUSH IX'",
    "FD E5 'PUSH IY'",
    "DD E1 'POP IX'",
    "FD E1 'POP IY'",
    "08 'EX AF, AF'",
    "18 ee 'JR e'",
    
    NULL,
  };

#define NUM_DESC (sizeof(desc))

unsigned char bytes[] = {
  0xdd, 0xcb, 0x23, 0x6e,
  0xcd, 0xcb, 0x23, 0x6e,
  0xF5, 0xE5, 0xC5, 0xD5, 0xDD, 0xE5, 0xFD, 0xE5,
  0xFD, 0xE1, 0xDD, 0xE1, 0xD1, 0xC1, 0xE1, 0xF1, 
  0x08, 0xD9, 0xF5, 0xE5, 0xC5, 0xD5, 0xD1, 0xC1,
  0xE1, 0xF1, 0xD9, 0x08, 0x18, 0xE2, 
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

	case '\'':
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
	case '\'':
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
		case 'd':
		case 'b':
		case 'e':
		  sprintf(field_val, "%X", fields[field_i].value);
		  strcat(decode_string, field_val);
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
  int i;
  int num_match = 0;
  int total_matched = 0;
  unsigned char *start = bytes;
  boolean any_match;

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
	      bytes += num_match;
	      strcat(disasm_str, "\n");
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
