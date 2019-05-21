//
// Arduino Mega sketch for the Z80 shield
// Serial monitor based control program
//

void run_bsm(int stim);
char * bsm_state_name();

// Run in quiet mode
boolean quiet = false;

typedef unsigned char BYTE;
typedef void (*FPTR)();
typedef void (*CMD_FPTR)(String cmd);


#define VERTICAL_LABELS  0

// Pin definitions

const int D0_Pin      = 21;
const int D1_Pin      = 19;
const int D2_Pin      = 10;
const int D3_Pin      = 59;
const int D4_Pin      = 60;
const int D5_Pin      = 61;
const int D6_Pin      = 62;
const int D7_Pin      = 63;

const int A0_Pin      = 9;
const int A1_Pin      = 8;
const int A2_Pin      = 7;
const int A3_Pin      = 6;
const int A4_Pin      = 5;
const int A5_Pin      = 4;
const int A6_Pin      = 3;
const int A7_Pin      = 2;
const int A8_Pin      = 64;
const int A9_Pin      = 65;
const int A10_Pin     = 66;
const int A11_Pin     = 67;
const int A12_Pin     = 68;
const int A13_Pin     = 69;
const int A14_Pin     = 53;
const int A15_Pin     = 52;

const int BUSREQ_Pin  = 28;
const int BUSACK_Pin  = 24;
const int WR_Pin      = 32;
const int RD_Pin      = 34;
const int MREQ_Pin    = 30;
const int IOREQ_Pin   = 46;
const int HALT_Pin    = 22;
const int NMI_Pin     = 39;
const int INT_Pin     = 18;
const int M1_Pin      = 35;
const int WAIT_Pin    = 37;
const int A_CLK_Pin   = 41;
const int A_RES_Pin   = 33;
const int RFSH_Pin    = 31;
const int SW0_Pin     = 29;
const int SW1_Pin     = 25;

const int MAPREQ_Pin  = 0;

const int address_pins[] =
  {
    A0_Pin,
    A1_Pin,
    A2_Pin,
    A3_Pin,
    A4_Pin,
    A5_Pin,
    A6_Pin,
    A7_Pin,
    A8_Pin,
    A9_Pin,
    A10_Pin,
    A11_Pin,
    A12_Pin,
    A13_Pin,
    A14_Pin,
    A15_Pin,
  };

const int data_pins[] =
  {
    D0_Pin,
    D1_Pin,
    D2_Pin,
    D3_Pin,
    D4_Pin,
    D5_Pin,
    D6_Pin,
    D7_Pin,
  };

// Z80 modes
enum
  {
    MODE_SLAVE,
    NUM_MODES,
  };

// Cycle type
enum
  {
    CYCLE_NONE,
    CYCLE_MEM,
    CYCLE_IO,
  };

// Cycle direction
enum
  {
    CYCLE_DIR_NONE,
    CYCLE_DIR_RD,
    CYCLE_DIR_WR,
  };

// Signal to function
enum
  {
    EV_ASSERT,
    EV_DEASSERT,
  };

enum 
  {
    EV_A_BUSREQ, EV_D_BUSREQ,
    EV_A_BUSACK, EV_D_BUSACK,
    EV_A_MREQ,   EV_D_MREQ,
    EV_A_IOREQ,  EV_D_IOREQ,
    EV_A_WR,     EV_D_WR,
    EV_A_RD,     EV_D_RD,
    EV_A_M1,     EV_D_M1,
    EV_A_RFSH,   EV_D_RFSH,
    EV_A_NMI,    EV_D_NMI,
    EV_A_INT,    EV_D_INT,
    EV_A_WAIT,   EV_D_WAIT,
    EV_A_CLK,    EV_D_CLK,
    EV_A_RES,    EV_D_RES,
  };

// IO addresses
const int IO_ADDR_PIO0    = 0x00;
const int IO_ADDR_PIO0_AD = IO_ADDR_PIO0+0;
const int IO_ADDR_PIO0_BD = IO_ADDR_PIO0+1;
const int IO_ADDR_PIO0_AC = IO_ADDR_PIO0+2;
const int IO_ADDR_PIO0_BC = IO_ADDR_PIO0+3;

const int IO_ADDR_PIO1    = 0x80;
const int IO_ADDR_PIO1_AD = IO_ADDR_PIO1+0;
const int IO_ADDR_PIO1_BD = IO_ADDR_PIO1+1;
const int IO_ADDR_PIO1_AC = IO_ADDR_PIO1+2;
const int IO_ADDR_PIO1_BC = IO_ADDR_PIO1+3;

const int IO_ADDR_CTC   = 0x40;
const int IO_ADDR_BANK  = 0xC0;

// Function that dumps a set of control signals, in a compact form
// The signals to dump are defined in an array


struct
{
  String signame;
  const int pin;
  int   current_state;
  struct
  {
    int     mode;        // Mode
    uint8_t mode_dir;    // The direction we set this line when in this mode
    uint8_t mode_val;    // Default value for this mode
  } modes[1];
  int     assert_ev;   // Assert event
  int     deassert_ev; // Deassert event
}
// The order in this list defines the order in which events will be raised, which affects how th e
// bus state machine will be laid out.
//
  signal_list[] =
    {
      {  "BUSREQ", BUSREQ_Pin, 0, {{MODE_SLAVE, OUTPUT, HIGH}}, EV_A_BUSREQ, EV_D_BUSREQ},
      {  "BUSACK", BUSACK_Pin, 0, {{MODE_SLAVE, INPUT, HIGH}},  EV_A_BUSACK, EV_D_BUSACK},
      {  "  MREQ", MREQ_Pin,   0, {{MODE_SLAVE, INPUT, HIGH}},  EV_A_MREQ, EV_D_MREQ},
      {  " IOREQ", IOREQ_Pin,  0, {{MODE_SLAVE, INPUT, HIGH}},  EV_A_IOREQ, EV_D_IOREQ},
      {  "    WR", WR_Pin,     0, {{MODE_SLAVE, INPUT, HIGH}},  EV_A_WR, EV_D_WR},
      {  "    RD", RD_Pin,     0, {{MODE_SLAVE, INPUT, HIGH}},  EV_A_RD, EV_D_RD},
      {  "    M1", M1_Pin,     0, {{MODE_SLAVE, INPUT, HIGH}},  EV_A_M1, EV_D_M1},
      {  "  RFSH", RFSH_Pin,   0, {{MODE_SLAVE, INPUT, HIGH}},  EV_A_RFSH, EV_D_RFSH},
      {  "   NMI", NMI_Pin,    0, {{MODE_SLAVE, OUTPUT, HIGH}}, EV_A_NMI, EV_D_NMI},
      {  "   INT", INT_Pin,    0, {{MODE_SLAVE, OUTPUT, HIGH}}, EV_A_INT, EV_D_INT},
      {  "  WAIT", WAIT_Pin,   0, {{MODE_SLAVE, OUTPUT, HIGH}}, EV_A_WAIT, EV_D_WAIT},
      {  "   CLK", A_CLK_Pin,  0, {{MODE_SLAVE, OUTPUT, HIGH}}, EV_A_CLK, EV_D_CLK},
      {  "   RES", A_RES_Pin,  0, {{MODE_SLAVE, OUTPUT, HIGH}}, EV_A_RES, EV_D_RES},
      {  "---",    0,          0, {{MODE_SLAVE, INPUT, HIGH}},  0, 0},
    };

// Indices for signals
enum
  {
    SIG_BUSREQ,
    SIG_BUSACK,
    SIG_MREQ,
    SIG_IOREQ,
    SIG_WR,
    SIG_RD,
    SIG_M1,
    SIG_RFSH,
    SIG_NMI,
    SIG_INT,
    SIG_WAIT,
    SIG_CLK,
    SIG_RES,
  };



struct TRANSITION
{
  int stim;
  int next_state;
};

#define NUM_ENTRY 2
#define NUM_TRANS 3

struct STATE
{
int        statenum;
char       *state_name;
FPTR       entry[NUM_ENTRY];
TRANSITION trans[NUM_TRANS];
};

int current_state;

enum 
  {
    STATE_IDLE,
    STATE_OP1,
    STATE_OP2,
    STATE_OP3,
    STATE_OP4,
    STATE_OP5,
    STATE_RFSH1,
    STATE_RFSH2,
    STATE_NUM
  };

// table of instructions

struct INSTRUCTION
{
  boolean valid;
  const char *opcode_name;
  int length;
};

INSTRUCTION instruction[256] =
  {
    {true, "NOP", 1}     //00
    

  };

////////////////////////////////////////////////////////////////////////////////
//
// Bus state machine
//
//


void entry_null();

int inst_addr;
int inst_inst[3];

// opcode1 entry action
void entry_opcode3()
{
  // Store instruction data
  inst_addr = addr_state();

  inst_inst[0] = data_state();
}

const STATE bsm[] =
  {
    { 
      STATE_IDLE,
      "Idle",
      {
	entry_null,
      },
      {
	{EV_A_M1,   STATE_OP1},
	{EV_A_RFSH, STATE_RFSH1},
      }
    },
    { 
      STATE_OP1,
      "Opcode 1",
      {
	entry_null,
      },
      {
	{EV_A_MREQ, STATE_OP2},
      }
    },
    { 
      STATE_OP2,
      "Opcode 2",
      {
	entry_null,
      },
      {
	{EV_A_RD, STATE_OP3},
      }
    },
    { 
      STATE_OP3,
      "Opcode 3",
      {
	entry_opcode3,
      },
      {
	{EV_D_M1, STATE_IDLE},
      }
    },
    { 
      STATE_OP4,
      "Opcode 4",
      {
	entry_null,
      },
      {
	{EV_D_M1, STATE_IDLE},
      }
    },
    { 
      STATE_OP5,
      "Opcode 5",
      {
	entry_null,
      },
      {
	{EV_A_RD, STATE_OP3},
      }
    },
    { 
      STATE_RFSH1,
      "Refresh Cycle",
      {
	entry_null,
      },
      {
	{EV_D_RFSH, STATE_IDLE},
      }
    },
  };

void entry_null()
{
  if (!quiet )
    {
      Serial.print("State: ");
      Serial.println(bsm_state_name());
    }
}

char * bsm_state_name()
{
  int i = find_state_index(current_state);
  return(bsm[i].state_name);
}

////////////////////////////////////////////////////////////////////////////////
//
// general utility functions
//
////////////////////////////////////////////////////////////////////////////////

// Has leading zero support
char hexbuf[5];

char *to_hex(int value, int numdig)
{
  switch(numdig)
    {
    case 4:
      snprintf(hexbuf, sizeof(hexbuf), "%04X", value);
      break;
    case 2:
      snprintf(hexbuf, sizeof(hexbuf), "%02X", value);
      break;
    default:
      snprintf(hexbuf, sizeof(hexbuf), "%04X", value);
      break;
    }
  
  return(hexbuf);
}

////////////////////////////////////////////////////////////////////////////////
//
// Utility functions
//
////////////////////////////////////////////////////////////////////////////////



// Reset the Z80
void reset_z80()
{
  // Drive reset
  assert_signal(SIG_RES);

  // Drive some clocks
  t_state();
  t_state();

  // Release reset
  deassert_signal(SIG_RES);

}

// Do a half t state
int clk = 0;

void half_t_state()
{

  if ( clk )
    {
      digitalWrite(A_CLK_Pin, HIGH);
    }
  else
    {
      digitalWrite(A_CLK_Pin, LOW);
    }
  
  clk = !clk;
}

// Do a clock cycle or T state (high then low)
//

void t_state()
{
  half_t_state();
  half_t_state();
}

// Request and get the bus
void bus_request()
{
  // Bus request low
  assert_signal(SIG_BUSREQ);
  
  // We now clock until busack is low
  // Send one clock with no check
  t_state();
  
  while(  (signal_state("BUSACK") == HIGH) && !Serial.available())
    {
      Serial.println("Clocking..");
      t_state();
    }
  
  // We should have control of the bus now
}

void addr_bus_inputs()
{
  pinMode ( A0_Pin , INPUT );
  pinMode ( A1_Pin , INPUT );
  pinMode ( A2_Pin , INPUT );
  pinMode ( A3_Pin , INPUT );
  pinMode ( A4_Pin , INPUT );
  pinMode ( A5_Pin , INPUT );
  pinMode ( A6_Pin , INPUT );
  pinMode ( A7_Pin , INPUT );
  pinMode ( A8_Pin , INPUT );
  pinMode ( A9_Pin , INPUT );
  pinMode ( A10_Pin, INPUT );
  pinMode ( A11_Pin, INPUT );
  pinMode ( A12_Pin, INPUT );
  pinMode ( A13_Pin, INPUT );
  pinMode ( A14_Pin, INPUT );
  pinMode ( A15_Pin, INPUT );
}

void addr_bus_outputs()
{
  pinMode ( A0_Pin , OUTPUT );
  pinMode ( A1_Pin , OUTPUT );
  pinMode ( A2_Pin , OUTPUT );
  pinMode ( A3_Pin , OUTPUT );
  pinMode ( A4_Pin , OUTPUT );
  pinMode ( A5_Pin , OUTPUT );
  pinMode ( A6_Pin , OUTPUT );
  pinMode ( A7_Pin , OUTPUT );
  pinMode ( A8_Pin , OUTPUT );
  pinMode ( A9_Pin , OUTPUT );
  pinMode ( A10_Pin, OUTPUT );
  pinMode ( A11_Pin, OUTPUT );
  pinMode ( A12_Pin, OUTPUT );
  pinMode ( A13_Pin, OUTPUT );
  pinMode ( A14_Pin, OUTPUT );
  pinMode ( A15_Pin, OUTPUT );
}

void data_bus_inputs()
{
  pinMode ( D0_Pin , INPUT );
  pinMode ( D1_Pin , INPUT );
  pinMode ( D2_Pin , INPUT );
  pinMode ( D3_Pin , INPUT );
  pinMode ( D4_Pin , INPUT );
  pinMode ( D5_Pin , INPUT );
  pinMode ( D6_Pin , INPUT );
  pinMode ( D7_Pin , INPUT );
}

void data_bus_outputs()
{
  pinMode ( D0_Pin , OUTPUT );
  pinMode ( D1_Pin , OUTPUT );
  pinMode ( D2_Pin , OUTPUT );
  pinMode ( D3_Pin , OUTPUT );
  pinMode ( D4_Pin , OUTPUT );
  pinMode ( D5_Pin , OUTPUT );
  pinMode ( D6_Pin , OUTPUT );
  pinMode ( D7_Pin , OUTPUT );
}

// Set the control up to use the Mega to control everything
// This will allow us to single step and so on. It's a way to prevent
// contention on the bus when we start as well.

void initialise_z80_for_control()
{
  // Put processor in slave mode. This selects the Mega Clock and reset lines rather than the onboard
  // signals. The Z80 is now  slave of the Mega
  
  // Put processor in reset
  assert_signal(SIG_RES);

  // Address bus is driven by z80
  addr_bus_inputs();

  // We leave data bus alone for now
  data_bus_inputs();

  // Control signals in slave mode
  set_signals_to_mode(MODE_SLAVE);

  // 
}

////////////////////////////////////////////////////////////////////////////////
//
// State monitoring
//
////////////////////////////////////////////////////////////////////////////////

// Returns address bus state, ie address on bus

unsigned int addr_state()
{
  unsigned int a = 0;

  // Get all the address lines and accumulate the address
  for(int i=15; i>=0; i--)
    {
      // Make room for bit
      a <<= 1;

      // Add bit in
      switch(digitalRead(address_pins[i]))
	{
	case HIGH:
	  a++;
	  break;
	  
	case LOW:
	  break;
	}
    }
  
  return(a);  
}

// Returns data bus state, ie data on bus

unsigned int data_state()
{
  unsigned int a = 0;

  // Get all the data lines and accumulate the data
  for(int i=7; i>=0; i--)
    {
      // Make room for bit
      a <<= 1;

      // Add bit in
      switch(digitalRead(data_pins[i]))
	{
	case HIGH:
	  a++;
	  break;
	  
	case LOW:
	  break;
	}
    }
  
  return(a);  
}

// Sets data bus value
void set_data_state(unsigned int x)
{
  unsigned int a = x;

  // Get all the data lines and accumulate the data
  for(int i=0; i<8; i++)
    {
      // Set bit up
      switch(a & 1)
	{
	case 1:
	  digitalWrite(data_pins[i], HIGH);
	  break;
	  
	case 0:
	  digitalWrite(data_pins[i], LOW);
	  break;
	}
      a >>= 1;
    }
}


// Signal access
void assert_signal(int sig)
{
  // Always active low
  digitalWrite(signal_list[sig].pin, LOW);

  // Update current state
  signal_list[sig].current_state = LOW;
}

void deassert_signal(int sig)
{
  // Always active low
  digitalWrite(signal_list[sig].pin, HIGH);

  // Update current state
  signal_list[sig].current_state = HIGH;
}

// returns state of signal
int signal_state(String signal)
{
  int state = -1;
  
  for(int i=0;;i++)
    {
      if ( signal_list[i].signame == "---" )
	{
	  // Done
	  break;
	}
      
      if( signal_list[i].signame.endsWith(signal) )
	{
	  state = digitalRead(signal_list[i].pin);

	  // Update current state
	  signal_list[i].current_state = state;
	}
    }
  
  return(state);
}

void dump_misc_signals()
{
  if( quiet)
    {
      return;
    }

  if (VERTICAL_LABELS)
    {
      // Length of any name will do, they should all be the same
      int numlines = signal_list[0].signame.length();
      
      // Display labels in vertical form
      for(int l=0;l<numlines;l++)
	{
	  
	  for(int i=0;;i++)
	    {
	      if ( signal_list[i].signame == "---" )
		{
		  // Done
		  break;
		}
	      Serial.print(" ");	  
	      Serial.print( signal_list[i].signame.charAt(l) );
	    }
	  Serial.println("");
	}

      // Labels  printed, now display the value of the signal
      for(int i=0;;i++)
	{
	  
	  Serial.print(" ");

	  if ( signal_list[i].signame == "---" )
	    {
	      // Done
	      break;
	    }
	  
	  int val = digitalRead(signal_list[i].pin );
	  switch(val)
	    {
	    case HIGH:
	      Serial.print("1");
	      break;
	    case LOW:
	      Serial.print("0");
	      break;
	    }
	}
      
      Serial.println("");
      
    }
  else
    {
      for(int i=0;;i++)
	{
	  if ( signal_list[i].signame == "---" )
	    {
	      // Done
	      break;
	    }
	  Serial.print( signal_list[i].signame+": " );

	  int val = digitalRead(signal_list[i].pin );
	  switch(val)
	    {
	    case HIGH:
	      Serial.print("1");
	      break;
	    case LOW:
	      Serial.print("0");
	      break;
	    }
	  Serial.println("");
	}
    }
}

//
// Sets the signals in the signal list to one of the modes it supports.
//

void set_signals_to_mode(int mode)
{
  for(int i=0;;i++)
    {
      if ( signal_list[i].signame == "---" )
	{
	  // Done
	  break;
	}

      for(int m=0; m<NUM_MODES; m++)
	{
	  
	  if ( signal_list[i].modes[m].mode != mode )
	    {
	      continue;
	    }
	  
	  // Set this pin direction up
	  pinMode(signal_list[i].pin, signal_list[i].modes[m].mode_dir);

	  // Write default value if output
	  if ( signal_list[i].modes[m].mode_dir == OUTPUT )
	    {
	      digitalWrite(signal_list[i].pin, signal_list[i].modes[m].mode_val);
	      signal_list[i].current_state = signal_list[i].modes[m].mode_val;
	    }
	}
      
    }
}

//
// Initialises the current state for signals
//

void initialise_signals()
{
  for(int i=0;;i++)
    {
      if ( signal_list[i].signame == "---" )
	{
	  // Done
	  break;
	}

      signal_list[i].current_state  = digitalRead(signal_list[i].pin);
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// generate any bus events that may have occurred
//


void signal_scan()
{
  for(int i=0;;i++)
    {
      if ( signal_list[i].signame == "---" )
	{
	  // Done
	  break;
	}

      // If read state is different to current state then generate events
      int state  = digitalRead(signal_list[i].pin);
      if ( state != signal_list[i].current_state )
	{
	  if ( state == HIGH )
	    {
	      // De-asserted
	      signal_event(i, EV_DEASSERT);
	    }
	  else
	    {
	      // Asserted
	      signal_event(i, EV_ASSERT);
	    }
	  signal_list[i].current_state = state;
	}
      
    }
}


// Generate an event and process it
void signal_event(int sig, int sense)
{
  if ( !quiet )
    {
      Serial.print(signal_list[sig].signame);
    }

  if( sense == EV_ASSERT)
    {
      if ( !quiet )
	{
	  Serial.println(" ASSERT");
	}
      run_bsm(signal_list[sig].assert_ev);
    }
  else 
    {
      if ( !quiet )
	{
	  Serial.println(" DEASSERT");
	}
      run_bsm(signal_list[sig].deassert_ev);
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// Bus state machine
//
//

int find_state_index(int state)
{
  for(int i=0; i<STATE_NUM;i++)
    {
      if( bsm[i].statenum == state )
	{
	  return(i);
	}
    }

  // Error, default to idle
  Serial.println("***ERROR state not found ***");
  return(STATE_IDLE);
}

void run_bsm(int stim)
{
  int current_state_i = find_state_index(current_state);

  // A stimulus has come in, put it into the bsm
  for(int i=0; i<NUM_TRANS; i++)
    {
      if ( stim == bsm[current_state_i].trans[i].stim )
	{
	  // Transition
	  int next_state = bsm[current_state_i].trans[i].next_state;

	  current_state = next_state;

	  for(int j=0; j<NUM_ENTRY; j++)
	    {
	      FPTR fn = bsm[current_state_i].entry[j];
	      if ( fn != 0 )
		{
		  (*fn)();
		}
	    }
	}
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// Commands
//
////////////////////////////////////////////////////////////////////////////////

  
// Grab the z80, ready for other actions like single stepping
// or running test code. The inverse command is 'cmd_free_z80'

void cmd_grab_z80(String cmd)
{
  String arg;
  
  Serial.println("Grabbing Z80");


  //stored_bytes[indx] = arg.toInt();
  initialise_z80_for_control();

  // Now take code from our code array and single step it
  
}

////////////////////////////////////////////////////////////////////////////////
//
// Free the Z80
//
// Set the Mega IO so that the board can run code from flash and RAM using it's
// own clock and reset (reset may have to still be Mega)
//
////////////////////////////////////////////////////////////////////////////////

void cmd_dump_signals()
{
  unsigned int address;
  unsigned int data;

  // Address bus
  address = addr_state();
  
  // Data bus
  data = data_state();
    
  Serial.print("Addr:");  
  Serial.print(to_hex(address, 4));
  Serial.print("  Data:");
  Serial.print(to_hex(data, 2));
  Serial.println("");
  
  // Control signals on bus
  dump_misc_signals();
  
}

////////////////////////////////////////////////////////////////////////////////
//
// Runs the short piece of test Z80 code under full Mega control. This means
// that the Z80 code does not run from the RAM or flash chip on the board, but
// comes from the Mega
////////////////////////////////////////////////////////////////////////////////

// Test Z80 code
  
// Writes to RAM then reads it back
BYTE example_code_ram_chk[] =
  {
    0x3e, 0xaa,          // LOOP:   LD A, 03EH
    0x21, 0x34, 0x82,    //         LD HL 01234H
    0x77,                //         LD (HL), A
    0x7e,                //         LD   A,(HL)
    0x23,                //         INC HL
    0xc3, 0x5, 0x0     //         JR LOOP
  };

// Turns backlight off
BYTE example_code_lcd_bl_off[] =
  {
    0x0e, IO_ADDR_PIO1_BC,    // LOOP:   LD C, 60H
    0x3e, 0xcf,            //         LD  A, Mode3 control word
    0xed, 0x79,            //         OUT (C), A
    0x0e, IO_ADDR_PIO1_BC,    // LOOP:   LD C, 60H
    0x3e, 0xFB,            //         LD  A, Only B2 as output
    0xed, 0x79,            //         OUT (C), A
    0x0e, IO_ADDR_PIO1_BD,    // LOOP:   LD C, 60H
    0x3e, 0x00,            //         LD  A, All output set to 0
    0xed, 0x79,            //         OUT (C), A
    0x18, 0xfe

  };

// Writes some code to RAM then jumps to it
// Code can then be free run

BYTE example_code_ram[] =
  {
0x16, 0x07,              //    LD   D,ENDCODE-RAMCODE   
0x21, 0x00, 0x80,          //     LD   HL,8000H   
0x01, 0x12, 0x00,  //             LD   BC,RAMCODE   
          //   COPYLOOP:      
0x0A,        //             LD   A,(BC)   
0x77,        //             LD   (HL),A   
0x23,        //             INC   HL   
0x03,        //             INC   BC   
0x15,        //             DEC   DE   
0x20, 0xF9,     //             JR   NZ,COPYLOOP   
0xC3, 0x00, 0x80,  //             JP   8000H   
          //   RAMCODE:      
0x21, 0x00, 0x81,  //             LD   HL,8100H   
0x7E,        //   RLOOP:    LD   A,(HL)   
0x23,        //             INC   HL   
0x18, 0xFC,     //             JR   RLOOP   
          //   ENDCODE:      

    
  };

BYTE example_code_bank[] =
  {
    0x0e, 0xc0,          // LOOP:   LD C, 60H
    0x3e, 0xaa,          //         LD  A, AAH
    0xed, 0x79,         //          OUT (C), A
    0xc3, 0x05, 0x00
  };

//--------------------------------------------------------------------------------

struct
{
  const char *desc;
  BYTE  *code;
}
code_list[] =
  {
    {"Copy code to RAM and execute it", example_code_ram},
    {"Write value to bank register",    example_code_bank},
    {"Write then read RAM",             example_code_ram_chk},
    {"Turn LCD shield backlight off",   example_code_lcd_bl_off},
    {"-",                               0},
  };

// Current example code
BYTE *example_code = example_code_ram;


void cmd_set_example_code(String cmd)
{
  String arg = cmd.substring(1);

  int code_i = arg.toInt();
  example_code= code_list[code_i].code;
  
  Serial.print("Example code now '");
  Serial.print(code_list[code_i].desc);
  Serial.println("'");
}

void cmd_show_example_code(String cmd)
{
  Serial.println("Example Code");
  
  for(int i=0; code_list[i].code != 0; i++)
    {
      Serial.print(i);
      Serial.print(": ");
      Serial.println(code_list[i].desc);
    }
}

////////////////////////////////////////////////////////////////////////////////

// In this mode the Mega is essentially a slave of the Z80. We provide the clock and reset signals
// But we have to minitor bus signals to see what the Z80 wants to do. We emulate an address space
// starting at 0000H using the test code array above
// We could emulate RAM too, and that would start at 8000H, as th ereal board does.
//
// A keystroke allows the single stepping to proceed, or other actiosn to be
// issued, such as register dumps

enum {
  STATE_NONE,
  STATE_MEM_RD,
  STATE_MEM_WR,
  STATE_IO_RD,
  STATE_IO_WR,
  STATE_INT_ACK,
};

////////////////////////////////////////////////////////////////////////////////
//
// Runs code at a programming model level, all refresh cycles etc are hidden
//
////////////////////////////////////////////////////////////////////////////////

void cmd_run_test_code()
{
  boolean running = true;
  int state = STATE_NONE;
  int cycle_type = CYCLE_NONE;
  int cycle_dir = CYCLE_DIR_NONE;
  
  // We have a logical address space for the array of code such that the code starts at
  // 0000H, which is the reset vector

  // reset Z80
  reset_z80();

  // Clock and monitor the bus signals to work out what to do
  while( running )
    {
      // Half t states so we can examine all clock transitions
      half_t_state();
      delay(10);


      //Update events
      signal_scan();

      // Now check for things we have to do
      // We really only need respond to memory read/write and IO read/write

      int wr = signal_state("WR");
      int rd = signal_state("RD");
      int mreq = signal_state("MREQ");
      int ioreq = signal_state("IOREQ");
      int m1 = signal_state("M1");

      if ( (rd == HIGH) )
	{
	  // Data bus back to inputs
	  data_bus_inputs();
	}
      
      if ( (wr == LOW) && (mreq == LOW) )
	{
	  cycle_dir = CYCLE_DIR_WR;

	  // Write cycle
	  if (mreq == LOW )
	    {
	      cycle_type = CYCLE_MEM;
	    }
	}

      // Read cycle, put data on the bus, based on address
      if ( (rd == LOW) && (mreq == LOW))
	{
	  cycle_dir = CYCLE_DIR_RD;

	  // Write cycle
	  if (mreq == LOW )
	    {
	      cycle_type = CYCLE_MEM;
	    }

	  // Drive data bus
	  data_bus_outputs();
	  set_data_state(example_code[addr_state() & 0xff]);
	  Serial.print(" ");
	  Serial.print(addr_state(), HEX);
	  Serial.print(": ");
	  Serial.print(example_code[addr_state() & 0xff], HEX);
	}

      // Allow interaction
      
      Serial.println(" (G:Grab Bus  R: release bus)");
      Serial.println(" (return:next q:quit 1:assert reset 0:deassert reset d:dump regs)");
      
      while ( Serial.available()==0)
	{
	}

      boolean cmdloop = true;
      
      while( cmdloop )
	{
	  switch( Serial.read())
	    {
	      
	    case 'G':
	      bus_request();
	      break;

	    case 'R':
	      deassert_signal(SIG_BUSREQ);
	      break;
	      
	    case '1':
	      assert_signal(SIG_RES);
	      break;

	    case '0':
	      deassert_signal(SIG_RES);
	      break;

	    case 'q':
	      running = false;
	      cmdloop = false;
	      break;
	      
	    case '\r':
	      cmdloop = false;
	      break;
	    }
	}
      
    }
}

// Traces test code at the t state level, all cycles are shown

void cmd_trace_test_code()
{
  boolean running = true;
  int state = STATE_NONE;
  int cycle_type = CYCLE_NONE;
  int cycle_dir = CYCLE_DIR_NONE;
  boolean fast_mode = false;       // Skip all output and interaction
  int fast_mode_n = 0;
  int trigger_address = 0x8000;    // trigger when we hit RAm by default
  boolean trigger_on = false;
  
  // We have a logical address space for the array of code such that the code starts at
  // 0000H, which is the reset vector

  // reset Z80
  reset_z80();

  // Clock and monitor the bus signals to work out what to do
  while( running )
    {
      // Half t states so we can examine all clock transitions
      half_t_state();
      delay(5);

      // Dump the status so we can see what's happening
      // Dump the status so we can see what's happening
      if ( !fast_mode )
	{
	  cmd_dump_signals();
	}
      else
	{
	  if( (fast_mode_n % 10)==0 )
	    {
	      Serial.println(fast_mode_n);
	    }
	}


      //Update events
      signal_scan();

      // Now check for things we have to do
      // We really only need respond to memory read/write and IO read/write

      int wr = signal_state("WR");
      int rd = signal_state("RD");
      int mreq = signal_state("MREQ");
      int ioreq = signal_state("IOREQ");

      if ( (rd == HIGH) )
	{
	  // Data bus back to inputs
	  data_bus_inputs();
	}
      
      if ( (wr == LOW) && (mreq == LOW) )
	{
	  cycle_dir = CYCLE_DIR_WR;

	  // Write cycle
	  if (mreq == LOW )
	    {
	      cycle_type = CYCLE_MEM;
	    }
	}

      // Read cycle, put data on the bus, based on address, only emulate FLASH for now
      if ( (rd == LOW) && (mreq == LOW) && (addr_state() < 0x8000))
	{
	  cycle_dir = CYCLE_DIR_RD;
	  
	  // Write cycle
	  if (mreq == LOW )
	    {
	      cycle_type = CYCLE_MEM;
	    }
	  
	  // Drive data bus
	  data_bus_outputs();
	  set_data_state(example_code[addr_state() & 0xff]);
	  if ( !fast_mode )
	    {
	      Serial.print("Putting data on bus ");
	      Serial.print(addr_state(), HEX);
	      Serial.print(" ");
	      Serial.print(example_code[addr_state() & 0xff], HEX);
	    }
	}
      
      // If we are running t states then skip the menu stuff.
      // If there's serial input then we stop the t states
      
      if ( fast_mode )
	{
	  if ( fast_mode_n > 0 )
	    {
	      fast_mode_n--;
	    }

	  // Do we turn fast mode off?
	  if ( fast_mode_n == 0 )
	    {
	      fast_mode = false;
	      quiet = false;
	    }

	  if ( Serial.available()>0 )
	    {
	      // Turn fast mode off if there's a keypress
	      fast_mode = false;
	      quiet = false;
	    }

	  //Turn fast mode off if we hit the trigger address
	  if( (addr_state() == trigger_address) && trigger_on )
	    {
	      fast_mode = false;
	      quiet = false;
	      Serial.print("Trigger address reached (");
	      Serial.print(trigger_address & 0xffff, HEX);
	      Serial.println(")");
	    }
	}
      else
	{
	  // Allow interaction
	  Serial.println("");
	  Serial.print("Breakpoiint:");
	  if ( trigger_on )
	    {
	      Serial.print(trigger_address & 0xffff, HEX);
	    }
	  else
	    {
	      Serial.print("OFF ");
	    }
	  
	  Serial.print(" Bus state:");
	  Serial.println(bsm_state_name());
	  Serial.println(" (G:Grab Bus  R: release bus M:Mega control  F:Free run T:Drive n tstates) b:Breakpoint B:Toggle breakpoint");
	  Serial.println(" (return:next q:quit 1:assert reset 0:deassert reset d:dump regs)");
	  
	  while ( Serial.available() == 0)
	    {
	    }
	  Serial.println(Serial.available());

	  boolean cmdloop = true;
	  
	  while( cmdloop )
	    {
	      if( Serial.available() > 0 )
		{
		  switch( Serial.read())
		    {
		    case 'P':
		      bus_request();
		      
		      // Write to PIO to set D10 as low (turns LCD backlight off)
		      //write_io();
		      break;

		    case 'T':
		      fast_mode = true;
		      fast_mode_n = 100;
		      quiet = true;
		      delay(100);
		      fast_mode_n = get_parameter();
		      cmdloop = false;
		      break;

		    case 'b':
		      delay(100);
		      trigger_address = get_hex_parameter();
		      trigger_on = true;
		      cmdloop=false;
		      break;

		    case 'B':
		      trigger_on = !trigger_on;
		      break;
		      
		    case 'M':
		      // Select Mega control of reset and clock
		      digitalWrite(SW0_Pin, HIGH);
		      digitalWrite(SW1_Pin, LOW);
		      break;
		      
		    case 'F':
		      // Free run
		      digitalWrite(SW0_Pin, LOW);
		      digitalWrite(SW1_Pin, LOW);
		      break;
		      
		    case 'G':
		      bus_request();
		      break;
		      
		    case 'R':
		      deassert_signal(SIG_BUSREQ);
		      break;
		      
		    case '1':
		      assert_signal(SIG_RES);
		  break;
		  
		    case '0':
		      deassert_signal(SIG_RES);
		      break;
		      
		    case 'q':
		      running = false;
		      cmdloop = false;
		      break;
		      
		    case '\r':
		      cmdloop = false;
		      break;
		    }
		}
	      
	    }
	}
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// Command Table
//
////////////////////////////////////////////////////////////////////////////////

// Null cmd function
void cmd_dummy()
{
}

String cmd;
struct
{
  String cmdname;
  CMD_FPTR   handler;
} cmdlist [] =
  {
    {"g",         cmd_grab_z80},
    {"t",         cmd_trace_test_code},
    {"l",         cmd_show_example_code},
    {"s",         cmd_set_example_code},
    {"---",       cmd_dummy},
  };

// Interaction with the Mega from the host PC is through a 'monitor' command line type interface.

const String monitor_cmds = "t: Trace code l:list example code s:set code";
void run_monitor()
{
  char c;
  int i;
  String test;


  
  if( Serial.available()>0 )
    {
      c = Serial.read();
      //Serial.println(c,HEX);
      switch(c)
	{
	case '\r':
	case '\n':
	  // We have a command, process it
	  //Serial.println("'"+cmd+"'");  
	  for(i=0;; i++)
	    {
	      if ( cmdlist[i].cmdname == "---" )
		{
		  //Serial.println("break");
		  break;
		}
		
	      test = cmd.substring(0, (cmdlist[i].cmdname).length());
	      //	      Serial.println("'"+test+"'");
	      if( test == cmdlist[i].cmdname )
		{
		  (*(cmdlist[i].handler))(cmd);
		  Serial.println(monitor_cmds);
		  Serial.print("> ");
		}
	    }

	  cmd = "";
	  break;

	default:
	  cmd += c;
	  break;
	}
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// Gets a numeric parameter form th eserial input
//

int get_parameter()
{
  String s = "";
  int c;
  int n = 0;

  while( Serial.available() > 0 )
    {
      n++;
      c = Serial.read();

      if (isDigit(c) )
	{
	  s += (char)c;
	}
    }

  //Serial.print(n);
  //  Serial.print("par=");

  //Serial.println(s);
  return(s.toInt());
}

// gets a hex parameter from the command string
int get_hex_parameter()
{
  String s = "";
  int c;
  int n = 0;

  while( Serial.available() > 0 )
    {
      n++;
      c = Serial.read();

      if (isHexadecimalDigit(c) )
	{
	  s += (char)c;
	}
    }

  //  Serial.print(n);
  //Serial.print("hex par=");
  //Serial.println(s);

  // Convert from hex to binary
  long l_val = strtol(s.c_str(), NULL, 16);
  
  return((int)l_val);
}

////////////////////////////////////////////////////////////////////////////////
//
// Setup
//
////////////////////////////////////////////////////////////////////////////////

void setup()
{
  
  // Fixed initialisation. These data directions are fixed.

  // Control pins
  pinMode(SW0_Pin, OUTPUT);
  pinMode(SW1_Pin, OUTPUT);

  pinMode(NMI_Pin, OUTPUT);
  digitalWrite(NMI_Pin, 1);

  pinMode(NMI_Pin, OUTPUT);
  digitalWrite(NMI_Pin, 1);
  
  pinMode(A_CLK_Pin, OUTPUT);
  pinMode(A_RES_Pin, OUTPUT);
  
  // Select Mega control of reset and clock
  digitalWrite(SW0_Pin, HIGH);
  digitalWrite(SW1_Pin, LOW);

  initialise_z80_for_control();

  // Initialise signals
  initialise_signals();

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.println("Z80 Shield Monitor");
  Serial.println("    (Set line ending to carriage return)");
  Serial.println(monitor_cmds);
}

////////////////////////////////////////////////////////////////////////////////
//
// Loop
//
////////////////////////////////////////////////////////////////////////////////

void loop()
{
  run_monitor();
}

////////////////////////////////////////////////////////////////////////////////


