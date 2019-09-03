//
// Arduino Mega sketch for the Z80 shield
// Serial monitor based control program
//

typedef unsigned char BYTE;
typedef void (*FPTR)();
typedef void (*CMD_FPTR)(String cmd);

void run_bsm(int stim);
char * bsm_state_name();
unsigned int addr_state();
unsigned int data_state();

// Run in quiet mode
boolean quiet = false;
boolean fast_mode = false;       // Skip all output and interaction

// Current example code
BYTE *example_code;
int example_code_length;


#define VERTICAL_LABELS  0

#define FLASH_ERASE_SECTOR_CMD  0x30
#define FLASH_ERASE_CHIP_CMD    0x10

// Pin definitions

const int D0_Pin      = 30;
const int D1_Pin      = 31;
const int D2_Pin      = 32;
const int D3_Pin      = 33;
const int D4_Pin      = 34;
const int D5_Pin      = 35;
const int D6_Pin      = 36;
const int D7_Pin      = 37;

const int A0_Pin      = 22;
const int A1_Pin      = 23;
const int A2_Pin      = 24;
const int A3_Pin      = 25;
const int A4_Pin      = 26;
const int A5_Pin      = 27;
const int A6_Pin      = 28;
const int A7_Pin      = 29;
const int A8_Pin      = 53;
const int A9_Pin      = 52;
const int A10_Pin     = 51;
const int A11_Pin     = 50;
const int A12_Pin     = 10;
const int A13_Pin     = 11;
const int A14_Pin     = 12;
const int A15_Pin     = 13;

const int BUSREQ_Pin  = 45;
const int BUSACK_Pin  = 44;
const int WR_Pin      = 48;
const int RD_Pin      = 49;
const int MREQ_Pin    = 47;
const int IOREQ_Pin   = 46;
const int HALT_Pin    = 43;
const int NMI_Pin     = 41;
const int INT_Pin     = 42;
const int M1_Pin      = 39;
const int WAIT_Pin    = 40;
const int A_CLK_Pin   = 4;
const int A_RES_Pin   = 5;
const int RFSH_Pin    = 38;
const int SW0_Pin     = 6;
const int SW1_Pin     = 7;

const int MAPRQM_Pin  = 2;
const int MAPRQI_Pin  = 3;

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
    MODE_SLAVE,          // Z80 as a slave, but bus master
    MODE_MEGA_MASTER,    // Mega as bus master
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
    EV_A_MAPRQM, EV_D_MAPRQM,
    EV_A_MAPRQI, EV_D_MAPRQI,
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
  } modes[2];
  int     assert_ev;   // Assert event
  int     deassert_ev; // Deassert event
}
// The order in this list defines the order in which events will be raised, which affects how th e
// bus state machine will be laid out.
//
  signal_list[] =

    {
      {  "BUSREQ", BUSREQ_Pin, 0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, LOW }}, EV_A_BUSREQ, EV_D_BUSREQ},
      {  "BUSACK", BUSACK_Pin, 0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, INPUT,  HIGH}},  EV_A_BUSACK, EV_D_BUSACK},
      {  "  MREQ", MREQ_Pin,   0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_MREQ, EV_D_MREQ},
      {  " IOREQ", IOREQ_Pin,  0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_IOREQ, EV_D_IOREQ},
      {  "    WR", WR_Pin,     0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_WR, EV_D_WR},
      {  "    RD", RD_Pin,     0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_RD, EV_D_RD},
      {  "    M1", M1_Pin,     0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_M1, EV_D_M1},
      {  "  RFSH", RFSH_Pin,   0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_RFSH, EV_D_RFSH},
      {  "   NMI", NMI_Pin,    0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_NMI, EV_D_NMI},
      {  "   INT", INT_Pin,    0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_INT, EV_D_INT},
      {  "  WAIT", WAIT_Pin,   0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_WAIT, EV_D_WAIT},
      {  "   CLK", A_CLK_Pin,  0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_CLK, EV_D_CLK},
      {  "   RES", A_RES_Pin,  0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_RES, EV_D_RES},
      {  "MAPRQM", MAPRQM_Pin, 0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, LOW }}, EV_A_MAPRQM, EV_D_MAPRQM},
      {  "MAPRQI", MAPRQI_Pin, 0, {{MODE_SLAVE, OUTPUT, LOW },{MODE_MEGA_MASTER, OUTPUT, LOW }},  EV_A_MAPRQI, EV_D_MAPRQI},
      {  "---",    0,          0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  0, 0},
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
    SIG_MAPRQM,
    SIG_MAPRQI,
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
    STATE_MEM1,
    STATE_MEM_RD,
    STATE_MEM_WR,
    STATE_MEM_WR_END,
    STATE_MEM_RD_END,
    
    
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
	{EV_A_MREQ, STATE_MEM1},
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
      "Opcode Memory Access",
      {
	entry_mem1,
      },
      {
	{EV_A_RD, STATE_OP3},
      }
    },
    { 
      STATE_OP3,
      "Opcode Read",
      {
	// same as a memory access
	entry_mem_rd,
      },
      {
	{EV_D_RD, STATE_OP4},
      }
    },
    { 
      STATE_OP4,
      "Opcode Read End",
      {
	entry_mem_rd_end,
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
    //------------------------------------------------------------------------------
    // Memory accesses
    //
    // We may want to enable the RAM chip if there's an access to there
    //
    { 
      STATE_MEM1,
      "Memory Access",
      {
	entry_mem1,
      },
      {
	{EV_A_RD, STATE_MEM_RD},
	{EV_A_WR, STATE_MEM_WR},
      }
    },
    { 
      STATE_MEM_RD,
      "Memory Read Access",
      {
	entry_mem_rd,
      },
      {
	{EV_D_MREQ, STATE_MEM_RD_END},
      }
    },
    { 
      STATE_MEM_WR,
      "Memory Write Access",
      {
	entry_null,
      },
      {
	{EV_D_MREQ, STATE_MEM_WR_END},
      }
    },
    { 
      STATE_MEM_RD_END,
      "Memory Read Access END",
      {
	entry_mem_rd_end,
      },
      {
	{EV_D_RD, STATE_IDLE},
      }
    },
    { 
      STATE_MEM_WR_END,
      "Memory Write Access End",
      {
	entry_null,
      },
      {
	{EV_D_WR, STATE_IDLE},
      }
    },
  };

void entry_null()
{
}

void entry_mem1()
{
  
  // If the address is in the range of the RAM chip then we enableit.
  // This is because the Mega doesn't really have enough memory to emulate the RAM, so we use the real chip
  if ( addr_state() >= 0x8000 )
    {
      // It is a RAM access
      // Enable the memory map for a while

      // We don't drive the bus
      data_bus_inputs();
      
      // Turn memory map back on
      digitalWrite(MAPRQM_Pin, LOW);

      if ( !fast_mode )
	{
	  Serial.print("Allowing RAM to put data on bus");
	  Serial.print(addr_state(), HEX);
	  Serial.print(": ");
	  Serial.print(data_state(), HEX);
	}
    }

    // If it's a flash access then we will drive data if WR is asserted
}

// Memory read cycle
void entry_mem_rd()
{
  // We get data from either the example code (flash) or our emulated RAM
  if ( addr_state() < 0x8000 )
    {
      // Emulate flash
      data_bus_outputs();
      
      set_data_state(example_code[addr_state()]);

      if ( !fast_mode )
	{
	  Serial.print("Putting data on bus ");
	  Serial.print(addr_state(), HEX);
	  Serial.print(" ");
	  Serial.print(example_code[addr_state()], HEX);
	}
    }
}

void entry_mem_rd_end()
{
  // release the data bus, either if we have driven it or the RAm chip has, it makes no difference
  data_bus_inputs();

  // release the memory map
  //  if (addr_state() >= 0x8000)
  //{
      // Turn memory off
      digitalWrite(MAPRQM_Pin, HIGH);
      // }
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

void flush_serial()
{
  delay(100);
  while(Serial.available() )
    {
      Serial.read();
    }
  
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
  // We put signals in bus control states
  set_signals_to_mode(MODE_MEGA_MASTER);
  
}

void bus_release()
{
  deassert_signal(SIG_BUSREQ);
  
  // Put bus signals back to slave mode
  set_signals_to_mode(MODE_SLAVE);

  addr_bus_inputs();
  data_bus_inputs();
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

////////////////////////////////////////////////////////////////////////////////
//
// performs bus cycles to access devices etc
//
//
////////////////////////////////////////////////////////////////////////////////

// Signal is MREQ or IOREQ

BYTE read_cycle(int address, int signal)
{
  BYTE data = 0;
  
  // We drive the address bus and read the data bus
  addr_bus_outputs();

  // Put address on bus
  set_addr_state(address);

  // Clock
  t_state();

  // Assert required signals
  assert_signal(signal);
  assert_signal(SIG_RD);

  // Clock again
  t_state();
  t_state();

  // read data
  data_bus_inputs();
  data = data_state();

  // De-assert control
  deassert_signal(signal);
  deassert_signal(SIG_RD);

  // All done, return data
  return(data);
}

void write_cycle(int address, BYTE data, int signal)
{
  // We drive the address bus and write the data bus
  addr_bus_outputs();

  // Put address on bus
  set_addr_state(address);

  // Clock
  t_state();
  t_state();
  
  // Assert required signals
  assert_signal(signal);
  assert_signal(SIG_WR);

  // Clock again
  t_state();
  t_state();

  // Set data up
  data_bus_outputs();
  set_data_state(data);

  t_state();

  // Latch data
  deassert_signal(SIG_WR);  

  t_state();
  
  // De-assert control
  deassert_signal(signal);

  t_state();
// Couple of extra clocks 
  t_state();
  t_state();
  
  // release data bus
  data_bus_inputs();

}

////////////////////////////////////////////////////////////////////////////////
//
// Flash utilities
//
//

void flash_wait_done(int bit_value)
{
  data_bus_inputs();
  int d6 = 0;
  int old_d6 = 0;
  int data = 0;

  // We have to select the flash chip and enable outputs
  
  // Do a read of the flash chip
  data = read_cycle(0, SIG_MREQ);
	
  while( (data & 0x80) != bit_value )
    {
      data = read_cycle(0, SIG_MREQ);

      d6 = (data & 0x40);
      if( d6 != old_d6 )
	{
	}
      old_d6 = d6;
      
      if( Serial.available() )
	{
	  //Serial.println("Keypress break");
	  break;
	}
    }
}

// Writes a byte to the flash at a particular address in a given bank
void flash_write_byte(int bank, int addr, BYTE data)
{
  // Set bank up
  write_cycle(IO_ADDR_BANK, bank, SIG_IOREQ);

  // Perform flash write cycle
  write_cycle(0x5555, 0xAA, SIG_MREQ);
  write_cycle(0x2AAA, 0x55, SIG_MREQ);
  write_cycle(0x5555, 0xA0, SIG_MREQ);
  write_cycle(addr, data, SIG_MREQ);

  // We just wrote when the program completes.
  flash_wait_done(data & 0x80);

  // All done
}

// Erase sector
void flash_erase(int cmd, int sector)
{
  // Perform flash erase cycle
  write_cycle(0x5555, 0xAA, SIG_MREQ);
  write_cycle(0x2AAA, 0x55, SIG_MREQ);
  write_cycle(0x5555, 0x80, SIG_MREQ);
  write_cycle(0x5555, 0xAA, SIG_MREQ);
  write_cycle(0x2AAA, 0x55, SIG_MREQ);
  write_cycle(0x5555, cmd, SIG_MREQ);

  // Wait for completion
  flash_wait_done(0x80);
  
  // All done
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

// drive address bus
void set_addr_state(int address)
{
  // Set all the address lines
  for(int i=15; i>=0; i--)
    {
      // Add bit in
      switch(address & (1 << i))
	{
	default:
	  digitalWrite(address_pins[i], HIGH);
	  break;

	case 0:
	  digitalWrite(address_pins[i], LOW);
	  break;
	}
    }
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
	  current_state_i = find_state_index(current_state);
	  
	  if (!quiet )
	    {
	      Serial.print("State: ");
	      Serial.println(bsm_state_name());
	    }

	  for(int j=0; j<NUM_ENTRY; j++)
	    {
	      FPTR fn = bsm[current_state_i].entry[j];
	      if ( fn != 0 )
		{
		  (*fn)();
		}
	    }
	  break;
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

// Turns backlight off
BYTE example_code_lcd_bl_flash[] =
  {
                    0x31,  0x00,  0x90,  //   0000 : 			LD SP, 9000H 
                           0x0e,  0x83,  //   0003 : 			LD   C, IO_ADDR_PIO1_BC 
                           0x16,  0x81,  //   0005 : 			LD   D, IO_ADDR_PIO1_BD 
                           0x26,  0xfb,  //   0007 : 			LD   H, 0FBH 
                           0x2e,  0x00,  //   0009 : 			LD   L, 00H 
                    0xcd,  0x3f,  0x00,  //   000b : 			CALL PIOINIT 
                           0x0e,  0x83,  //   000e : 			LD   C, IO_ADDR_PIO1_BC 
                           0x16,  0x81,  //   0010 : 			LD   D, IO_ADDR_PIO1_BD 
                           0x26,  0xff,  //   0012 : 			LD   H, 0FFH 
                           0x2e,  0x00,  //   0014 : 			LD   L, 00H 
                    0xcd,  0x3f,  0x00,  //   0016 : 			CALL PIOINIT 
                           0x0e,  0x83,  //   0019 : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0xcf,  //   001b : 			LD A, 0CFH 
                           0xed,  0x79,  //   001d : 			OUT (C),A 
                           0x0e,  0x83,  //   001f : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0xfb,  //   0021 : 			LD A, 0FBH 
                           0xed,  0x79,  //   0023 : 			OUT (C),A 
                           0x0e,  0x81,  //   0025 : 			LD C, IO_ADDR_PIO1_BD 
                           0x3e,  0x00,  //   0027 : 			LD A, 00H 
                           0xed,  0x79,  //   0029 : 			OUT (C),A 
                           0x0e,  0x83,  //   002b : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0xcf,  //   002d : 			LD A, 0CFH 
                           0xed,  0x79,  //   002f : 			OUT (C),A 
                           0x0e,  0x83,  //   0031 : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0xff,  //   0033 : 			LD A, 0FFH 
                           0xed,  0x79,  //   0035 : 			OUT (C),A 
                           0x0e,  0x81,  //   0037 : 			LD C, IO_ADDR_PIO1_BD 
                           0x3e,  0x00,  //   0039 : 			LD A, 00H 
                           0xed,  0x79,  //   003b : 			OUT (C),A 
                           0x18,  0xc4,  //   003d : 			JR START 
                           0x3e,  0xcf,  //   003f : 			LD A, 0CFH 
                           0xed,  0x79,  //   0041 : 			OUT (C), A 
                                  0x7c,  //   0043 : 				LD A, H 
                           0xed,  0x79,  //   0044 : 			OUT (C), A 
                                  0x7d,  //   0046 : 				LD A, L 
                                  0x4a,  //   0047 : 				LD C, D 
                           0xed,  0x79,  //   0048 : 			OUT (C), A 
                                  0xc9,  //   004a : 				RET
  };































BYTE example_code_lcd_slow_flash[] =
  {
                                         //   0000 : IO_ADDR_PIO0:   EQU   80H   
                                         //   0000 : IO_ADDR_PIO0_AD:   EQU   IO_ADDR_PIO0+0   
                                         //   0000 : IO_ADDR_PIO0_BD:   EQU   IO_ADDR_PIO0+1   
                                         //   0000 : IO_ADDR_PIO0_AC:   EQU   IO_ADDR_PIO0+2   
                                         //   0000 : IO_ADDR_PIO0_BC:   EQU   IO_ADDR_PIO0+3   
                                         //   0000 : IO_ADDR_PIO1:   EQU   80H   
                                         //   0000 : IO_ADDR_PIO1_AD:   EQU   IO_ADDR_PIO1+0   
                                         //   0000 : IO_ADDR_PIO1_BD:   EQU   IO_ADDR_PIO1+1   
                                         //   0000 : IO_ADDR_PIO1_AC:   EQU   IO_ADDR_PIO1+2   
                                         //   0000 : IO_ADDR_PIO1_BC:   EQU   IO_ADDR_PIO1+3   
                                         //   0000 : .ORG   0   
                    0x31,  0x00,  0x90,  //   0000 : LD   SP,9000H   
                                         //   0003 : ; 
                           0x0E,  0x83,  //   0003 : START:    LD   C,IO_ADDR_PIO1_BC   
                           0x3E,  0xCF,  //   0005 : LD   A,0CFH   
                           0xED,  0x79,  //   0007 : OUT   (C),A   
                           0x0E,  0x83,  //   0009 : LD   C,IO_ADDR_PIO1_BC   
                           0x3E,  0xFB,  //   000B : LD   A,0FBH   
                           0xED,  0x79,  //   000D : OUT   (C),A   
                           0x0E,  0x83,  //   000F : LD   C,IO_ADDR_PIO1_BC   
                           0x3E,  0x00,  //   0011 : LD   A,00H   
                           0xED,  0x79,  //   0013 : OUT   (C),A   
                    0xCD,  0x2F,  0x00,  //   0015 : CALL   DELAY   
                                         //   0018 : ; 
                           0x0E,  0x83,  //   0018 : LD   C,IO_ADDR_PIO1_BC   
                           0x3E,  0xCF,  //   001A : LD   A,0CFH   
                           0xED,  0x79,  //   001C : OUT   (C),A   
                           0x0E,  0x83,  //   001E : LD   C,IO_ADDR_PIO1_BC   
                           0x3E,  0xFF,  //   0020 : LD   A,0FFH   
                           0xED,  0x79,  //   0022 : OUT   (C),A   
                           0x0E,  0x83,  //   0024 : LD   C,IO_ADDR_PIO1_BC   
                           0x3E,  0x00,  //   0026 : LD   A,00H   
                           0xED,  0x79,  //   0028 : OUT   (C),A   
                    0xCD,  0x2F,  0x00,  //   002A : CALL   DELAY   
                                         //   002D : ; 
                           0x18,  0xD4,  //   002D : JR   START   
                           0x26,  0xFF,  //   002F : DELAY:    LD   H,0FFH   
                                         //   0031 : ; 
                           0x2E,  0x2F,  //   0031 : LOOPH:    LD   L,0FFH   
                                  0x2D,  //   0033 : LOOPL:    DEC   L   
                           0x20,  0xFD,  //   0034 : JR   NZ,LOOPL   
                           0x25,         //   0036 : H   
                           0x20,  0xF8,  //   0037 : JR   NZ,LOOPH   
                                  0xC9,  //   0039 : RET      
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

BYTE example_code_lcd_test[] =
  {
                    0x31,  0x00,  0x90,  //   0000 : 		START:	LD  SP, 9000H 
                           0x0e,  0x82,  //   0003 : 			LD   C, IO_ADDR_PIO1_AC 
                           0x16,  0x80,  //   0005 : 			LD   D, IO_ADDR_PIO1_AD 
                           0x26,  0x0f,  //   0007 : 			LD   H, 0FH 
                           0x2e,  0x00,  //   0009 : 			LD   L, 00H 
                    0xcd,  0x80,  0x01,  //   000b : 			CALL PIOINIT 
                           0x0e,  0x83,  //   000e : 			LD   C, IO_ADDR_PIO1_BC 
                           0x16,  0x81,  //   0010 : 			LD   D, IO_ADDR_PIO1_BD 
                           0x26,  0xfc,  //   0012 : 			LD   H, 0FCH 
                           0x2e,  0x00,  //   0014 : 			LD   L, 00H 
                    0xcd,  0x80,  0x01,  //   0016 : 			CALL PIOINIT 
                           0x0e,  0x02,  //   0019 : 			LD   C, IO_ADDR_PIO0_AC 
                           0x16,  0x00,  //   001b : 			LD   D, IO_ADDR_PIO0_AD 
                           0x26,  0xf8,  //   001d : 			LD   H, 0F8H 
                           0x2e,  0x01,  //   001f : 			LD   L, AD_CS 
                    0xcd,  0x80,  0x01,  //   0021 : 			CALL PIOINIT 
                           0x3e,  0x01,  //   0024 : 			LD   A, AD_CS 
                    0x32,  0x02,  0xa0,  //   0026 : 			LD   (SHADOW_AD), A 
                           0x3e,  0x00,  //   0029 : 			LD   A, 0 
                    0x32,  0x00,  0xa0,  //   002b : 			LD   (SHADOW_A), A 
                           0x3e,  0x00,  //   002e : 			LD   A, 0 
                    0x32,  0x01,  0xa0,  //   0030 : 			LD   (SHADOW_B), A 
                    0xcd,  0x8c,  0x01,  //   0033 : 			CALL RS_LOW 
                    0xcd,  0xac,  0x01,  //   0036 : 			CALL E_LOW 
                    0xcd,  0x75,  0x01,  //   0039 : 		      	CALL DELAY		; 
                           0x16,  0x03,  //   003c : 			LD   D, 03H 
                    0xcd,  0x16,  0x02,  //   003e : 			CALL SDATA4 
                    0xcd,  0x75,  0x01,  //   0041 : 		     	CALL DELAY 
                           0x16,  0x03,  //   0044 : 			LD   D, 03H 
                    0xcd,  0x16,  0x02,  //   0046 : 			CALL SDATA4 
                    0xcd,  0x75,  0x01,  //   0049 : 		     	CALL DELAY 
                           0x16,  0x03,  //   004c : 		       LD   D, 03H 
                    0xcd,  0x16,  0x02,  //   004e : 			CALL SDATA4 
                    0xcd,  0x75,  0x01,  //   0051 : 		     	CALL DELAY 
                           0x16,  0x02,  //   0054 : 		       	LD   D, 02H 
                    0xcd,  0x16,  0x02,  //   0056 : 			CALL SDATA4 
                    0xcd,  0x75,  0x01,  //   0059 : 		     	CALL DELAY 
                           0x16,  0x0e,  //   005c : 			LD   D, 0EH 
                    0xcd,  0xcc,  0x01,  //   005e : 			CALL SDATA8 
                    0xcd,  0x75,  0x01,  //   0061 : 		     	CALL DELAY 
                           0x16,  0x06,  //   0064 : 		        LD   D, 06H 
                    0xcd,  0xcc,  0x01,  //   0066 : 			CALL SDATA8 
                    0xcd,  0x75,  0x01,  //   0069 : 		     	CALL DELAY 
                           0x16,  0x01,  //   006c : 			LD   D, 01H 
                    0xcd,  0xcc,  0x01,  //   006e : 			CALL SDATA8 
                    0xcd,  0x75,  0x01,  //   0071 : 		     	CALL DELAY 
                           0x16,  0x5a,  //   0074 : 			       ld  d, 'Z' 
                    0xcd,  0xf1,  0x01,  //   0076 : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   0079 : 			          	CALL DELAY 
                           0x16,  0x38,  //   007c : 			       ld  d, '8' 
                    0xcd,  0xf1,  0x01,  //   007e : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   0081 : 			          	CALL DELAY 
                           0x16,  0x30,  //   0084 : 			       ld  d, '0' 
                    0xcd,  0xf1,  0x01,  //   0086 : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   0089 : 			          	CALL DELAY 
                           0x16,  0x20,  //   008c : 			       ld  d, ' ' 
                    0xcd,  0xf1,  0x01,  //   008e : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   0091 : 			          	CALL DELAY 
                           0x16,  0x53,  //   0094 : 			       ld  d, 'S' 
                    0xcd,  0xf1,  0x01,  //   0096 : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   0099 : 			          	CALL DELAY 
                           0x16,  0x68,  //   009c : 			       ld  d, 'h' 
                    0xcd,  0xf1,  0x01,  //   009e : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   00a1 : 			          	CALL DELAY 
                           0x16,  0x69,  //   00a4 : 			       ld  d, 'i' 
                    0xcd,  0xf1,  0x01,  //   00a6 : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   00a9 : 			          	CALL DELAY 
                           0x16,  0x65,  //   00ac : 			       ld  d, 'e' 
                    0xcd,  0xf1,  0x01,  //   00ae : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   00b1 : 			          	CALL DELAY 
                           0x16,  0x6c,  //   00b4 : 			       ld  d, 'l' 
                    0xcd,  0xf1,  0x01,  //   00b6 : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   00b9 : 			          	CALL DELAY 
                           0x16,  0x64,  //   00bc : 			       ld  d, 'd' 
                    0xcd,  0xf1,  0x01,  //   00be : 			     call DDATA8 
                    0xcd,  0x75,  0x01,  //   00c1 : 			          	CALL DELAY 
                           0x16,  0x02,  //   00c4 : 		AD:	       ld d, 2 
                    0xcd,  0xcc,  0x01,  //   00c6 : 			       call     SDATA8 
                    0xcd,  0xe9,  0x00,  //   00c9 : 				CALL	 ADSAMPLE 
             0xed,  0x43,  0x00,  0xb0,  //   00cc : 			LD       (ADS0), BC 
                           0x3e,  0x41,  //   00d0 : 			       LD A, 'A' 
                                  0x81,  //   00d2 : 				       ADD    C 
                                  0x57,  //   00d3 : 				       LD D, A 
                    0xcd,  0xf1,  0x01,  //   00d4 : 			 call DDATA8 
                           0x18,  0xeb,  //   00d7 : 			JR     AD 
                           0x18,  0xfe,  //   00d9 : 		LOOP:	JR   LOOP 
                                  0x7e,  //   00db : 			     DSTR: LD A,(HL) 
                           0xfe,  0x00,  //   00dc : 			     CP    0 
                           0x20,  0x01,  //   00de : 			     JR     NZ, CONT 
                                  0xc9,  //   00e0 : 				     RET 
                                  0x7e,  //   00e1 : 				     CONT:  LD A,(HL) 
                                  0x57,  //   00e2 : 				     LD D, A 
                    0xcd,  0xf1,  0x01,  //   00e3 : 			     CALL  DDATA8 
                                  0x23,  //   00e6 : 				     INC HL 
                           0x18,  0xf2,  //   00e7 : 			     JR DSTR 
                    0xcd,  0x60,  0x02,  //   00e9 : 			CALL CLK_HIGH 
                    0xcd,  0x30,  0x02,  //   00ec : 			CALL CS_LOW 
                    0xcd,  0x50,  0x02,  //   00ef : 			CALL CLK_LOW 
                    0xcd,  0x60,  0x02,  //   00f2 : 		        CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   00f5 : 			CALL CLK_LOW 
                    0xcd,  0x60,  0x02,  //   00f8 : 		        CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   00fb : 			CALL CLK_LOW 
                    0xcd,  0x60,  0x02,  //   00fe : 		        CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   0101 : 			CALL CLK_LOW 
                    0xcd,  0x60,  0x02,  //   0104 : 		        CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   0107 : 			CALL CLK_LOW 
                    0xcd,  0x60,  0x02,  //   010a : 		        CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   010d : 			CALL CLK_LOW 
                    0xcd,  0x60,  0x02,  //   0110 : 		        CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   0113 : 			CALL CLK_LOW 
                    0xcd,  0x60,  0x02,  //   0116 : 		        CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   0119 : 		        CALL CLK_LOW 
                    0xcd,  0x80,  0x02,  //   011c : 			CALL DIN_HIGH     	; START bit 
                    0xcd,  0x60,  0x02,  //   011f : 		        CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   0122 : 			CALL CLK_LOW 
                    0xcd,  0x80,  0x02,  //   0125 : 			CALL DIN_HIGH           ;SGL 
                    0xcd,  0x60,  0x02,  //   0128 : 			CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   012b : 			CALL CLK_LOW 
                    0xcd,  0x70,  0x02,  //   012e : 			CALL DIN_LOW 
                    0xcd,  0x60,  0x02,  //   0131 : 			CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   0134 : 			CALL CLK_LOW 
                    0xcd,  0x70,  0x02,  //   0137 : 			CALL DIN_LOW 
                    0xcd,  0x60,  0x02,  //   013a : 			CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   013d : 			CALL CLK_LOW 
                    0xcd,  0x70,  0x02,  //   0140 : 			CALL DIN_LOW 
                    0xcd,  0x60,  0x02,  //   0143 : 			CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   0146 : 			CALL CLK_LOW 
                    0xcd,  0x70,  0x02,  //   0149 : 			CALL DIN_LOW 
                    0xcd,  0x60,  0x02,  //   014c : 			CALL CLK_HIGH 
                    0xcd,  0x50,  0x02,  //   014f : 			CALL CLK_LOW 
                    0xcd,  0x70,  0x02,  //   0152 : 			CALL DIN_LOW 
                    0xcd,  0x60,  0x02,  //   0155 : 			CALL CLK_HIGH 
                           0x16,  0x0a,  //   0158 : 			LD	D, 10 
                    0x01,  0x00,  0x00,  //   015a : 			LD      BC, 0    	;result 
                    0xcd,  0x60,  0x02,  //   015d : 		        CALL  CLK_HIGH 
                    0xcd,  0x90,  0x02,  //   0160 : 			CALL GET_DATABIT 
                           0xcb,  0x21,  //   0163 : 			SLA     C 
                           0xcb,  0x10,  //   0165 : 			RL      B 
                           0xe6,  0x01,  //   0167 : 			AND     1 
                                  0xb1,  //   0169 : 				OR      C 
                                  0x4f,  //   016a : 				LD      C, A 
                    0xcd,  0x50,  0x02,  //   016b : 			CALL   CLK_LOW 
                                  0x15,  //   016e : 				DEC D 
                           0x20,  0xec,  //   016f : 			JR   NZ, GETLOOP 
                    0xcd,  0x40,  0x02,  //   0171 : 			CALL CS_HIGH 
                                  0xc9,  //   0174 : 				RET 
                           0x26,  0x02,  //   0175 : 			LD   H,02H    
                           0x2e,  0xff,  //   0177 : 		LOOPH:    LD   L,0FFH    
                                  0x2d,  //   0179 : 			LOOPL:    DEC   L    
                           0x20,  0xfd,  //   017a : 		          JR   NZ,LOOPL    
                                  0x25,  //   017c : 			          DEC   H    
                           0x20,  0xf8,  //   017d : 		          JR   NZ,LOOPH    
                                  0xc9,  //   017f : 			          RET       
                           0x3e,  0xcf,  //   0180 : 			LD A, 0CFH 
                           0xed,  0x79,  //   0182 : 			OUT (C), A 
                                  0x7c,  //   0184 : 				LD A, H 
                           0xed,  0x79,  //   0185 : 			OUT (C), A 
                                  0x7d,  //   0187 : 				LD A, L 
                                  0x4a,  //   0188 : 				LD C, D 
                           0xed,  0x79,  //   0189 : 			OUT (C), A 
                                  0xc9,  //   018b : 				RET 
                                  0xc5,  //   018c : 			RS_LOW:PUSH   BC 
                                  0xd5,  //   018d : 			      PUSH    DE 
                           0x0e,  0x81,  //   018e : 			LD    C, IO_ADDR_PIO1_BD 
                    0x21,  0x01,  0xa0,  //   0190 : 			LD    HL,SHADOW_B 
                                  0x7e,  //   0193 : 				LD    A, (HL) 
                           0xcb,  0x87,  //   0194 : 			RES   0, A 
                                  0x77,  //   0196 : 				LD    (HL), A 
                           0xed,  0x79,  //   0197 : 			OUT   (C), A 
                                  0xd1,  //   0199 : 				POP   DE 
                                  0xc1,  //   019a : 				POP   BC 
                                  0xc9,  //   019b : 				RET 
                                  0xc5,  //   019c : 			RS_HIGH:PUSH  BC 
                                  0xd5,  //   019d : 			      PUSH    DE 
                           0x0e,  0x81,  //   019e : 		        LD    C, IO_ADDR_PIO1_BD 
                    0x21,  0x01,  0xa0,  //   01a0 : 			LD    HL,SHADOW_B 
                                  0x7e,  //   01a3 : 				LD    A, (HL) 
                           0xcb,  0xc7,  //   01a4 : 			SET   0, A 
                                  0x77,  //   01a6 : 				LD    (HL), A 
                           0xed,  0x79,  //   01a7 : 			OUT   (C), A 
                                  0xd1,  //   01a9 : 				POP   DE	 
                                  0xc1,  //   01aa : 				POP   BC 
                                  0xc9,  //   01ab : 				RET 
                                  0xc5,  //   01ac : 			E_LOW:  PUSH  BC 
                                  0xd5,  //   01ad : 			        PUSH    DE 
                           0x0e,  0x81,  //   01ae : 			LD    C, IO_ADDR_PIO1_BD 
                    0x21,  0x01,  0xa0,  //   01b0 : 			LD    HL,SHADOW_B 
                                  0x7e,  //   01b3 : 				LD    A, (HL) 
                           0xcb,  0x8f,  //   01b4 : 			RES   1, A 
                                  0x77,  //   01b6 : 				LD    (HL), A 
                           0xed,  0x79,  //   01b7 : 			OUT   (C), A 
                                  0xd1,  //   01b9 : 				POP   DE 
                                  0xc1,  //   01ba : 				POP   BC 
                                  0xc9,  //   01bb : 				RET 
                                  0xc5,  //   01bc : 			E_HIGH: PUSH BC 
                                  0xd5,  //   01bd : 			      PUSH    DE 
                           0x0e,  0x81,  //   01be : 			LD    C, IO_ADDR_PIO1_BD 
                    0x21,  0x01,  0xa0,  //   01c0 : 			LD    HL,SHADOW_B 
                                  0x7e,  //   01c3 : 				LD    A, (HL) 
                           0xcb,  0xcf,  //   01c4 : 			SET   1, A 
                                  0x77,  //   01c6 : 				LD    (HL), A 
                           0xed,  0x79,  //   01c7 : 			OUT   (C), A 
                                  0xd1,  //   01c9 : 				POP   DE	 
                                  0xc1,  //   01ca : 				POP   BC 
                                  0xc9,  //   01cb : 				RET 
                           0x0e,  0x80,  //   01cc : 			LD    C, IO_ADDR_PIO1_AD 
                    0xcd,  0x8c,  0x01,  //   01ce : 			CALL  RS_LOW 
                    0xcd,  0xbc,  0x01,  //   01d1 : 			CALL  E_HIGH 
                                  0x7a,  //   01d4 : 				LD    A, D 
                           0x0e,  0x80,  //   01d5 : 			LD    C, IO_ADDR_PIO1_AD	 
                           0xed,  0x79,  //   01d7 : 			OUT   (C), A 
                    0xcd,  0xac,  0x01,  //   01d9 : 			CALL  E_LOW 
                    0xcd,  0xbc,  0x01,  //   01dc : 			CALL  E_HIGH 
                                  0x7a,  //   01df : 				LD    A, D 
                           0xcb,  0x27,  //   01e0 : 			SLA   A 
                           0xcb,  0x27,  //   01e2 : 			SLA   A 
                           0xcb,  0x27,  //   01e4 : 			SLA   A 
                           0xcb,  0x27,  //   01e6 : 			SLA   A 
                           0xed,  0x79,  //   01e8 : 			OUT   (C), A 
                    0xcd,  0xac,  0x01,  //   01ea : 			CALL  E_LOW 
                    0xcd,  0x9c,  0x01,  //   01ed : 			CALL  RS_HIGH 
                                  0xc9,  //   01f0 : 				RET 
                           0x0e,  0x80,  //   01f1 : 			LD    C, IO_ADDR_PIO1_AD 
                    0xcd,  0x9c,  0x01,  //   01f3 : 			CALL  RS_HIGH 
                    0xcd,  0xbc,  0x01,  //   01f6 : 			CALL  E_HIGH 
                                  0x7a,  //   01f9 : 				LD    A, D 
                           0x0e,  0x80,  //   01fa : 			LD    C, IO_ADDR_PIO1_AD	 
                           0xed,  0x79,  //   01fc : 			OUT   (C), A 
                    0xcd,  0xac,  0x01,  //   01fe : 			CALL  E_LOW 
                    0xcd,  0xbc,  0x01,  //   0201 : 			CALL  E_HIGH 
                                  0x7a,  //   0204 : 				LD    A, D 
                           0xcb,  0x27,  //   0205 : 			SLA   A 
                           0xcb,  0x27,  //   0207 : 			SLA   A 
                           0xcb,  0x27,  //   0209 : 			SLA   A 
                           0xcb,  0x27,  //   020b : 			SLA   A 
                           0xed,  0x79,  //   020d : 			OUT   (C), A 
                    0xcd,  0xac,  0x01,  //   020f : 			CALL  E_LOW 
                    0xcd,  0x9c,  0x01,  //   0212 : 			CALL  RS_HIGH 
                                  0xc9,  //   0215 : 				RET 
                    0xcd,  0x8c,  0x01,  //   0216 : 		SDATA4: CALL  RS_LOW 
                    0xcd,  0xbc,  0x01,  //   0219 : 			CALL  E_HIGH 
                                  0x7a,  //   021c : 				LD    A, D 
                           0xcb,  0x27,  //   021d : 			SLA   A 
                           0xcb,  0x27,  //   021f : 			SLA   A 
                           0xcb,  0x27,  //   0221 : 			SLA   A 
                           0xcb,  0x27,  //   0223 : 			SLA   A 
                           0x0e,  0x80,  //   0225 : 			LD    C, IO_ADDR_PIO1_AD 
                           0xed,  0x79,  //   0227 : 			OUT   (C), A 
                    0xcd,  0xac,  0x01,  //   0229 : 			CALL  E_LOW 
                    0xcd,  0x9c,  0x01,  //   022c : 			CALL  RS_HIGH 
                                  0xc9,  //   022f : 				RET 
                                  0xc5,  //   0230 : 			CS_LOW: PUSH    BC 
                                  0xd5,  //   0231 : 			        PUSH    DE 
                           0x0e,  0x00,  //   0232 : 			LD    C, IO_ADDR_PIO0_AD 
                    0x21,  0x02,  0xa0,  //   0234 : 			LD    HL,SHADOW_AD 
                                  0x7e,  //   0237 : 				LD    A, (HL) 
                           0xcb,  0x87,  //   0238 : 			RES   0, A 
                                  0x77,  //   023a : 				LD    (HL), A 
                           0xed,  0x79,  //   023b : 			OUT   (C), A 
                                  0xd1,  //   023d : 				POP   DE 
                                  0xc1,  //   023e : 				POP   BC 
                                  0xc9,  //   023f : 				RET 
                                  0xc5,  //   0240 : 			CS_HIGH: PUSH    BC 
                                  0xd5,  //   0241 : 			        PUSH    DE 
                           0x0e,  0x00,  //   0242 : 			LD    C, IO_ADDR_PIO0_AD 
                    0x21,  0x02,  0xa0,  //   0244 : 			LD    HL,SHADOW_AD 
                                  0x7e,  //   0247 : 				LD    A, (HL) 
                           0xcb,  0xc7,  //   0248 : 			SET   0, A 
                                  0x77,  //   024a : 				LD    (HL), A 
                           0xed,  0x79,  //   024b : 			OUT   (C), A 
                                  0xd1,  //   024d : 				POP   DE 
                                  0xc1,  //   024e : 				POP   BC 
                                  0xc9,  //   024f : 				RET 
                                  0xc5,  //   0250 : 			CLK_LOW: PUSH    BC 
                                  0xd5,  //   0251 : 			        PUSH    DE 
                           0x0e,  0x00,  //   0252 : 			LD    C, IO_ADDR_PIO0_AD 
                    0x21,  0x02,  0xa0,  //   0254 : 			LD    HL,SHADOW_AD 
                                  0x7e,  //   0257 : 				LD    A, (HL) 
                           0xcb,  0x8f,  //   0258 : 			RES   1, A 
                                  0x77,  //   025a : 				LD    (HL), A 
                           0xed,  0x79,  //   025b : 			OUT   (C), A 
                                  0xd1,  //   025d : 				POP   DE 
                                  0xc1,  //   025e : 				POP   BC 
                                  0xc9,  //   025f : 				RET 
                                  0xc5,  //   0260 : 			CLK_HIGH: PUSH    BC 
                                  0xd5,  //   0261 : 			        PUSH    DE 
                           0x0e,  0x00,  //   0262 : 			LD    C, IO_ADDR_PIO0_AD 
                    0x21,  0x02,  0xa0,  //   0264 : 			LD    HL,SHADOW_AD 
                                  0x7e,  //   0267 : 				LD    A, (HL) 
                           0xcb,  0xcf,  //   0268 : 			SET   1, A 
                                  0x77,  //   026a : 				LD    (HL), A 
                           0xed,  0x79,  //   026b : 			OUT   (C), A 
                                  0xd1,  //   026d : 				POP   DE 
                                  0xc1,  //   026e : 				POP   BC 
                                  0xc9,  //   026f : 				RET 
                                  0xc5,  //   0270 : 			DIN_LOW: PUSH    BC 
                                  0xd5,  //   0271 : 			        PUSH    DE 
                           0x0e,  0x00,  //   0272 : 			LD    C, IO_ADDR_PIO0_AD 
                    0x21,  0x02,  0xa0,  //   0274 : 			LD    HL,SHADOW_AD 
                                  0x7e,  //   0277 : 				LD    A, (HL) 
                           0xcb,  0x97,  //   0278 : 			RES   2, A 
                                  0x77,  //   027a : 				LD    (HL), A 
                           0xed,  0x79,  //   027b : 			OUT   (C), A 
                                  0xd1,  //   027d : 				POP   DE 
                                  0xc1,  //   027e : 				POP   BC 
                                  0xc9,  //   027f : 				RET 
                                  0xc5,  //   0280 : 			DIN_HIGH: PUSH    BC 
                                  0xd5,  //   0281 : 			        PUSH    DE 
                           0x0e,  0x00,  //   0282 : 			LD    C, IO_ADDR_PIO0_AD 
                    0x21,  0x02,  0xa0,  //   0284 : 			LD    HL,SHADOW_AD 
                                  0x7e,  //   0287 : 				LD    A, (HL) 
                           0xcb,  0xd7,  //   0288 : 			SET   2, A 
                                  0x77,  //   028a : 				LD    (HL), A 
                           0xed,  0x79,  //   028b : 			OUT   (C), A 
                                  0xd1,  //   028d : 				POP   DE 
                                  0xc1,  //   028e : 				POP   BC 
                                  0xc9,  //   028f : 				RET 
                                  0xc5,  //   0290 : 			GET_DATABIT: PUSH    BC 
                                  0xd5,  //   0291 : 			        PUSH    DE 
                           0x0e,  0x00,  //   0292 : 			LD    C, IO_ADDR_PIO0_AD 
                           0xed,  0x78,  //   0294 : 			IN     A, (C) 
                           0xe6,  0x08,  //   0296 : 			AND    08H 
                           0xcb,  0x3f,  //   0298 : 			SRL    A 
                           0xcb,  0x3f,  //   029a : 			SRL    A 
                           0xcb,  0x3f,  //   029c : 			SRL    A	 
                                  0xd1,  //   029e : 				POP   DE 
                                  0xc1,  //   029f : 				POP   BC 
                                  0xc9,  //   02a0 : 				RET 
  };















































































//--------------------------------------------------------------------------------

struct
{
  const char *desc;
  BYTE  *code;
  int length;
}
  code_list[] =
    {
      {"Copy code to RAM and execute it", example_code_ram,          sizeof(example_code_ram)},
      {"Write value to bank register",    example_code_bank,         sizeof(example_code_bank)},
      {"Write then read RAM",             example_code_ram_chk,      sizeof(example_code_ram_chk)},
      {"Turn LCD shield backlight off",   example_code_lcd_bl_off,   sizeof(example_code_lcd_bl_off)},
      {"Flash turn LCD shield backlight", example_code_lcd_bl_flash, sizeof(example_code_lcd_bl_flash)},
      {"Slow Flash turn LCD shield backlight", example_code_lcd_slow_flash, sizeof(example_code_lcd_slow_flash)},
      {"LCD test",                             example_code_lcd_test, sizeof(example_code_lcd_test)},
      {"-",                               0,                         0},
    };


void cmd_set_example_code(String cmd)
{
  String arg = cmd.substring(1);

  int code_i = arg.toInt();
  example_code        = code_list[code_i].code;
  example_code_length = code_list[code_i].length;
  
  Serial.print("Example code now '");
  Serial.print(code_list[code_i].desc);
  Serial.print("  len:");
  Serial.print(example_code_length);
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

////////////////////////////////////////////////////////////////////////////////
//
// Runs code at a programming model level, all refresh cycles etc are hidden
//
////////////////////////////////////////////////////////////////////////////////

void cmd_run_test_code(String cmd)
{
}

// Traces test code at the t state level, all cycles are shown

void cmd_trace_test_code(String cmd)
{
  boolean running = true;
  int cycle_type = CYCLE_NONE;
  int cycle_dir = CYCLE_DIR_NONE;
  int cycle_n = 0;
  
  int fast_mode_n = 0;
  int trigger_address = 0x8000;    // trigger when we hit RAm by default
  boolean trigger_on = false;
  
  // We have a logical address space for the array of code such that the code starts at
  // 0000H, which is the reset vector

  // reset Z80
  reset_z80();
  
  // Enable IO and emulate memory
  // We will allow the RAM to provide RAM data
  
  deassert_signal(SIG_MAPRQM); 
  assert_signal(SIG_MAPRQI);

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
	  if( (fast_mode_n % 100)==0 )
	    {
	      Serial.println(fast_mode_n);
	    }
	  if( fast_mode_n == -1 ) 
	    {
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
      int maprqm = signal_state("MAPRQM");
      int last_addr = -1;
      
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
	}
      
      // If we are running t states then skip the menu stuff.
      // If there's serial input then we stop the t states
      
      if ( fast_mode )
	{
	  // Give an indication of execution by displaying every
	  #if 0
	  if( addr_state()!=last_addr  )
	    {
	      Serial.println(to_hex(addr_state(), 4));
	    }
	  last_addr = addr_state();
#endif			   
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
	  Serial.print("Breakpoint:");
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
	  Serial.println(" (G:Grab Bus  R: release bus M:Mega control  F:Free run t:Drive n tstates) b:Breakpoint B:Toggle breakpoint)");
	  Serial.println(" (I:Request IO Map i:Release IO map J:Request MEM Map j:Release MEM map)");
	  Serial.println(" (return:next q:quit 1:assert reset 0:deassert reset d:dump regs f:Run forever)");
	  
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
		    case 't':
		      fast_mode = true;
		      fast_mode_n = 100;
		      quiet = true;
		      delay(100);
		      fast_mode_n = get_parameter();
		      cmdloop = false;
		      break;

		    case 'f':
		      fast_mode = true;
		      fast_mode_n = -1;
		      quiet = true;
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

		    case 'I':
		      digitalWrite(MAPRQI_Pin, HIGH);
		      break;

		    case 'i':
		      digitalWrite(MAPRQI_Pin, LOW);
		      break;

		    case 'J':
		      digitalWrite(MAPRQM_Pin, HIGH);
		      break;

		    case 'j':
		      digitalWrite(MAPRQM_Pin, LOW);
		      break;
		      
		    case 'G':
		      bus_request();
		      break;
		      
		    case 'R':
		      bus_release();

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
// Flash menu
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//
// Memory monitor
//
//
////////////////////////////////////////////////////////////////////////////////



void cmd_memory(String cmd)
{

  boolean running = true;
  boolean cmdloop = true;
  int address = 0;
  int working_address = 0;
  int working_space = SIG_MREQ;
  
  // Grab the bus from the Z80 as we are going to do memory accesses ourselves
  bus_request();

  // Enable both memory maps
  assert_signal(SIG_MAPRQM);
  assert_signal(SIG_MAPRQI);
  
  // Clock and monitor the bus signals to work out what to do
  while( running )
    {
      cmd_dump_signals();

      //Update events
      signal_scan();

      // Allow interaction
      Serial.print("Working address: ");
      Serial.print(working_address, HEX);

      Serial.print(" Space:");
      switch(working_space)
	{
	case SIG_MREQ:
	  Serial.print("MEM ");
	  break;
	  
	case SIG_IOREQ:
	  Serial.print("IO  ");
	  break;
	  
	default:
	  Serial.print("??? ");
	  break;
	}
      
      Serial.print("");

      
      Serial.print(" Bus state:");
      Serial.println(bsm_state_name());
      Serial.println(" (r:Display memory  a: Set address w:write byte e:Erase flash sector E:Erase chip)");
      Serial.println(" (m:Mem space i:IO space b:Set bank X:write example code to 0000 Y:write code to all banks)");

      Serial.println(" (return:next q:quit)");
      
      while ( Serial.available() == 0)
	{
	}
      Serial.println(Serial.available());

      cmdloop=true;

      while( cmdloop )
	{
	  if( Serial.available() > 0 )
	    {
	      switch( Serial.read())
		{
		case 'r':
		  // display memory at address
		  char ads[10];
		  BYTE d;
		  address=working_address;
		  for(int i=0; i<256; i++)
		    {
		      if( (i%16)==0)
			{
			  Serial.println("");

			  sprintf(ads, "%04X", address);
			  Serial.print(ads);
			  Serial.print(": ");
			}
		      d = read_cycle(address, working_space);
		      sprintf(ads, "%02X ", d);
		      Serial.print(ads);
		      address++;
		    }
		  Serial.println("");
		  break;

		case 'w':
		  delay(100);
		  write_cycle(working_address, get_hex_parameter(), working_space);
		  break;

		case 'm':
		  working_space = SIG_MREQ;
		  break;

		case 'i':
		  working_space = SIG_IOREQ;
		  break;
		  
		case 'a':
		  // Set address to manipulate
		  delay(100);
		  working_address = get_hex_parameter();

		  cmdloop=false;
		  break;

		case 'b':
		  // Write a bank value to bank register
		  delay(100);

		  write_cycle(IO_ADDR_BANK, get_hex_parameter(), SIG_IOREQ);
		  cmdloop=false;
		  break;

		case 'e':
		  // Erase a sector
		  delay(100);

		  Serial.println("Starting erase...");
		  flash_erase(FLASH_ERASE_SECTOR_CMD, get_hex_parameter());
		  Serial.println("done.");
		  cmdloop=false;
		  break;

		case 'E':
		  flush_serial();
		  
		  // Erase a sector
		  Serial.println("Starting chip erase...");
		  flash_erase(FLASH_ERASE_CHIP_CMD, 0x5555);
		  Serial.println("done.");
		  cmdloop=false;
		  break;

		case 'X':
		  //flush_serial();
		  // Take the example code and write it to flash
		  for(int i=0; i<example_code_length; i++)
		    {
		      flash_write_byte(0, i, example_code[i]);
		    }
		  break;

		case 'Y':
		  //flush_serial();
		  // Take the example code and write it to all banks of flash 
		  for(int b=0; b<16;b++)
		    {
		      Serial.print("Writing to bank ");
		      Serial.println(b);
		      for(int i=0; i<example_code_length; i++)
			{
			  flash_write_byte(b, i, example_code[i]);
			}
		    }
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

  addr_bus_inputs();
  data_bus_inputs();

  //Release bus
  bus_release();
}


////////////////////////////////////////////////////////////////////////////////
//
// Command Table
//
////////////////////////////////////////////////////////////////////////////////

// Null cmd function
void cmd_dummy(String cmd)
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
    {"m",         cmd_memory},
    {"---",       cmd_dummy},
  };

// Interaction with the Mega from the host PC is through a 'monitor' command line type interface.

const String monitor_cmds = "t: Trace code l:list example code s:set code m:memory";
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

  example_code = example_code_ram;
  example_code_length = sizeof(example_code_ram);
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














































































































