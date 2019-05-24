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

const int MAPRQM_Pin  = 36;
const int MAPRQI_Pin  = 38;

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
  Serial.println("mem1");
  
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
  Serial.print("entry_mem_rd");
  
  // We get data from either the example code (flash) or our emulated RAM
  if ( addr_state() < 0x8000 )
    {
      // Emulate flash
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
}

void entry_mem_rd_end()
{
  // release the data bus, either if we have driven it or the RAm chip has, it makes no difference
  data_bus_inputs();

  // release the memory map
  if (addr_state() >= 0x8000)
    {
      // Turn memory off
      digitalWrite(MAPRQM_Pin, HIGH);
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
  
  // Do a read of he flash chip
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
	  Serial.println("Keypress break");
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
                           0x0e,  0x83,  //   0003 : 		START:	LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0xcf,  //   0005 : 			LD A, 0CFH 
                           0xed,  0x79,  //   0007 : 			OUT (C),A 
                           0x0e,  0x83,  //   0009 : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0xfb,  //   000b : 			LD A, 0FBH 
                           0xed,  0x79,  //   000d : 			OUT (C),A 
                           0x0e,  0x83,  //   000f : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0x00,  //   0011 : 			LD A, 00H 
                           0xed,  0x79,  //   0013 : 			OUT (C),A 
                           0x0e,  0x83,  //   0015 : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0xcf,  //   0017 : 			LD A, 0CFH 
                           0xed,  0x79,  //   0019 : 			OUT (C),A 
                           0x0e,  0x83,  //   001b : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0xff,  //   001d : 			LD A, 0FFH 
                           0xed,  0x79,  //   001f : 			OUT (C),A 
                           0x0e,  0x83,  //   0021 : 			LD C, IO_ADDR_PIO1_BC 
                           0x3e,  0x00,  //   0023 : 			LD A, 00H 
                           0xed,  0x79,  //   0025 : 			OUT (C),A 
                           0x18,  0xda,  //   0027 : 			JR START 
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
                    0xcd,  0x5d,  0x00,  //   000b : 			CALL PIOINIT 
                           0x0e,  0x83,  //   000e : 			LD   C, IO_ADDR_PIO1_BC 
                           0x16,  0x81,  //   0010 : 			LD   D, IO_ADDR_PIO1_BD 
                           0x26,  0xfc,  //   0012 : 			LD   H, 0FCH 
                           0x2e,  0x00,  //   0014 : 			LD   L, 00H 
                    0xcd,  0x5d,  0x00,  //   0016 : 			CALL PIOINIT 
                           0x3e,  0x00,  //   0019 : 			LD   A, 0 
                    0x32,  0x00,  0xa0,  //   001b : 			LD   (SHADOW_A), A 
                           0x3e,  0x00,  //   001e : 			LD   A, 0 
                    0x32,  0x01,  0xa0,  //   0020 : 			LD   (SHADOW_B), A 
                    0xcd,  0x77,  0x00,  //   0023 : 			CALL RS_HIGH 
                    0xcd,  0x85,  0x00,  //   0026 : 			CALL E_LOW 
                    0xcd,  0x52,  0x00,  //   0029 : 		  	CALL DELAY 
                           0x16,  0x02,  //   002c : 			LD   D, 02H 
                    0xcd,  0xd1,  0x00,  //   002e : 			CALL SDATA4 
                    0xcd,  0x52,  0x00,  //   0031 : 			CALL DELAY 
                           0x16,  0x02,  //   0034 : 			LD   D, 02H 
                    0xcd,  0xd1,  0x00,  //   0036 : 			CALL SDATA4 
                    0xcd,  0x52,  0x00,  //   0039 : 			CALL DELAY 
                           0x16,  0x02,  //   003c : 			LD   D, 02H 
                    0xcd,  0xd1,  0x00,  //   003e : 			CALL SDATA4 
                           0x16,  0x02,  //   0041 : 			LD   D, 02H 
                    0xcd,  0xd1,  0x00,  //   0043 : 			CALL SDATA4 
                           0x16,  0x28,  //   0046 : 			LD   D, 28H 
                    0xcd,  0xa1,  0x00,  //   0048 : 			CALL SDATA8 
                           0x16,  0x0f,  //   004b : 		        LD   D, 0FH 
                    0xcd,  0xa1,  0x00,  //   004d : 			CALL SDATA8 
                           0x18,  0xae,  //   0050 : 		LOOP:	JR   START 
                           0x26,  0xff,  //   0052 : 		DELAY:    LD   H,0FFH    
                           0x2e,  0xff,  //   0054 : 		LOOPH:    LD   L,0FFH    
                                  0x2d,  //   0056 : 			LOOPL:    DEC   L    
                           0x20,  0xfd,  //   0057 : 		          JR   NZ,LOOPL    
                                  0x25,  //   0059 : 			          DEC   H    
                           0x20,  0xf8,  //   005a : 		          JR   NZ,LOOPH    
                                  0xc9,  //   005c : 			          RET       
                           0x3e,  0xcf,  //   005d : 			LD A, 0CFH 
                           0xed,  0x79,  //   005f : 			OUT (C), A 
                                  0x7c,  //   0061 : 				LD A, H 
                           0xed,  0x79,  //   0062 : 			OUT (C), A 
                                  0x7d,  //   0064 : 				LD A, L 
                                  0x4a,  //   0065 : 				LD C, D 
                           0xed,  0x79,  //   0066 : 			OUT (C), A 
                                  0xc9,  //   0068 : 				RET 
                                  0xc5,  //   0069 : 			RS_LOW:PUSh   BC 
                           0x0e,  0x81,  //   006a : 			LD    C, IO_ADDR_PIO1_BD 
                    0x2a,  0x01,  0xa0,  //   006c : 			LD    HL,(SHADOW_B) 
                                  0x7e,  //   006f : 				LD    A, (HL) 
                           0xcb,  0x87,  //   0070 : 			RES   0, A 
                                  0x77,  //   0072 : 				LD    (HL), A 
                           0xed,  0x79,  //   0073 : 			OUT   (C), A 
                                  0xc1,  //   0075 : 				POp   BC 
                                  0xc9,  //   0076 : 				RET 
                                  0xc5,  //   0077 : 			RS_HIGH:PUSH  BC 
                           0x0e,  0x81,  //   0078 : 		        LD    C, IO_ADDR_PIO1_BD 
                    0x2a,  0x01,  0xa0,  //   007a : 			LD    HL,(SHADOW_B) 
                                  0x7e,  //   007d : 				LD    A, (HL) 
                           0xcb,  0xc7,  //   007e : 			SET   0, A 
                                  0x77,  //   0080 : 				LD    (HL), A 
                           0xed,  0x79,  //   0081 : 			OUT   (C), A 
                                  0xc1,  //   0083 : 				POP   BC 
                                  0xc9,  //   0084 : 				RET 
                                  0xc5,  //   0085 : 			E_LOW:  PUSH  BC 
                           0x0e,  0x81,  //   0086 : 			LD    C, IO_ADDR_PIO1_BD 
                    0x2a,  0x01,  0xa0,  //   0088 : 			LD    HL,(SHADOW_B) 
                                  0x7e,  //   008b : 				LD    A, (HL) 
                           0xcb,  0x8f,  //   008c : 			RES   1, A 
                                  0x77,  //   008e : 				LD    (HL), A 
                           0xed,  0x79,  //   008f : 			OUT   (C), A 
                                  0xc1,  //   0091 : 				POP   BC 
                                  0xc9,  //   0092 : 				RET 
                                  0xc5,  //   0093 : 			E_HIGH: PUSH BC 
                           0x0e,  0x81,  //   0094 : 			LD    C, IO_ADDR_PIO1_BD 
                    0x2a,  0x01,  0xa0,  //   0096 : 			LD    HL,(SHADOW_B) 
                                  0x7e,  //   0099 : 				LD    A, (HL) 
                           0xcb,  0xcf,  //   009a : 			SET   1, A 
                                  0x77,  //   009c : 				LD    (HL), A 
                           0xed,  0x79,  //   009d : 			OUT   (C), A 
                                  0xc1,  //   009f : 				POP   BC 
                                  0xc9,  //   00a0 : 				RET 
                    0xcd,  0x69,  0x00,  //   00a1 : 			CALL  RS_LOW 
                    0xcd,  0x93,  0x00,  //   00a4 : 			CALL  E_HIGH 
                                  0x7a,  //   00a7 : 				LD    A, D 
                           0xed,  0x79,  //   00a8 : 			OUT   (C), A 
                    0xcd,  0x85,  0x00,  //   00aa : 			CALL  E_LOW 
                    0xcd,  0x93,  0x00,  //   00ad : 			CALL  E_HIGH 
                                  0x7a,  //   00b0 : 				LD    A, D 
                           0xcb,  0x27,  //   00b1 : 			SLA   A 
                           0xcb,  0x27,  //   00b3 : 			SLA   A 
                           0xcb,  0x27,  //   00b5 : 			SLA   A 
                           0xcb,  0x27,  //   00b7 : 			SLA   A 
                           0xed,  0x79,  //   00b9 : 			OUT   (C), A 
                    0xcd,  0x85,  0x00,  //   00bb : 			CALL  E_LOW 
                    0xcd,  0x93,  0x00,  //   00be : 			CALL  E_HIGH 
                    0xcd,  0x85,  0x00,  //   00c1 : 			CALL  E_LOW 
                    0xcd,  0x93,  0x00,  //   00c4 : 			CALL  E_HIGH 
                    0xcd,  0x85,  0x00,  //   00c7 : 			CALL  E_LOW 
                    0xcd,  0x93,  0x00,  //   00ca : 			CALL  E_HIGH 
                    0xcd,  0x77,  0x00,  //   00cd : 			CALL  RS_HIGH 
                                  0xc9,  //   00d0 : 				RET 
                    0xcd,  0x69,  0x00,  //   00d1 : 		SDATA4: CALL  RS_LOW 
                    0xcd,  0x93,  0x00,  //   00d4 : 			CALL  E_HIGH 
                                  0x7a,  //   00d7 : 				LD    A, D 
                           0xed,  0x79,  //   00d8 : 			OUT   (C), A 
                    0xcd,  0x85,  0x00,  //   00da : 			CALL  E_LOW 
                    0xcd,  0x93,  0x00,  //   00dd : 			CALL  E_HIGH 
                    0xcd,  0x85,  0x00,  //   00e0 : 			CALL  E_LOW 
                    0xcd,  0x93,  0x00,  //   00e3 : 			CALL  E_HIGH 
                    0xcd,  0x85,  0x00,  //   00e6 : 			CALL  E_LOW 
                    0xcd,  0x93,  0x00,  //   00e9 : 			CALL  E_HIGH 
                    0xcd,  0x85,  0x00,  //   00ec : 			CALL  E_LOW 
                    0xcd,  0x93,  0x00,  //   00ef : 			CALL  E_HIGH 
                    0xcd,  0x77,  0x00,  //   00f2 : 			CALL  RS_HIGH 
                                  0xc9,  //   00f5 : 				RET
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
	  Serial.println(" (G:Grab Bus  R: release bus M:Mega control  F:Free run T:Drive n tstates) b:Breakpoint B:Toggle breakpoint)");
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
		    case 'T':
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
		  // Take the example code and write it to flash
		  for(int i=0; i<example_code_length; i++)
		    {
		      flash_write_byte(0, i, example_code[i]);
		    }
		  break;

		case 'Y':
		  // Take the example code and write it to all banks of flash 
		  for(int b=0; b<16;b++)
		    {

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


































