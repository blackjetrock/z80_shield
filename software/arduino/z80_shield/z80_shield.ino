//
// Arduino Mega sketch for the Z80 shield
// Serial monitor based control program
//

//#include <stdio.h>
//#include <string.h>
//#include <ctype.h>

// If you enable this, the Mega will supply data from its own internal buffer when the Z80
// reads bytes from the ROM (i.e. Flash). Turn it off for the flash to supply the bytes, in
// which case you only need to remember to write your code/data to flash once. :)
//
#define ENABLE_MEGA_ROM_EMULATION   0
#define ENABLE_DIRECT_PORT_ACCESS   1
#define ENABLE_TIMINGS              0
#define ENABLE_LOCAL_ECHO           0

#define TRACE_SIZE                 40
#define II_TRACE_SIZE              80

typedef unsigned char BYTE;
typedef void (*FPTR)();
typedef void (*CMD_FPTR)(String cmd);

void cmd_reset_z80(String cmd);

void run_bsm(int stim);
String bsm_state_name();
unsigned int addr_state();
unsigned int data_state();

// Run in quiet mode
boolean quiet = false;
boolean fast_mode = false;       // Skip all output and interaction

// Some command must not be interrupted, some can be, some have to be (like 'f')
// The command must handle the setting and clearing of this flag
boolean uninterruptible_command = false;

// Set if the opcode of this bus cycle is a prefix type (does not work in injected code sequences)
boolean prefix_opcode = false;

const BYTE prefix_opcodes[] = {0xdd, 0xfd, 0xed, 0xcb };

#define NUM_PREFIX_OPCODES (sizeof(prefix_opcodes))
  
// Stop if M1 asserted (used for running to next instruction
boolean stop_on_m1 = false;

// Stop on full instruction
// Stops once a full instruction, i.e. not just a prefix type opcode has executed
boolean stop_on_full_instruction = false;

// Flag is set when opcode is traced, and cleared when M1 is asserted
// This is used to make the 'next instruction' commad work more intuitively
boolean opcode_traced = false;

// Array that tells us how long an instrcution is, for every opcode
// Used in inter instruction code execution
// Not array of struct so it stays in flash

const BYTE instruction_length[] =
  {
    0x08, 1,    // EX AF,AF'
    0x18, 2,    // JR x

    0x21, 3,
    0x22, 3,
    0x23, 1,
    
    0x3e, 2,
    0x31, 3,

    0x43, 3,
    
    0x77, 1,
    0x7e, 1,


    0xc1, 1,
    0xc3, 3,  // call
    0xc5, 1,
    
    0xd1, 1,
    0xd5, 1,
    0xd9, 1,   // exx
    0xdd, 1,   // prefix hl-> ix

    0xe1, 1,    // pop  ix/iy/hl
    0xe5, 1,    // push ix/iy/hl

    0xed, 1,

    0xf1, 1,
    0xf5, 1,
    0xfd, 1,   // prefix hl-> iy

    0x00, 1     // Both NOP and end of table
  };



// Inserting code between instructions
boolean inter_inst = false;
boolean inter_inst_trace = false;

BYTE *inter_inst_code = NULL;
unsigned int inter_inst_index = 0;
unsigned int inter_inst_code_length = 0;
unsigned int inter_inst_em_count = 0;
boolean ii_process_registers = false;

// Run to this address and stop in fast mode
int fast_to_address = -1;

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
    EV_A_NULL,   EV_D_NULL,       // just used for initialisation
  };

// Tracing
typedef enum
  {
    TRT_INVALID,
    TRT_MEM_RD,
    TRT_MEM_WR,
    TRT_IO_RD,
    TRT_IO_WR,
    TRT_OP_RD,    // Opcode fetch
  } TRACE_REC_TYPE;

typedef struct
{
  TRACE_REC_TYPE type;
  unsigned int data;
  unsigned int addr;
} TRACE_REC;

// There's a performance degradation if tracing is on so allow it to be turned off
int trace_on = 1;

int trace_index = 0;
int ii_trace_index = 0;

// Instruction bus trace
TRACE_REC trace[TRACE_SIZE];

// Inter Instruction bus trace
TRACE_REC ii_trace[II_TRACE_SIZE];

char trace_rec_buf[40];

char *textify_trace_rec(int i, TRACE_REC *trace)
{
  char type[20];
  
  switch(trace[i].type)
    {
    case TRT_MEM_RD:
      strcpy(type, "MEM RD");
      break;

    case TRT_MEM_WR:
      strcpy(type, "MEM WR");
      break;

    case TRT_IO_RD:
      strcpy(type, "IO  RD");
      break;

    case TRT_OP_RD:
      strcpy(type, "OP  RD");
      break;

    case TRT_IO_WR:
      strcpy(type, "IO  WR");
      break;
	     
    default:
      strcpy(type, "????");
      break;
    }
  
  sprintf(trace_rec_buf, "%02d: %s %04X %02X", i, type, trace[i].addr, trace[i].data);
  return(trace_rec_buf);
}

// Trace into he appropriate buffer

void trace_rec(TRACE_REC_TYPE trt)
{
  if ( trace_on )
    {
      if( inter_inst_trace )
	{
	  ii_trace[ii_trace_index].type = trt;
	  ii_trace[ii_trace_index].data = data_state();
	  ii_trace[ii_trace_index].addr = addr_state();
	  ii_trace_index++;
	  ii_trace_index = ii_trace_index % II_TRACE_SIZE;
	}
      else
	{
	  trace[trace_index].type = trt;
	  trace[trace_index].data = data_state();
	  trace[trace_index].addr = addr_state();
	  trace_index++;
	  trace_index = trace_index % TRACE_SIZE;
	}

      // Turn off ii tracing if ii is off. This is for an edge condition as we turn ii off (we want to trace the last
      // bus transfer, but we have turned ii off before we trace)
      if ( !inter_inst )
	{
	  inter_inst_trace = false;
	}
      
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// IO addresses
//

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
  const String signame;
  const String description;
  const String assertion_note;
  const int pin;
  int   current_state;
  const struct
  {
    int     mode;        // Mode
    uint8_t mode_dir;    // The direction we set this line when in this mode
    uint8_t mode_val;    // Default value for this mode
  } modes[2];
  const int     assert_ev;   // Assert event
  const int     deassert_ev; // Deassert event
}
  
// The order in this list defines the order in which events will be raised, which affects how th e
// bus state machine will be laid out.
//
  signal_list[] =
    {
      {  "BUSREQ", "Mega --> Z80",    "     - Asserted, means Mega is controlling the Z80's buses",
	 BUSREQ_Pin, 0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, LOW }}, EV_A_BUSREQ, EV_D_BUSREQ},
      {  "BUSACK", "Z80  --> Mega",   "    - Asserted, means Z80 acknowledges it's not in control of its buses",
	 BUSACK_Pin, 0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, INPUT,  HIGH}},  EV_A_BUSACK, EV_D_BUSACK},
      {  "    M1", "Z80  --> Mega",   "    - Asserted, means Z80 is doing an opcode fetch cycle",
	 M1_Pin,     0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_M1, EV_D_M1},
      {  "  MREQ", "Z80  --> Mega",   "    - Asserted, means address bus holds a memory address for a read or write",
	 MREQ_Pin,   0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_MREQ, EV_D_MREQ},
      {  " IOREQ", "Z80  --> Mega",   "    - Asserted, means lower half of address bus holds an IO address for a read or write",
	 IOREQ_Pin,  0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_IOREQ, EV_D_IOREQ},
      {  "  RFSH", "Z80  --> Mega",   "    - Asserted, means Z80 is in refresh state",
	 RFSH_Pin,   0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_RFSH, EV_D_RFSH},
      {  "    WR", "Z80  --> Mega",   "    - Asserted, means the data bus holds a value to be written",
	 WR_Pin,     0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_WR, EV_D_WR},
      {  "    RD", "Z80  --> Mega",   "    - Asserted, means the Z80 wants to read data from external device",
	 RD_Pin,     0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_RD, EV_D_RD},
      {  "   NMI", "Mega --> Z80",    "     - Asserted, means a non maskable interrupt is being sent to the Z80",
	 NMI_Pin,    0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_NMI, EV_D_NMI},
      {  "   INT", "Mega --> Z80",    "     - Asserted, means a maskable interrupt is being sent to the Z80",
	 INT_Pin,    0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_INT, EV_D_INT},
      {  "  WAIT", "Mega --> Z80",    "",
	 WAIT_Pin,   0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_WAIT, EV_D_WAIT},
      {  "   CLK", "Mega --> Z80",    "     - Asserted, means Z80 is in the second half of a T-state",
	 A_CLK_Pin,  0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_CLK, EV_D_CLK},
      {  "   RES", "Mega --> Z80",    "     - Asserted, means the Z80 is being held in reset state",
	 A_RES_Pin,  0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}}, EV_A_RES, EV_D_RES},
      {  "MAPRQM", "Mega --> Shield", "  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)",
	 MAPRQM_Pin, 0, {{MODE_SLAVE, OUTPUT, HIGH},{MODE_MEGA_MASTER, OUTPUT, LOW }}, EV_A_MAPRQM, EV_D_MAPRQM},
      {  "MAPRQI", "Mega --> Shield", "  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)",
	 MAPRQI_Pin, 0, {{MODE_SLAVE, OUTPUT, LOW },{MODE_MEGA_MASTER, OUTPUT, LOW }},  EV_A_MAPRQI, EV_D_MAPRQI},
      {  "    X1", "Z80  --> Mega",   "    - Asserted, means Z80 is doing an opcode fetch cycle",
	 M1_Pin,     0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  EV_A_M1, EV_D_M1},
      {  "---",    "",                "",
	 0,          0, {{MODE_SLAVE, INPUT,  HIGH},{MODE_MEGA_MASTER, OUTPUT, HIGH}},  0, 0},
    };

// Indices for signals
enum
  {
    SIG_BUSREQ,
    SIG_BUSACK,
    SIG_M1,
    SIG_MREQ,
    SIG_IOREQ,
    SIG_RFSH,
    SIG_WR,
    SIG_RD,
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
  const int stim;
  const int next_state;
};

#define NUM_ENTRY 2
#define NUM_TRANS 4

struct STATE
{
  const int        statenum;
  const String     state_name;
  const FPTR       entry[NUM_ENTRY];
  const TRANSITION trans[NUM_TRANS];
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
    STATE_MEM1,
    STATE_MEM_RD,
    STATE_MEM_WR,
    STATE_MEM_RD_END,
    STATE_MEM_WR_END,
    STATE_IO1,
    STATE_IO_RD,
    STATE_IO_WR,
    STATE_IO_RD_END,
    STATE_IO_WR_END,

    STATE_NULL,             // Used for initialisation
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

// Z80 registers state. We don't have access to most of them as yet,
// but the plan is to make them available
//
struct Z80_REGISTERS
{
  uint16_t pc;
  uint16_t sp;
  uint16_t af;
  uint16_t hl;
  uint16_t de;
  uint16_t bc;
  uint16_t ix;
  uint16_t iy;
  uint16_t afp;   // AF' or AF prime -> afp
  uint16_t hlp;
  uint16_t dep;
  uint16_t bcp;


  // Define the rest when we can do somthing with them
};

Z80_REGISTERS z80_registers;

////////////////////////////////////////////////////////////////////////////////
//
// Instruction trace
//
// Traces address and data values while code executes
// Can be used for general debug and also used by the register dump facility
//
//


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

// If we are stopping when M1 asserted then stop
void entry_opcode1()
{

  // Only stop on every M1 if we aren't stopping on full instructions
  // Full stopping on full instructions, we disallow stopping on M1 if the opcode is
  // a prefix type
   
  if ( stop_on_m1 && opcode_traced && (stop_on_full_instruction && (!prefix_opcode)) )
    {
      if ( (fast_to_address == -1) || ((fast_to_address != -1) && (fast_to_address == addr_state())) )
	{
	  fast_mode = false;
	  quiet = false;

	  // Turn off stopping on M1, it's always a one-shot
	  stop_on_m1 = false;
	}
    }

  // New opcode, so clear the opcode traced flag as this opcode hasn't been
  opcode_traced = false;

}

void entry_trc_op()
{
  // Trace opcode
  trace_rec(TRT_OP_RD);

  opcode_traced = true;
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
	{EV_A_MREQ, STATE_MEM1},
	{EV_A_IOREQ, STATE_IO1},
      }
    },
    { 
      STATE_OP1,
      "Opcode 1",
      {
	entry_opcode1,
      },
      {
	{EV_A_MREQ, STATE_OP2},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
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
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
    { 
      STATE_OP3,
      "Opcode Read",
      {
	// same as a memory access
	entry_op_rd,
	entry_trc_op,
      },
      {
	{EV_A_RFSH, STATE_RFSH1},
	{EV_D_RD, STATE_OP4},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
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
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
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
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
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
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
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
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
    { 
      STATE_MEM_RD,
      "Memory Read Access",
      {
	entry_mem_rd,
	entry_trc_mem_rd,
      },
      {
	{EV_D_MREQ, STATE_MEM_RD_END},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
    { 
      STATE_MEM_WR,
      "Memory Write Access",
      {
	entry_trc_mem_wr,
      },
      {
	{EV_D_MREQ, STATE_MEM_WR_END},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
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
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
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
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
#if 1
    //------------------------------------------------------------------------------
    // IO accesses
    //
    // 
    //
    { 
      STATE_IO1,
      "IO Access",
      {
	entry_null,
      },
      {
	{EV_A_RD, STATE_IO_RD},
	{EV_A_WR, STATE_IO_WR},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
    { 
      STATE_IO_RD,
      "IO Read Access",
      {
	entry_trc_io_rd,
      },
      {
	{EV_D_IOREQ, STATE_IO_RD_END},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
    { 
      STATE_IO_WR,
      "IO Write Access",
      {
	entry_trc_io_wr,
      },
      {
	{EV_D_IOREQ, STATE_IO_WR_END},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
    { 
      STATE_IO_RD_END,
      "IO Read Access END",
      {
	entry_null,
      },
      {
	{EV_D_RD, STATE_IDLE},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
    { 
      STATE_IO_WR_END,
      "IO Write Access End",
      {
	entry_null,
      },
      {
	{EV_D_WR, STATE_IDLE},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
	{EV_A_NULL, STATE_NULL},
      }
    },
#endif    
  };

void entry_null()
{
}

void entry_mem1()
{
  
  // If the address is in the range of the RAM chip then we enable it.
  // This is because the Mega doesn't really have enough memory to emulate the RAM, so we use the real chip
  if ( addr_state() >= 0x8000 )
    {
      // It is a RAM access
      // Enable the memory map for a while

      // We don't drive the bus
      data_bus_inputs();
      
      // Turn memory map back on (i.e. hardware supplies RAM data)
      digitalWrite(MAPRQM_Pin, LOW);

      if ( !fast_mode )
	{
	  Serial.print(F("Allowing RAM to put data on bus"));
	  Serial.print(addr_state(), HEX);
	  Serial.print(": ");
	  Serial.print(data_state(), HEX);
	}
    }

  // If it's a flash access then we will drive data if WR is asserted
}

////////////////////////////////////////////////////////////////////////////////

// Examine the II trace and find register values
// This code is specific to the register dump II code, change one and the other needs
// examination

void get_register_values_from_trace()
{
  int i;
  unsigned int twobyteval;
  int shift_opcode = 0x00;       // Holds last shift opcode
  boolean exx_flag = false;      // Holds flag that says which register set we are using in our code
  boolean grab_pc = true;        // Only use first PUSH AF to grab PC and SP
  
  for(i=0; i<II_TRACE_SIZE; i++)
    {
      // Speculatively calculate values
      twobyteval = ii_trace[(i+2) % II_TRACE_SIZE].data + ((ii_trace[(i+1) % II_TRACE_SIZE].data) <<8);
      switch(ii_trace[i].type)
	{
	case TRT_OP_RD:
	  switch(ii_trace[i].data)
	    {
	      // exx instruction
	      // For register display (and code) the current register set is always the non prime one (no ')
	      // The prime set (AF' etc) are always the ones you get after an exx instruction.
	      // This makes sense as if you display the registers inside an ISR that has done an exx at the start then the
	      // non prime register set is th eone you are working with and the saved context is the prime set
	      // So, we don't need to know if the code that is running has done a previous exx
	      // We do need to know when the code in the ii trace has done an exx though, as we need to
	      // dipslay the values in the correct place
	    case 0xD9:
	      exx_flag = !exx_flag;
	      break;
	      
	      // This is a shift opcode that indicates use IX instead of HL
	    case 0xDD:
	    case 0xFD:
	      shift_opcode = ii_trace[i].data;
	      break;
	      
	      // This is the first instruction in the ii code, we can get the PC and SP from this instruction
	    case 0xF5:     // PUSH AF
	      // Send AF value
	      if ( exx_flag )
		{
		  Serial.print(F("^0AF':"));
		  z80_registers.afp = twobyteval;
		  Serial.print( to_hex(z80_registers.afp, 4) );
		  Serial.print(F("$"));
		}
	      else
		{
		  Serial.print(F("^0AF :"));
		  z80_registers.af = twobyteval;
		  Serial.print( to_hex(z80_registers.af, 4) );
		  Serial.print(F("$"));
		}

	      // Only get PC and SP for first PUSH AF
	      if ( grab_pc )
		{
		  // get PC, it's the instruction address
		  Serial.print(F("^0PC :"));
		  z80_registers.pc = ii_trace[i].addr;;
		  Serial.print( to_hex(z80_registers.pc, 4) );
		  Serial.print(F("$"));
		  
		  // get SP, it's the address the first byte was written to plus 1
		  Serial.print(F("^0SP :"));
		  z80_registers.sp = ii_trace[(i+1) % II_TRACE_SIZE].addr+1;
		  Serial.print( to_hex(z80_registers.sp, 4) );
		  Serial.print(F("$"));
		  grab_pc = false;
		}
	      break;
	      
	    case 0xE1:
	      // we don't process the pop ix.iy, but need to clear the flags
	      shift_opcode = 0x00;
	      break;
	      
	    case 0xE5:     // PUSH HL
	      switch(shift_opcode)
		{
		case 0x00:    // No previous shift opcode		  
		  // Send HL value
		  if ( exx_flag )
		    {
		      Serial.print(F("^0HL':"));
		      z80_registers.hlp = twobyteval;
		      Serial.print( to_hex(z80_registers.hlp, 4) );
		      Serial.print(F("$"));
		    }
		  else
		    {
		      Serial.print(F("^0HL :"));
		      z80_registers.hl = twobyteval;
		      Serial.print( to_hex(z80_registers.hl, 4) );
		      Serial.print(F("$"));
		    }
		  break;
		  
		case 0xDD:
		  // Send IX value
		  Serial.print(F("^0IX :"));
		  z80_registers.ix = twobyteval;
		  Serial.print( to_hex(z80_registers.ix, 4) );
		  Serial.print(F("$"));
		  shift_opcode = 0x00; 
		  break;
		  
		case 0xFD:
		  // Send IY value
		  Serial.print(F("^0IY :"));
		  z80_registers.iy = twobyteval;
		  Serial.print( to_hex(z80_registers.iy, 4) );
		  Serial.print(F("$"));
		  shift_opcode = 0x00; 
		  break;
		}
	      break;

	    case 0xC5:     // PUSH BC
	      // Send BC value
	      if (exx_flag )
		{
		  Serial.print(F("^0BC':"));
		  z80_registers.bcp = twobyteval;
		  Serial.print( to_hex(z80_registers.bcp, 4) );
		  Serial.print(F("$"));
		}
	      else
		{
		  Serial.print(F("^0BC :"));
		  z80_registers.bc = twobyteval;
		  Serial.print( to_hex(z80_registers.bc, 4) );
		  Serial.print(F("$"));
		}
	      break;

	    case 0xD5:     // PUSH DE
	      // Send DE value
	      if (exx_flag )
		{
		  Serial.print(F("^0DE':"));
		  z80_registers.dep = twobyteval;
		  Serial.print( to_hex(z80_registers.dep, 4) );
		  Serial.print(F("$"));
		}
	      else
		{
		  Serial.print(F("^0DE :"));
		  z80_registers.de = twobyteval;
		  Serial.print( to_hex(z80_registers.de, 4) );
		  Serial.print(F("$"));
		}
	      break;
	    }
	  
	  break;
	}
    }
}

unsigned int get_instruction_length(BYTE opcode)
{
  int i;
  
  if( opcode ==0 )
    {
      return(1);
    }
  
  for(i=0; instruction_length[i*2] != 0; i++)
    {
      if( instruction_length[i*2] == opcode )
	{
	  return(instruction_length[i*2+1]);
	}
    }
  
  // Default to one byte
  return(1);
}

// Inserts instructions between instructions

void entry_core_ii_emulate()
{
  //Serial.print("II:");
  //Serial.println(inter_inst_em_count);

  inter_inst_em_count--;
  
  // We get data from our emulated code whatever the address
  // Emulate flash
  // Disable hardware so we can drive the bus
  digitalWrite(MAPRQM_Pin, HIGH);

  data_bus_outputs();
      
  set_data_state(inter_inst_code[inter_inst_index]);
  
  
  //Serial.print(F("Putting ii data on bus "));
  //Serial.print(addr_state(), HEX);
  //Serial.print(" ");
  //Serial.print(data_state(), HEX);
  //Serial.print(inter_inst_code[inter_inst_index], HEX);

  inter_inst_index++;

  if ( inter_inst_index >= inter_inst_code_length )
    {
      // End of inter inst code, turn everything off
      // We do, however want to trace this transfer in the ii_trace buffer so leave the ii_trace flag set until the next iteration
      fast_mode = false;
      inter_inst = false;
      uninterruptible_command = false;
      
      quiet = false;

      // We have run the code between instructions, we may have to process the result
      if( ii_process_registers )
	{
	  // Process trace to get register values
	  get_register_values_from_trace();
	}
      return;
    }
  
}

// Memory read cycle
void entry_mem_rd()
{
  // Instruction data bytes  come from flash (or perhaps emulated by mega unless we are inserting code between instructions
  // then we get code from the inter_inst array until it runs out
   // have we stopped emulating the instruction?
  if( inter_inst_em_count == 0 )
    {
      // Enable hardware so it can drive the bus
      digitalWrite(MAPRQM_Pin, LOW);
      
      data_bus_inputs();
    }
  else
    {
      if ( inter_inst && (inter_inst_code != NULL) && (inter_inst_em_count > 0) )
	{
	  entry_core_ii_emulate();
	  
	}
    }

#if ENABLE_MEGA_ROM_EMULATION
  // We get data from our emulated flash.
  if ( addr_state() < 0x8000 )
    {
      // Emulate flash
      // Disable hardware so we can drive the bus
      digitalWrite(MAPRQM_Pin, HIGH);
      data_bus_outputs();
      
      set_data_state(example_code[addr_state()]);

      Serial.print(F("Putting ii data on bus "));
      Serial.print(addr_state(), HEX);
      Serial.print(" ");
      Serial.print(example_code[addr_state()], HEX);

    }
#endif
}

// Memory read cycle
void entry_op_rd()
{
  int i;
  
  // Opcodes come rom flash (or perhaps emulated by mega unless we are inserting code between instructions
  // then we get code from the inter_inst array until it runs out
  if ( inter_inst && (inter_inst_code != NULL) )
    {
      // Find out how many opcode bytes to emulate after this one
      inter_inst_em_count = get_instruction_length(inter_inst_code[inter_inst_index]);
      entry_core_ii_emulate();
      
    }
  else
    {
      prefix_opcode = false;
      
      // For non-injected instructions we set a flag if this is a prefix opcode
      for(i=0; i<NUM_PREFIX_OPCODES;i++)
	{
	  if ( data_state() == prefix_opcodes[i] )
	    {
	      prefix_opcode = true;
	      break;
	    }
	}
    }
  
}


void entry_trc_mem_rd()
{
  // Trace
  
  trace_rec(TRT_MEM_RD);

}

void entry_trc_mem_wr()
{
  // Trace
  trace_rec(TRT_MEM_WR);
}

void entry_trc_io_rd()
{
  // Trace
  trace_rec(TRT_IO_RD);
}

void entry_trc_io_wr()
{
  // Trace
  trace_rec(TRT_IO_WR);
}

void entry_mem_rd_end()
{
  // release the data bus, either if we have driven it or the RAM/flash chip has, it makes no difference

  // Enable the memory hardware again
  digitalWrite(MAPRQM_Pin, LOW);

  data_bus_inputs();
}

String bsm_state_name()
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

  // The Z80 user manual says we need 3 full clock for the reset to complete
  for( int i=0; i<3; i++ )
    {
      t_state();
    }

  // Release reset
  deassert_signal(SIG_RES);

  z80_registers.pc = 0;
  
}

// Do a half t state
int clk = 0;

#if ENABLE_TIMINGS
#define NUM_TIMED_CLKS 20

// Variables where we store clock rate timing information
unsigned long last_clk = 0;
unsigned long now_clk = 0;
unsigned long last_clks[NUM_TIMED_CLKS];
int last_clks_i = 0;
#endif

void half_t_state()
{

  if ( clk )
    {
      //      Serial.println("Running half a clock (CLK HIGH)...\n");
      digitalWrite(A_CLK_Pin, HIGH);
    }
  else
    {
      //      Serial.println("Running half a clock (CLK LOW)...\n");
      digitalWrite(A_CLK_Pin, LOW);
    }
  
  clk = !clk;

#if ENABLE_TIMINGS
  now_clk = millis();
  last_clks[last_clks_i++] = now_clk-last_clk;
  last_clk = now_clk;
  last_clks_i = last_clks_i % NUM_TIMED_CLKS;
#endif
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
      Serial.println(F("Clocking.."));
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
  (void)sector;

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
#if ENABLE_DIRECT_PORT_ACCESS
  return(PINA + (PINB << 8));
#else
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
#endif
}

// drive address bus
void set_addr_state(int address)
{
#if ENABLE_DIRECT_PORT_ACCESS
  PORTA = address & 0xff;
  PORTB = (address & 0xff00) >> 8;
#else
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
#endif
}

// Inverts bits in an 8 bit value
unsigned int invert_byte(unsigned int x)
{
#define BIT(X, BITNUM, NEWBITNUM)  (((X & (1<<BITNUM)) >> BITNUM) << NEWBITNUM)

  return( BIT(x,0,7)+
	  BIT(x,1,6)+
	  BIT(x,2,5)+
	  BIT(x,3,4)+
	  BIT(x,4,3)+
	  BIT(x,5,2)+
	  BIT(x,6,1)+
	  BIT(x,7,0));
}

// Returns data bus state, ie data on bus

unsigned int data_state()
{
#if ENABLE_DIRECT_PORT_ACCESS
  return ( invert_byte(PINC) );
#else
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
#endif
}

// Sets data bus value
void set_data_state(unsigned int x)
{
#if ENABLE_DIRECT_PORT_ACCESS
  PORTC = invert_byte(x);
#else
  
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
#endif
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
	  //signal_list[i].current_state = state;
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
	  
	  for(int i=0; signal_list[i].signame != "---" ;i++)
	    {
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
      for(int i=0; signal_list[i].signame != "---"; i++)
	{
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
	  Serial.print("      (");
	  Serial.print(signal_list[i].description+")");

	  if( val == LOW )
	    {
	      Serial.print(signal_list[i].assertion_note);
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
  for(int i=0; signal_list[i].signame != "---"; i++)
    {
      signal_list[i].current_state  = digitalRead(signal_list[i].pin);
    }
}


////////////////////////////////////////////////////////////////////////////////
//
// generate any bus events that may have occurred
//


void signal_scan()
{
  for(int i=0; signal_list[i].signame != "---"; i++)
    {
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
	  Serial.println(F(" ASSERT"));
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
  Serial.println(F("***ERROR state not found ***"));
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
  (void)cmd;

  initialise_z80_for_control();

  Serial.println(F("\n-------------------------------------------------------------------"));
  Serial.println(F("The Arduino has grabbed the Z80, the Z80 is now the Arduino's slave"));
  Serial.println(F("-------------------------------------------------------------------"));
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
    
  if( signal_state("M1") == HIGH )
    Serial.print(F("Addr:"));  
  else
    Serial.print(F("PC:"));  

  Serial.print(to_hex(address, 4));

  Serial.print(F("  Data:"));
  Serial.print(to_hex(data, 2));
  Serial.println("\n");
  
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
const BYTE example_code_ram_chk[] =
  {
    0x31, 0x00, 0x90,    // set stack up
    0x3e, 0xaa,          // LOOP:   LD A, 03EH
    0x21, 0x34, 0x82,    //         LD HL 01234H
    0x77,                //         LD (HL), A
    0x7e,                //         LD   A,(HL)
    0x23,                //         INC HL
    0xc3, 0x5, 0x0     //         JR LOOP
  };

// Turns backlight off
const BYTE example_code_lcd_bl_off[] =
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
const BYTE example_code_lcd_bl_flash[] =
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

const BYTE example_code_lcd_slow_flash[] =
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

const BYTE example_code_ram[] =
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

const BYTE example_code_bank[] =
  {
    0x0e, 0xc0,          // LOOP:   LD C, 60H
    0x3e, 0xaa,          //         LD  A, AAH
    0xed, 0x79,         //          OUT (C), A
    0xc3, 0x05, 0x00
  };

const BYTE example_code_lcd_test[] =
  {
    0x31,
    0x00,
    0x90,
    0x0E,
    0x82,
    0x16,
    0x80,
    0x26,
    0x0F,
    0x2E,
    0x00,
    0xCD,
    0x80,
    0x01,
    0x0E,
    0x83,
    0x16,
    0x81,
    0x26,
    0xFC,
    0x2E,
    0x00,
    0xCD,
    0x80,
    0x01,
    0x0E,
    0x02,
    0x16,
    0x00,
    0x26,
    0xF8,
    0x2E,
    0x01,
    0xCD,
    0x80,
    0x01,
    0x3E,
    0x01,
    0x32,
    0x02,
    0xA0,
    0x3E,
    0x00,
    0x32,
    0x00,
    0xA0,
    0x3E,
    0x00,
    0x32,
    0x01,
    0xA0,
    0xCD,
    0x8C,
    0x01,
    0xCD,
    0xAC,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x03,
    0xCD,
    0x16,
    0x02,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x03,
    0xCD,
    0x16,
    0x02,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x03,
    0xCD,
    0x16,
    0x02,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x02,
    0xCD,
    0x16,
    0x02,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x0E,
    0xCD,
    0xCC,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x06,
    0xCD,
    0xCC,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x01,
    0xCD,
    0xCC,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x5A,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x38,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x30,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x20,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x53,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x68,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x69,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x65,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x6C,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x64,
    0xCD,
    0xF1,
    0x01,
    0xCD,
    0x75,
    0x01,
    0x16,
    0x02,
    0xCD,
    0xCC,
    0x01,
    0xCD,
    0xE9,
    0x00,
    0xED,
    0x43,
    0x00,
    0xB0,
    0x3E,
    0x41,
    0x81,
    0x57,
    0xCD,
    0xF1,
    0x01,
    0x18,
    0xEB,
    0x18,
    0xFE,
    0x7E,
    0xFE,
    0x00,
    0x20,
    0x01,
    0xC9,
    0x7E,
    0x57,
    0xCD,
    0xF1,
    0x01,
    0x23,
    0x18,
    0xF2,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x30,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x80,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x80,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x70,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x70,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x70,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x70,
    0x02,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x50,
    0x02,
    0xCD,
    0x70,
    0x02,
    0xCD,
    0x60,
    0x02,
    0x16,
    0x0A,
    0x01,
    0x00,
    0x00,
    0xCD,
    0x60,
    0x02,
    0xCD,
    0x90,
    0x02,
    0xCB,
    0x21,
    0xCB,
    0x10,
    0xE6,
    0x01,
    0xB1,
    0x4F,
    0xCD,
    0x50,
    0x02,
    0x15,
    0x20,
    0xEC,
    0xCD,
    0x40,
    0x02,
    0xC9,
    0x26,
    0x02,
    0x2E,
    0xFF,
    0x2D,
    0x20,
    0xFD,
    0x25,
    0x20,
    0xF8,
    0xC9,
    0x3E,
    0xCF,
    0xED,
    0x79,
    0x7C,
    0xED,
    0x79,
    0x7D,
    0x4A,
    0xED,
    0x79,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x81,
    0x21,
    0x01,
    0xA0,
    0x7E,
    0xCB,
    0x87,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x81,
    0x21,
    0x01,
    0xA0,
    0x7E,
    0xCB,
    0xC7,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x81,
    0x21,
    0x01,
    0xA0,
    0x7E,
    0xCB,
    0x8F,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x81,
    0x21,
    0x01,
    0xA0,
    0x7E,
    0xCB,
    0xCF,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0x0E,
    0x80,
    0xCD,
    0x8C,
    0x01,
    0xCD,
    0xBC,
    0x01,
    0x7A,
    0x0E,
    0x80,
    0xED,
    0x79,
    0xCD,
    0xAC,
    0x01,
    0xCD,
    0xBC,
    0x01,
    0x7A,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0xED,
    0x79,
    0xCD,
    0xAC,
    0x01,
    0xCD,
    0x9C,
    0x01,
    0xC9,
    0x0E,
    0x80,
    0xCD,
    0x9C,
    0x01,
    0xCD,
    0xBC,
    0x01,
    0x7A,
    0x0E,
    0x80,
    0xED,
    0x79,
    0xCD,
    0xAC,
    0x01,
    0xCD,
    0xBC,
    0x01,
    0x7A,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0xED,
    0x79,
    0xCD,
    0xAC,
    0x01,
    0xCD,
    0x9C,
    0x01,
    0xC9,
    0xCD,
    0x8C,
    0x01,
    0xCD,
    0xBC,
    0x01,
    0x7A,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0xCB,
    0x27,
    0x0E,
    0x80,
    0xED,
    0x79,
    0xCD,
    0xAC,
    0x01,
    0xCD,
    0x9C,
    0x01,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x00,
    0x21,
    0x02,
    0xA0,
    0x7E,
    0xCB,
    0x87,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x00,
    0x21,
    0x02,
    0xA0,
    0x7E,
    0xCB,
    0xC7,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x00,
    0x21,
    0x02,
    0xA0,
    0x7E,
    0xCB,
    0x8F,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x00,
    0x21,
    0x02,
    0xA0,
    0x7E,
    0xCB,
    0xCF,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x00,
    0x21,
    0x02,
    0xA0,
    0x7E,
    0xCB,
    0x97,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x00,
    0x21,
    0x02,
    0xA0,
    0x7E,
    0xCB,
    0xD7,
    0x77,
    0xED,
    0x79,
    0xD1,
    0xC1,
    0xC9,
    0xC5,
    0xD5,
    0x0E,
    0x00,
    0xED,
    0x78,
    0xE6,
    0x08,
    0xCB,
    0x3F,
    0xCB,
    0x3F,
    0xCB,
    0x3F,
    0xD1,
    0xC1,
    0xC9,
  };







//--------------------------------------------------------------------------------
// Code that runs between instructions to dump registers

const BYTE inter_inst_code_regdump[] =
  {
    0xF5,
    0xE5,
    0xC5,
    0xD5,
    0xDD,
    0xE5,
    0xFD,
    0xE5,
    0xFD,
    0xE1,
    0xDD,
    0xE1,
    0xD1,
    0xC1,
    0xE1,
    0xF1,
    0x08,
    0xD9,
    0xF5,
    0xE5,
    0xC5,
    0xD5,
    0xD1,
    0xC1,
    0xE1,
    0xF1,
    0xD9,
    0x08,
    0x18,
    0xE2,
  };






























//--------------------------------------------------------------------------------

struct
{
  const char *desc;
  BYTE  *code;
  int length;
}
  const code_list[] =
    {
      {"Copy code to RAM and execute it", example_code_ram,          sizeof(example_code_ram)},
      {"Write value to bank register",    example_code_bank,         sizeof(example_code_bank)},
      {"Write then read RAM",             example_code_ram_chk,      sizeof(example_code_ram_chk)},
      {"Turn LCD shield backlight off",   example_code_lcd_bl_off,   sizeof(example_code_lcd_bl_off)},
      {"Flash turn LCD shield backlight", example_code_lcd_bl_flash, sizeof(example_code_lcd_bl_flash)},
      {"Slow Flash turn LCD shield backlight", example_code_lcd_slow_flash, sizeof(example_code_lcd_slow_flash)},
      {"LCD test",                             example_code_lcd_test, sizeof(example_code_lcd_test)},
      //{"DF playing around test",          example_code_df_test,      sizeof(example_code_df_test)},
      {"-",                               0,                         0},
    };


void cmd_set_example_code(String cmd)
{
  if( cmd.length() == 1 )
    {
      Serial.println(F("Set example code using 'sN' where 'N' is from the example code list"));
      return;
    }

  String arg = cmd.substring(1);

  int code_i = arg.toInt();
  example_code        = code_list[code_i].code;
  example_code_length = code_list[code_i].length;
  
  Serial.print(F("\nExample code now '"));
  Serial.print(code_list[code_i].desc);
  Serial.print(F("  len:"));
  Serial.print(example_code_length);
  Serial.println("'");

  Serial.print("\nCode example "+arg+" has been set in the Mega memory. ");
#if ENABLE_MEGA_ROM_EMULATION
  Serial.println(F("Mega is emulating ROM in this sketch build, and will supply this code to the Z80."));
#else
  Serial.println(F("This sketch build is not emulating ROM, so remember to write"));
  Serial.println(F("it to flash so the hardware runs it."));
#endif
}

void cmd_show_example_code(String cmd)
{
  (void)cmd;

  Serial.println(F("\nCode examples in this build:"));
  Serial.println(F("----------------------------"));
  
  for(int i=0; code_list[i].code != 0; i++)
    {
      Serial.print(i);
      Serial.print(": ");
      Serial.println(code_list[i].desc);
    }

  Serial.println(F("\nUse 's' option of command menu to set one of these code examples"));
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
  (void)cmd;
}

// Traces test code at the t state level, all cycles are shown

void cmd_trace_test_code(String cmd)
{
  (void)cmd;
  boolean running = true;
  unsigned long start, end;

  
#ifdef __UNUSED_CODE__
  int cycle_type = CYCLE_NONE;
  int cycle_dir = CYCLE_DIR_NONE;
  int cycle_n = 0;
#endif
  
  int fast_mode_n = 0;


  unsigned int trigger_address = 0x8000;    // trigger when we hit RAm by default
  boolean trigger_on = false;


#if ENABLE_TIMINGS
  unsigned long average = 0;
  unsigned long t_now = 0, t_last = 0;
#endif
  
  // We have a logical address space for the array of code such that the code starts at
  // 0000H, which is the reset vector

  // Enable IO and memory
  // We will allow the RAM to provide RAM data
  //
#if ENABLE_MEGA_ROM_EMULATION
  // ROM data comes from the Mega
  //
  deassert_signal(SIG_MAPRQM); 
#else
  // ROM data comes from the flash
  //
  assert_signal(SIG_MAPRQM); 
#endif
  assert_signal(SIG_MAPRQI);

  // Clock and monitor the bus signals to work out what to do
  while( running )
    {
      // Half t states so we can examine all clock transitions
      half_t_state();

      //delay(5);

      if( signal_state("M1") == LOW )
        z80_registers.pc = (uint16_t)addr_state();  

      // Dump the status so we can see what's happening
      if ( !fast_mode )
	{
	  Serial.print(F("\nBus state:"));
	  Serial.println(bsm_state_name());

	  cmd_dump_signals();
	  Serial.println("");
	}
      else
	{
	  if( (fast_mode_n % 1000000)==0 )
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
#ifdef __UNUSED_CODE__
      int ioreq = signal_state("IOREQ");
      int maprqm = signal_state("MAPRQM");
      int last_addr = -1;
#endif
      int trace_size;
      
      if ( (rd == HIGH) )
	{
	  // Data bus back to inputs
	  data_bus_inputs();
	}
      
      if ( (wr == LOW) && (mreq == LOW) )
	{
#ifdef __UNUSED_CODE__
	  cycle_dir = CYCLE_DIR_WR;

	  // Write cycle
	  if (mreq == LOW )
	    {
	      cycle_type = CYCLE_MEM;
	    }
#endif
	}

      // Read cycle, put data on the bus, based on address, only emulate FLASH for now
      if ( (rd == LOW) && (mreq == LOW) && (addr_state() < 0x8000))
	{
#ifdef __UNUSED_CODE__
	  cycle_dir = CYCLE_DIR_RD;
	  // Write cycle
	  if (mreq == LOW )
	    {
	      cycle_type = CYCLE_MEM;
	    }
#endif	  
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

	  if ( fast_mode_n == -1 )
            {
	      
              // Casts are to keep the compiler quiet. For the moment the fast_to_address
              // is signed so it can use -1 as a sentinel value
              //
#if 0
              if( fast_to_address != -1 )
                {
                  if( (int16_t)z80_registers.PC == fast_to_address )
                    {
                      fast_mode = false;
                      quiet = false;
                      fast_to_address = -1;
                    }
                }
#endif
	    }

	  if ( (!uninterruptible_command) && (Serial.available() > 0) )
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
	      Serial.print(F("Trigger address reached ("));
	      Serial.print(trigger_address & 0xffff, HEX);
	      Serial.println(")");
	    }
	}
      else
	{
#if ENABLE_TIMINGS
	  t_now = millis();
#endif
	  // Allow interaction
	  if ( trigger_on )
	    {
	      Serial.print(F("\nBreakpoint active:"));
	      Serial.println(trigger_address & 0xffff, HEX);
	    }
	  
	  Serial.println( F("\nTrace Menu") );
          Serial.println( F("==========") );

	  Serial.println(F("t:Mega drive n tstates       f:Mega drive tstates forever"));
	  Serial.println(F("c:Mega drive tstates, continues to given Z80 instruction address"));
	  Serial.println(F("n:Mega drive tstates until next Z80 instruction\n"));
	  Serial.println(F("F:Free run (at ~4.5MHz)      M:Mega provide clock (at ~80Hz)"));
	  Serial.println(F("G:Mega take Z80 bus (BUSREQ) R:Mega release Z80 bus"));
	  Serial.println(F("I:Mega take IO map           i:Hardware take IO map"));
	  Serial.println(F("J:Mega take memory map       j:Hardware take memory map\n"));
	  Serial.println(F("r:reset Z80"));
	  Serial.println(F("1:assert reset               0:deassert reset\n"));
	  Serial.println(F("b:Breakpoint                 B:Toggle breakpoint\n"));
	  Serial.println(F("-:Display trace              =:Display II Trace\n"));
	  Serial.println(F("X:Assert INT x:desaart INT   Y:Assert NMI  y:Deassert NMI\n"));

	  Serial.println(F("return: drive half a clock   q:quit menu"));

#if ENABLE_TIMINGS
	  Serial.print(F("Elapsed:"));
	  Serial.print(t_now-t_last);
	  t_last = t_now;
	  Serial.println(F("ms"));
#endif
	  
	  boolean cmdloop = true;
	  
          String trace_cmd = "";
          Serial.print(F("trace> "));
	  Serial.flush();
	  
	  while( cmdloop )
	    {
              while ( Serial.available() == 0 );

              char c = Serial.read();
              trace_cmd += c;

#if ENABLE_LOCAL_ECHO
              // Echo back key
              //
              Serial.print( c ); Serial.flush();
#endif
	      
	      if( c == '\n' || c == '\r' )
		{
		  //Serial.print("Action ");
		  //Serial.println(trace_cmd);

#if ENABLE_TIMINGS
		  t_last = millis();
#endif
		  switch( trace_cmd.charAt(0) )
		    {
#if ENABLE_TIMINGS
		    case 'z':
		      // Display stored timing information
		      Serial.println(F("Mega CLK timing"));
		      average = 0;
		  
		      for(int i=0; i<NUM_TIMED_CLKS;i++)
			{
			  Serial.print(" ");
			  Serial.print(last_clks[i]);
			  average += last_clks[i];
			}
		      Serial.println("");
		      Serial.print(F("Average:"));
		      Serial.println(average / NUM_TIMED_CLKS);
		  
		      break;
#endif		  
		    case 't':
		      fast_mode = true;
		      quiet = true;
		      delay(100);
		      if( trace_cmd.length() > 2 )
			fast_mode_n = strtol((trace_cmd.c_str())+1, NULL, 10);
		      else
			fast_mode_n = -1;
		      cmdloop = false;
		      break;

		    case 'n':
		      fast_mode = true;
		      quiet = true;
		      delay(100);
		      fast_mode_n = -1;
		      stop_on_m1 = true;
		      fast_to_address = -1;
		      cmdloop = false;

		      // Turn off the flag that indicates that an opcode has been traced. This means that
		      // we will only stop on M1 if a new instruction opcode (and bus transfers for that instruction as we stop
		      // on M1) has been traced. This has the effect of making the 'next command' drive the bus until a new
		      // instruction has been traced. This is a more intuitive behaviour than driving to M1 as that might
		      // only drive until a half completed instruction has finished and not trace anything.
		      opcode_traced = false;

		      // We want to stop on a full instruction, i.e. not after a prefix instruction. This is so the
		      // injected code (between instruction) is only executed when ther eis no outstanding prefix
		      // opcode
		      
		      stop_on_full_instruction = true;
		      break;

		    case 'c':
		      fast_mode = true;
		      quiet = true;
		      delay(100);
		      fast_mode_n = -1;
		      stop_on_m1 = true;
		      fast_to_address = strtol((trace_cmd.c_str())+1, NULL, 16);
		      cmdloop = false;
		      break;

		    case 'f':
		      fast_mode = true;
		      fast_mode_n = -1;
		      quiet = true;
		      delay(100);
		      cmdloop = false;
		      break;

		    case 'g':
		      // reset the instruction trace index so we always have a fresh trace for this
		      // code. It makes it easier to examine later
		      ii_trace_index = 0;
		      
		      fast_mode = true;
		      fast_mode_n = -1;
		      inter_inst = true;
		      inter_inst_trace = true;
		      inter_inst_code = inter_inst_code_regdump;
		      inter_inst_code_length = sizeof(inter_inst_code_regdump);
		      inter_inst_index = 0;
		      quiet = true;
		      uninterruptible_command = true;
		      delay(100);
		      cmdloop = false;

		      // When completed, get register values
		      ii_process_registers = true;
		      break;

		    case 'b':
		      delay(100);
		      trigger_address = strtol((trace_cmd.c_str())+1, NULL, 16);
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

		      assert_signal(SIG_RES);

		      // Mega relinquishes control of reset and clock, letting hw have it
		      //
		      digitalWrite(SW0_Pin, LOW);
		      digitalWrite(SW1_Pin, LOW);

		      // Mega relinquishes IO and memory map
		      //
		      digitalWrite(MAPRQI_Pin, LOW);
		      digitalWrite(MAPRQM_Pin, LOW);

		      // Reset the Z80
		      //
		      reset_z80();

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
		      
		    case 'r':
		      cmd_reset_z80("");
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


		    case '-':
		    case '=':

		      // Display trace data
		      if (trace_cmd.charAt(0)=='=')
			{
			  trace_size = II_TRACE_SIZE;
			  Serial.println(F("Inter Instruction Trace"));
			}
		      else
			{
			  trace_size = TRACE_SIZE;
			  Serial.println(F("Trace"));
			}
		      
		      for(int i=0; i<trace_size;i++)
			{
			  Serial.print(textify_trace_rec(i, (trace_cmd.charAt(0)=='=')?ii_trace: trace));
			  if ( i == ((trace_cmd.charAt(0)=='=')?ii_trace_index: trace_index) )
			    {
			      Serial.println("  <- oldest");
			    }
			  else
			    {
			      Serial.println("");
			    }
			}
		      break;

		    case 'X':
		      assert_signal(SIG_INT);
		      break;

		    case 'x':
		      deassert_signal(SIG_INT);
		      break;
		      
		    case 'Y':
		      assert_signal(SIG_NMI);
		      break;

		    case 'y':
		      deassert_signal(SIG_NMI);
		      break;
		      
		    case '\r':
		      cmdloop = false;
		      break;
		    }
		
		  Serial.print(F("trace> ")); Serial.flush();
		  trace_cmd = "";
		}
	      
	    }
	}
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// Upload an intex hex file and write to bank given
// Flow control:
// After each line the transmitter should wait for a '+' character
//
void upload_to_bank(int bank)
{
  char c = 0;
  boolean done = false;
  char ascii_data[255];
  int idx = 0;
  int length = 0;
  int address = 0;
  int j;
  char ascii_byte[3];
  char ascii_address[5];
  
  int byte;
  int bank_addr = 0;

  // We have to wait here for a signal that the upload program has started
  boolean started = false;
  boolean exit_upload = false;
  
  while( !started )
    {
      while ( Serial.available() == 0)
	{
	}
      c = Serial.read();
      
      switch(c)
	{
	case 'S':
	  started = true;
	  Serial.println(F("Started"));
	  break;
	  
	case ' ':
	  exit_upload = true;
	  break;
	}

    }
  
  if ( exit_upload )
    {
      // Aborted
      return;
    }
  
  while( !done )
    {
      while ( Serial.available() == 0)
	{
	}
      c = Serial.read();

      switch(c)
	{
	case '\r':
	case '\n':
	  break;
	  
	case '-':
	  // End of line, write to flash
	  ascii_byte[0] = ascii_data[1];
	  ascii_byte[1] = ascii_data[2];
	  ascii_byte[2] = '\0';
	  sscanf(ascii_byte, "%x", &length);
	  
	  ascii_address[0] = ascii_data[3];
	  ascii_address[1] = ascii_data[4];
	  ascii_address[2] = ascii_data[5];
	  ascii_address[3] = ascii_data[6];
	  ascii_address[4] = '\0';
	  sscanf(ascii_address, "%x", &address);

	  // Don't display debug for speed
	  if ( 1 )
	    {
	      Serial.print(F("Address:"));
	      Serial.println(ascii_address);
	      Serial.print(F("Length:"));
	      Serial.println(length);
	    }
	  
	  if ( length == 0 )
	    {
	      done = true;
	    }
	  else
	    {
	      for(j=9; j<idx-3; j+=2)
		{
		  ascii_byte[0] = ascii_data[j];
		  ascii_byte[1] = ascii_data[j+1];
		  ascii_byte[2] = '\0';

		  sscanf(ascii_byte, "%x", &byte);
		  // Write data to flash
		  flash_write_byte(bank, bank_addr++, byte);
		}
	    }

	  // Send signal next line is needed
	  Serial.print("+");
	  idx = 0;
	  break;

	default:
	  ascii_data[idx++] = c;
	  break;
	}
    }
}



////////////////////////////////////////////////////////////////////////////////
//
// Memory monitor
//
//
////////////////////////////////////////////////////////////////////////////////



void cmd_memory(String cmd)
{
  (void)cmd;

  boolean running = true;
  boolean cmdloop = true;
  int address = 0;
  int working_address = 0;
  int working_space = SIG_MREQ;
  unsigned long start, end;
  
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
      Serial.print(F("Working address: "));
      Serial.print((unsigned short)working_address, HEX);

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

      
      Serial.print(F(" Bus state:"));
      Serial.println(bsm_state_name());

      Serial.println(F(" (r:Display memory  a:Set address  w:write byte  e:Erase flash sector         E:Erase chip)"));
      Serial.println(F(" (m:Mem space       i:IO space     b:Set bank    X:write example code to 0000 Y:write code to all banks)"));
      Serial.println(F(" (u:upload bin to flash bank 0)"));
      Serial.println(F(" (return:next q:quit)"));
      
      cmdloop=true;

      Serial.print(F("memory> ")); Serial.flush();
      String memory_cmd = "";
      while( cmdloop )
	{
          while( Serial.available() == 0 );

          char c = Serial.read();
          memory_cmd += c;

#if ENABLE_LOCAL_ECHO	  
          // Echo back key
          //
          Serial.print( c ); Serial.flush();
#endif
          if( c == '\n' || c == '\r' )
	    {
#if ENABLE_TIMINGS
	      start = millis();
#endif
	    
	      switch( memory_cmd.charAt(0) )
		{
		case 'u':
		  // Upload binary file to flash bank 0
		  Serial.println(F("Start upload of binary file. Will write to bank 0"));
		  
		  upload_to_bank(0);
		  break;

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
		  write_cycle(working_address, strtol((memory_cmd.c_str())+1, NULL, 16), working_space);
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
		  working_address = strtol((memory_cmd.c_str())+1, NULL, 16);
		  cmdloop=false;
		  break;
              
		case 'b':
		  // Write a bank value to bank register
		  delay(100);
              
		  write_cycle(IO_ADDR_BANK, strtol((memory_cmd.c_str())+1, NULL, 16), SIG_IOREQ);
		  cmdloop=false;
		  break;

		case 'e':
		  // Erase a sector
		  delay(100);

		  Serial.println(F("Starting erase..."));
		  flash_erase(FLASH_ERASE_SECTOR_CMD, strtol((memory_cmd.c_str())+1, NULL, 16));
		  Serial.println("done.");
		  cmdloop=false;
		  break;

		case 'E':
		  flush_serial();
		  
		  // Erase a sector
		  Serial.println(F("Starting chip erase..."));
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
		      Serial.print(F("Writing to bank "));
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

#if ENABLE_TIMINGS
	      end = millis();
	      Serial.print(F("Elapsed:"));
	      Serial.print(end-start);
	      Serial.println("ms");
#endif		  

	      memory_cmd = "";
	      Serial.print(F("memory> ")); Serial.flush();
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

void cmd_reset_z80(String cmd)
{
  (void)cmd;

  reset_z80();

  Serial.println( F("\n------------------") );
  Serial.println( F("Z80 has been reset") );
  Serial.println( F("------------------") );
}


// Null cmd function
void cmd_dummy(String cmd)
{
  (void)cmd;
}

String cmd;
struct
{
  String     cmdname;
  String     desc;
  CMD_FPTR   handler;
} cmdlist [] =
  {
    {"g",    "Grab the Z80",       cmd_grab_z80},
    {"t",    "Trace test code",    cmd_trace_test_code},
    {"l",    "List example code",  cmd_show_example_code},
    {"s",    "Set example code",   cmd_set_example_code},
    {"m",    "Memory management",  cmd_memory},
    {"r",    "Reset the Z80",      cmd_reset_z80},
    {"---",  "",                   cmd_dummy},
  };



void print_commands()
{
  int i = 0;
  
  Serial.println( F("\nCommand Menu") );
  Serial.println( F("============\n") );

  while( cmdlist[i].desc != "" )
    {
      Serial.print  ( cmdlist[i].cmdname );
      Serial.print  ( F(": ") );
      Serial.println( cmdlist[i].desc );
      i++;
    }
}

// Interaction with the Mega from the host PC is through a 'monitor' command line type interface.

const String monitor_cmds = "t: Trace code l:list example code s:set code m:memory";

void run_monitor()
{
  char c;
  int i;
  String test;

  if( Serial.available()>0 )
    {
      // Build up a command string from the serial input. When we received end-of-line
      // look up the received string in the commands table and run the handler function
      // for that command
      //
      c = Serial.read();

#if ENABLE_LOCAL_ECHO      
      // Echo back key
      //
      Serial.print( c ); Serial.flush();
#endif
      
      switch(c)
	{
	case '\r':
	case '\n':
	  for(i=0; cmdlist[i].cmdname != "---"; i++)
	    {
	      test = cmd.substring(0, (cmdlist[i].cmdname).length());
	      if( test == cmdlist[i].cmdname )
		{
		  unsigned long start, end;

		  // Found, run the handler then represent the menu
		  //
		  
#if ENABLE_TIMINGS
		  start = millis();
#endif
		  (*(cmdlist[i].handler))(cmd);
#if ENABLE_TIMINGS
		  end = millis();
		  Serial.print(F("Elapsed:"));
		  Serial.print(end-start);
		  Serial.println(F("ms"));
#endif		  
		  print_commands();
		  
		  Serial.print("\n");
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
// Gets a numeric parameter from the serial input
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

  // initialize serial communication
  Serial.begin(115200);

  for(int i=0; i<24; i++ )
    Serial.println("");
    
  Serial.println(F("Z80 Shield Monitor"));
  Serial.println(F("    (Set line ending to carriage return)"));

  // Use the command menu function because it outputs status to the user
  //
  cmd_grab_z80(F("initialisation"));

  print_commands();

  // Initialise signals
  initialise_signals();

#if ENABLE_MEGA_ROM_EMULATION
  // Set the Mega to supply the first example bit of code. Use the user function
  // because it prints a message saying what it has done
  //
  cmd_set_example_code("s0");

  digitalWrite(MAPRQI_Pin, HIGH);
  digitalWrite(MAPRQM_Pin, HIGH);
#else
  Serial.println("\nThis sketch build is not emulating ROM, so we're going to");
  Serial.println("run whatever is in the flash chip");

  digitalWrite(MAPRQI_Pin, LOW);
  digitalWrite(MAPRQM_Pin, LOW);
#endif
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
//
//
// Disassembler
//
//

#if 1

#define ENDIAN_SWAP(x) (((x&0xff)<<8)+((x&0xff00)>>8))

const char * const disasm_desc[] =
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

#define NUM_DISASM_DESC (sizeof(disasm_desc))

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

  //printf("\n%s %s literal", original_word, is_literal?"is":"is not");
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
  //printf("\nField %d: Add %d(%d) : %c     -> %d", index, value, radix, fields[index].name, fields[index].value);
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

  //printf("\nNew field set field %c to %d", name, value);
  fields[num_fields].name = name;
  fields[num_fields].value = value;
  num_fields++;
  //printf("\n Now %d fields", num_fields);
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
  
  //printf("\n%s", __FUNCTION__);
  
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
	  //	  printf("\n>>> Hex ");
	  
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
		  //printf("\n>>>>%d %c", char_i, *word);
		  
		  // process nibble
		  charval = (byte & nibble_mask) >> (4*(char_i-1));
		  
		  //printf("\n>>>>%d %d %d", charval, nibble_mask, byte);
		  
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
	  //printf("\n>>> Binary");


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
		  //printf("\n>>>>%d %c", char_i, *word);
		  
		  // process bit
		  charval = (byte & bit_mask) >> (1*(char_i-1));
		  
		  //printf("\n>>>>%d %d %d", charval, bit_mask, byte);
		  
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
  
  //printf("\nWord:'%s' Byte:%02X", word, byte);
  //printf("\nWord value    :%02X", vword);
  //printf("\nWord and byte %s", matches?"match":"do not match");
  //printf("\n");
  return(matches);
}

// Returns number of matching bytes

boolean disasm_desc_match(char *disasm_desc, unsigned char *bytes)
{
  int di;    // disasm_desc string index
  int bi;    // byte list index
  boolean all_match = true;
  boolean done = false;
  int num_words = 0;
  
  char word[20];
  char c[2];

  //printf("\nDISASM_DESC:'%s'", disasm_desc);

  // The disasm_desc has words that encode the instruction, process them in order
  // one word per byte
  di = 0;
  word[0] = '\0';

  // Get words and process them
  while ( !done && (strlen(disasm_desc) > 0) )
    {
      //printf("\ndoing '%c'", *disasm_desc);
      
      switch(*disasm_desc)
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
	  c[0] = *disasm_desc;
	  c[1] = '\0';
	  strcat(word, c);

	  if( strlen(disasm_desc) == 1 )
	    {
	      if( !match_word(word, *(bytes++)) )
		{
		  all_match = false;
		}

	      num_words++;
	    }
	  break;
	}

     
      disasm_desc++;
    }
  
  // If the number of matches equals the length then the next word should be the
  // format string
  if( all_match )
    {
      //printf("\nBytes match ");
      return(num_words);
    }
  else
    {
        return(0);
    }
}

// We have a disasm_desc and some bytes, we now have to decode the disasm_desc and create the
// disassembled instruction

char decode_string[40];

char * disasm_desc_decode(char *disasm_desc, unsigned char *bytes)
{

  int i;
  char fmt_str[40];
  char field_val[10];
  int field_i;
  
  dump_fields();
  
  //printf("\nDECODE:%s", disasm_desc);
  boolean done = false;
  decode_string[0] = '\0';
  
  fmt_str[0] = '\0';
  
  for(i=0; (i<strlen(disasm_desc)) && !done; i++)
    {
      switch(*(disasm_desc+i))
	{
	case ':':
	  // Get the format string
	  strcpy(fmt_str, disasm_desc+i+1);
	  fmt_str[strlen(disasm_desc+i)-2] = '\0';
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
  
  //printf("\nFMT str:'%s'", fmt_str);
  //printf("\n                                               %s", decode_string);
  
}

char disasm_str[30];

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
      for(i=0; i< NUM_DISASM_DESC; i++)
	{
	  // Reset field values
	  num_fields = 0;

	  // End of disasm_desc list
	  if( disasm_desc[i] == NULL )
	    {
	      break;
	    }

	  // Does this one match?
	  if ( (num_match = disasm_desc_match(disasm_desc[i], bytes)) > 0 )
	    {
	      any_match = true;
	      
	      // Yes, get decoded instruction
	      disasm_desc_decode(disasm_desc[i], bytes);
	      
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
	  //printf("\nNo match at:");
	  for(i=0; i<10; i++)
	    {
	      //printf("\n%02X", *(bytes+i));
	    }
	  break;
	}
    }
}

#endif
