//
// Arduino Mega sketch for the Z80 shield
// Serial monitor based control program
//

#define VERTICAL_LABELS  0

typedef unsigned char BYTE;
typedef void (*FPTR)();
typedef void (*CMD_FPTR)(String cmd);

// Pin definitions

#if 0

const int D0_Pin      = 2;
const int D1_Pin      = 3;
const int D2_Pin      = 4;
const int D3_Pin      = 5;
const int D4_Pin      = 6;
const int D5_Pin      = 7;
const int D6_Pin      = 8;
const int D7_Pin      = 9;

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
const int NMI_Pin     = 51;
const int INT_Pin     = 42;
const int M1_Pin      = 39;
const int WAIT_Pin    = 40;
const int A_CLK_Pin   = 38;
const int A_RES_Pin   = 36;

const int SW0_Pin     = 34;
const int SW1_Pin     = 32;
#else

const int D0_Pin      = 21;
const int D1_Pin      = 19;
const int D2_Pin      = 10;
const int D3_Pin      = 11;
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
const int A8_Pin      = 10;
const int A9_Pin      = 11;
const int A10_Pin     = 64;
const int A11_Pin     = 65;
const int A12_Pin     = 66;
const int A13_Pin     = 69;
const int A14_Pin     = 53;
const int A15_Pin     = 52;

const int BUSREQ_Pin  = 45;
const int BUSACK_Pin  = 24;
const int WR_Pin      = 32;
const int RD_Pin      = 34;
const int MREQ_Pin    = 30;
const int IOREQ_Pin   = 44;
const int HALT_Pin    = 22;
const int NMI_Pin     = 39;
const int INT_Pin     = 18;
const int M1_Pin      = 35;
const int WAIT_Pin    = 37;
const int A_CLK_Pin   = 41;
const int A_RES_Pin   = 33;
const int RFSH_Pin    = 31;
const int SW0_Pin     = 34;
const int SW1_Pin     = 32;

#endif

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
  
  while(  signal_state("BUSACK") == HIGH)
    {
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

// Generate an event and process it
void signal_event(int sig, int sense)
{
  Serial.print(signal_list[sig].signame);
  
  if( sense == EV_ASSERT)
    {
      Serial.println(" ASSERT");
    }
  else 
    {
      Serial.println(" DEASSERT");
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
  //arg = cmd.substring(1);

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

void cmd_free_z80()
{
}

  
void cmd_dump_signals()
{
  unsigned int address;
  unsigned int data;

  
  Serial.println("");
  Serial.println("Z80 State");  

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

BYTE example_code[] =
  {
    0x3e, 0xaa,          // LOOP:   LD A, 03EH
    0x21, 0x12, 0x34,    //         LD HL 01234H
    0x77,                //         LD (HL), A
    0x23,                //         INC HL
    0xc3, 0x12, 0x34     //         JR LOOP
  };


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

      // Dump the status so we can see what's happening
      cmd_dump_signals();

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
	  Serial.print("Putting data on bus ");
	  Serial.print(addr_state(), HEX);
	  Serial.print(" ");
	  Serial.print(example_code[addr_state() & 0xff], HEX);
	}

      // Allow interaction
      Serial.println(" (return:next q:quit 1:assert reset 0:deassert reset d:dump regs)");
      
      while ( !Serial.available())
	{
	}

      boolean cmdloop = true;
      
      while( cmdloop )
	{
	  switch( Serial.read())
	    {
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
    {"t",         cmd_run_test_code},
    {"---",       cmd_dummy},
  };

// Interaction with the Mega from the host PC is through a 'monitor' command line type interface.

void run_monitor()
{
  char c;
  int i;
  String test;
  
  if( Serial.available() )
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


