//
// Arduino Mega sketch for the Z80 shield
// Serial monitor based control program
//

typedef unsigned char BYTE;

typedef void (*FPTR)();
typedef void (*CMD_FPTR)(String cmd);

// Pin definitions
const D0_Pin      = 2;
const D1_Pin      = 3;
const D2_Pin      = 4;
const D3_Pin      = 5;
const D4_Pin      = 6;
const D5_Pin      = 7;
const D6_Pin      = 8;
const D7_Pin      = 9;

const A0_Pin      = 22;
const A1_Pin      = 23;
const A2_Pin      = 24;
const A3_Pin      = 25;
const A4_Pin      = 26;
const A5_Pin      = 27;
const A6_Pin      = 28;
const A7_Pin      = 29;
const A8_Pin      = 53;
const A9_Pin      = 52;
const A10_Pin     = 51;
const A11_Pin     = 50;
const A12_Pin     = 10;
const A13_Pin     = 11;
const A14_Pin     = 12;
const A15_Pin     = 13;

const BUSREQ_Pin  = 45;
const BUSACK_Pin  = 44;
const WR_Pin      = 48;
const RD_Pin      = 49;
const MREQ_Pin    = 47;
const IOREQ_Pin   = 46;
const HALT_Pin    = 43;
const NMI_Pin     = 51;
const INT_Pin     = 42;
const M1_Pin      = 39;
const WAIT_Pin    = 40;
const A_CLK_Pin   = 0;
const A_RESET_Pin = 0;

////////////////////////////////////////////////////////////////////////////////
//
// Utility functions
//
////////////////////////////////////////////////////////////////////////////////

// Assert a control line
void assert_ctrl(int what)
{
  // Always active low
  digitalWrite(what, 0);
}

// De-assert a control line
void de_assert_ctrl(int what)
{
  // Always active low
  digitalWrite(what, 1);
}

void take_bus()
{
  pinMode ( BUSREQ, OUTPUT );
  digitalWrite ( BUSREQ_Pin, 0 );
}

// Set the control up to use the Mega to control everything

void initialise_z80_for_control()
{
  // Take over bus as a quick way to get the Z80 off the bus
  take_bus();

  // Put processor in reset
  assert_ctrl(A_RES);
  
  pinMode ( D0 , INPUT );
  pinMode ( D1 , INPUT );
  pinMode ( D2 , INPUT );
  pinMode ( D3 , INPUT );
  pinMode ( D4 , INPUT );
  pinMode ( D5 , INPUT );
  pinMode ( D6 , INPUT );
  pinMode ( D7 , INPUT );
  pinMode ( A0 , INPUT );
  pinMode ( A1 , INPUT );
  pinMode ( A2 , INPUT );
  pinMode ( A3 , INPUT );
  pinMode ( A4 , INPUT );
  pinMode ( A5 , INPUT );
  pinMode ( A6 , INPUT );
  pinMode ( A7 , INPUT );
  pinMode ( A8 , INPUT );
  pinMode ( A9 , INPUT );
  pinMode ( A10, INPUT );
  pinMode ( A11, INPUT );
  pinMode ( A12, INPUT );
  pinMode ( A13, INPUT );
  pinMode ( A14, INPUT );
  pinMode ( A15, INPUT );

  pinMode ( BUSACK, INPUT );

  // 
}


////////////////////////////////////////////////////////////////////////////////
//
// Commands
//
////////////////////////////////////////////////////////////////////////////////

// Modify the buffer
void cmd_modify(String cmd)
{
  String arg;
  
  Serial.println("MOD");
  arg = cmd.substring(1);

  stored_bytes[indx] = arg.toInt();
  
}

////////////////////////////////////////////////////////////////////////////////
//
// Command Table
//
////////////////////////////////////////////////////////////////////////////////

const int NUM_CMDS = 9;  
String cmd;
struct
{
  String cmdname;
  CMD_FPTR   handler;
} cmdlist [NUM_CMDS] =
  {
    {"m",         cmd_modify},
    {"clear",     cmd_clear},
    {"display",   cmd_display},
    {"next",      cmd_next},
    {"prev",      cmd_prev},
    {"i",         cmd_index},
    {"close",     cmd_close},
    {"writefile", cmd_writefile},
    {"send",      cmd_send},
  };

void run_monitor()
{
  char c;
  int i;
  String test;
  
  if( Serial.available() )
    {
      c = Serial.read();

      switch(c)
	{
	case '\r':
	case '\n':
	  // We have a command, process it
	  Serial.println("'"+cmd+"'");  
	  for(i=0; i<NUM_CMDS; i++)
	    {
	      test = cmd.substring(0, (cmdlist[i].cmdname).length());
	      Serial.println("'"+test+"'");
	      if( test == cmdlist[i].cmdname )
		{
		  (*(cmdlist[i].handler))(cmd);
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
  

  initialise_z80();
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


