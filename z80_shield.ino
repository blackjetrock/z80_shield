//
// Arduino Mega sketch for the Z80 shield
// Serial monitor based control program
//

typedef unsigned char BYTE;
typedef void (*FPTR)();
typedef void (*CMD_FPTR)(String cmd);

// Pin definitions
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

////////////////////////////////////////////////////////////////////////////////
//
// Utility functions
//
////////////////////////////////////////////////////////////////////////////////

// Assert a control line
void assert_ctrl(int what)
{
  // Always active low
  digitalWrite(what, LOW);
}

// De-assert a control line
void de_assert_ctrl(int what)
{
  // Always active low
  digitalWrite(what, HIGH);
}

// Do a clock cycle or T state (high then low)
//

void t_state()
{
  digitalWrite(A_CLK_Pin, HIGH);
  digitalWrite(A_CLK_Pin, LOW);
}

// Request and get the bus
void bus_request()
{
  // Bus request low
  assert_ctrl(BUSREQ_Pin);
  
  // We now clock until busack is low
  // Send one clock with no check
  t_state();
  
  while(  digitalRead(BUSACK_Pin) == HIGH)
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
  assert_ctrl(A_RES_Pin);

  // Address bus is driven by z80
  addr_bus_inputs();

  // We leave data bus alone for now
  data_bus_inputs();
  

  pinMode ( BUSACK_Pin, INPUT );

  // 
}


////////////////////////////////////////////////////////////////////////////////
//
// Commands
//
////////////////////////////////////////////////////////////////////////////////

// Example Z80 code

//        LD  A, 0AAH
// LOOP:  LD  HL, 01234H
//        LD  (HL), A
//        INC HL
//        JR  LOOP
//

BYTE example_code[] =
  {
    0x3e, 0xaa,          // LOOP:   LD A, 03EH
    0x21, 0x12, 0x34,    //         LD HL 01234H
    0x77,                //         LD (HL), A
    0x23,                //         INC HL
    0x18, -9             //         JR LOOP
  };
  
// Grab the z80, ready for other actions like single stepping
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
// Runs the short piece of test Z80 code under full Mega control. This means
// that the Z80 code does not run from the RAM or flash chip on the board, but
// comes from the Mega
////////////////////////////////////////////////////////////////////////////////

void cmd_run_test_code()
{

}

////////////////////////////////////////////////////////////////////////////////
//
// Command Table
//
////////////////////////////////////////////////////////////////////////////////

const int NUM_CMDS = 2;  
String cmd;
struct
{
  String cmdname;
  CMD_FPTR   handler;
} cmdlist [NUM_CMDS] =
  {
    {"g",         cmd_grab_z80},
    {"t",         cmd_run_test_code},
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
  // Fixed initialisation. These data directions are fixed.

  // Control pins
  pinMode(SW0_Pin, OUTPUT);
  pinMode(SW1_Pin, OUTPUT);

  // Select Mega control of reset and clock
  digitalWrite(SW0_Pin, HIGH);
  digitalWrite(SW1_Pin, LOW);


  initialise_z80_for_control();
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


