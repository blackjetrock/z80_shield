# z80_shield
Z80 Shield project

This is a Z80 shield. Information is here:

https://trochilidae.blogspot.com/2019/12/z80-arduino-using-mega-as-debugger-ever.html

The PCB is powered by the Mega when it is plugged in and by a USB socket when running standalone as a single board computer.
Only run from one USB socket at a time, i.e. don't plug the PCB USB in if there's a Mega attached (powered or not).

This is a work in progress, two of the latest V2.0 PCBs have been built and work. It's a medium difficulty build, there's some 
surface mount and the flash chip is a package that isn't the easiest to solder but nothing too impossible.

Let me know about any problems.


Quick Start
===========

The example that follows will make a lot more sense if you look at the Z80 databook and study the bus transactions, if you aren't alredy familiar with them. The Z80 shield tracing is at the clock edge level and is very detailed.

Here's an example of what the Z80 Shield can do. It takes one of the built-in code examples and runs it. The example 
code we will run is this:

    0x31, 0x00, 0x90,    // set stack up
    0x3e, 0xaa,          // LOOP:   LD A, 03EH
    0x21, 0x34, 0x82,    //         LD HL 01234H
    0x77,                //         LD (HL), A
    0x7e,                //         LD   A,(HL)
    0x23,                //         INC HL
    0xc3, 0x5, 0x0     //         JR LOOP

It's in the Arduino sketch that is run on the Mega, there's no need to assemble or download it.

To interact with the shield you can use the Arduino SDK serial monitor, or the z80s_term.tcl script. The script is a 
bit better as it has support for register dumping and some other things.

Plug a USB cable into your computer and the other end into the USB connector on the Mega. (The USB connector on the shield 
is used to power the shield when it is running as a stand-alone single board computer (SBC).

Determine which ttyUSB the shield is attached as (Run dmesg after plugging the shield in and check for the last device 
created).

Clone the github repository.

Run z80_shield/software/scripts/z80s_term.tcl:

  ./z80s_term.tcl /dev/ttyUSB0

If you can't access the USB device without running as root, then run as root or sort the permissions out:

Opening /dev/ttyUSB0
Error in startup script: couldn't open "/dev/ttyUSB0": permission denied
    while executing
"open $device r+"
    invoked from within
"set f [open $device r+]"
    (file "./z80s_term.tcl" line 12)


Run as root:

sudo ./z80s_term.tcl /dev/ttyUSB0

Two windows should appear. One is a terminal window that you type commands in and the other is a window where the register 
contents will appear when you ask for them.

You should see something like this in the terminal window:

Z80 Shield Monitor
    (Set line ending to carriage return)
-------------------------------------------------------------------
The Arduino has grabbed the Z80, the Z80 is now the Arduino's slave
-------------------------------------------------------------------
Command Menu
============
g: Grab the Z80
t: Trace test code
l: List example code
s: Set example code
m: Memory management
r: Reset the Z80
This sketch build is not emulating ROM, so we're going to
run whatever is in the flash chip

This is the main menu for the Z80 shield. the other window should have something like:

  PC : .... ....
  SP : .... ....
  AF : .... ....
  BC : .... ....
  DE : .... ....
  HL : .... ....
  AF': .... ....
  BC': .... ....
  DE': .... ....
  HL': .... ....
  IX : .... ....
  IY : .... ....
  I  : .... ....
  R  : .... ....

in it. This is where the Z80 register contents will be displayed if you ask for them to be dumped.

Select option 'l' (lower case 'L') from the main menu:

Code examples in this build:
----------------------------
0: Copy code to RAM and execute it
1: Write value to bank register
2: Write then read RAM
3: Turn LCD shield backlight off
4: Flash turn LCD shield backlight
5: Slow Flash turn LCD shield backlight
6: LCD test

This is a list of the example code in the sketch. We will use option 2 for this example.
Select it by typing 's2':

s2
Example code now 'Write then read RAM  len:14'
Code example 2 has been set in the Mega memory. This sketch build is not emulating ROM, so remember to write
it to flash so the hardware runs it.

As the displayed message states, the Z80 shield is set up by default to run code from the flash chip, so the code has to be written to the flash chip in order to run it. This is done in the memory menu:

Type 'm' at the main menu:

m
Clocking..
Clocking..
Clocking..
Clocking..
Clocking..
Clocking..
Clocking..
Addr:0000  Data:FF
BUSREQ: 0      (Mega --> Z80)     - Asserted, means Mega is controlling the Z80's buses
BUSACK: 0      (Z80  --> Mega)    - Asserted, means Z80 acknowledges it's not in control of its buses
    M1: 1      (Z80  --> Mega)
  MREQ: 1      (Z80  --> Mega)
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 1      (Z80  --> Mega)
    WR: 1      (Z80  --> Mega)
    RD: 1      (Z80  --> Mega)
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 1      (Mega --> Z80)
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 1      (Z80  --> Mega)
BUSACK ASSERT
Working address: 0 Space:MEM  Bus state:Idle
 (r:Display memory  a:Set address  w:write byte  e:Erase flash sector         E:Erase chip)
 (m:Mem space       i:IO space     b:Set bank    X:write example code to 0000 Y:write code to all banks)
 (d:Disassemble address)
 (u:upload bin to flash bank 0)
 (return:next q:quit)
memory> 

You are now in th ememory menu and can perform tasks to do with the memory devices on the Z80 shield. We need to erase the chip and then write the selected example code to the flash chip (in every sector).

Type 'E' to erase the chip:

memory> E
Starting chip erase...
done.
memory> Addr:0000  Data:FF
BUSREQ: 0      (Mega --> Z80)     - Asserted, means Mega is controlling the Z80's buses
BUSACK: 0      (Z80  --> Mega)    - Asserted, means Z80 acknowledges it's not in control of its buses
    M1: 1      (Z80  --> Mega)
  MREQ: 1      (Z80  --> Mega)
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 1      (Z80  --> Mega)
    WR: 1      (Z80  --> Mega)
    RD: 1      (Z80  --> Mega)
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 1      (Mega --> Z80)
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 1      (Z80  --> Mega)
Working address: 0 Space:MEM  Bus state:Idle
 (r:Display memory  a:Set address  w:write byte  e:Erase flash sector         E:Erase chip)
 (m:Mem space       i:IO space     b:Set bank    X:write example code to 0000 Y:write code to all banks)
 (d:Disassemble address)
 (u:upload bin to flash bank 0)
 (return:next q:quit)
memory> 

It takes a while to erase the chip, the 'done' message will appear a few seconds after the erase starts.

We now have a blank flash chip and can write the code to all sectors. type 'Y':

memory> Y
Writing to bank 0

Writing to bank 1
Writing to bank 2
Writing to bank 3
Writing to bank 4
Writing to bank 5
Writing to bank 6
Writing to bank 7
Writing to bank 8
Writing to bank 9
Writing to bank 10
Writing to bank 11
Writing to bank 12
Writing to bank 13
Writing to bank 14
Writing to bank 15
memory>

(At the moment you may have to hit enter to get these messages to appear, it's a bug).
The code is now written to the flash chip. To see it set the address to 0000 by typing a0000:

memory> a0000
memory> Addr:00FF  Data:00
BUSREQ: 0      (Mega --> Z80)     - Asserted, means Mega is controlling the Z80's buses
BUSACK: 0      (Z80  --> Mega)    - Asserted, means Z80 acknowledges it's not in control of its buses
    M1: 1      (Z80  --> Mega)
  MREQ: 1      (Z80  --> Mega)
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 1      (Z80  --> Mega)
    WR: 1      (Z80  --> Mega)
    RD: 1      (Z80  --> Mega)
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 1      (Mega --> Z80)
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 1      (Z80  --> Mega)
Working address: 0 Space:MEM  Bus state:Idle
 (r:Display memory  a:Set address  w:write byte  e:Erase flash sector         E:Erase chip)
 (m:Mem space       i:IO space     b:Set bank    X:write example code to 0000 Y:write code to all banks)
 (d:Disassemble address)
 (u:upload bin to flash bank 0)
 (return:next q:quit)
memory> 

The working address is now 0000. Read memory from this address using the 'r' command:

memory> r

0000: 31 00 90 3E AA 21 34 82 77 7E 23 C3 05 00 FF FF 
0010: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
0020: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
0030: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
0040: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
0050: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
0060: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
0070: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
0080: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
0090: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
00A0: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
00B0: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
00C0: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
00D0: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
00E0: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
00F0: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
memory> 

You can see the example code at address 0000. It's a short piece of code, but the bytes displayed should agree with those in the example code above.

We can now single step this code and see the values of the registers change as the code runs. To do this type 'q' to get back to the main menu:

memory> q
memory> Command Menu
============
g: Grab the Z80
t: Trace test code
l: List example code
s: Set example code
m: Memory management
r: Reset the Z80

Then type 't' to enter the trace menu:

t
Bus state:Idle
Addr:8000  Data:00
BUSREQ: 1      (Mega --> Z80)
BUSACK: 0      (Z80  --> Mega)    - Asserted, means Z80 acknowledges it's not in control of its buses
    M1: 1      (Z80  --> Mega)
  MREQ: 0      (Z80  --> Mega)    - Asserted, means address bus holds a memory address for a read or write
 IOREQ: 0      (Z80  --> Mega)    - Asserted, means lower half of address bus holds an IO address for a read or write
  RFSH: 1      (Z80  --> Mega)
    WR: 0      (Z80  --> Mega)    - Asserted, means the data bus holds a value to be written
    RD: 1      (Z80  --> Mega)
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 0      (Mega --> Z80)     - Asserted, means Z80 is in the second half of a T-state
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 1      (Z80  --> Mega)

  MREQ ASSERT
State: Memory Access
Allowing RAM to put data on bus8000: 0 IOREQ ASSERT
    WR ASSERT
State: Memory Write Access
   CLK ASSERT
Trace Menu
==========
t:Mega drive n tstates       f:Mega drive tstates forever
c:Mega drive tstates, continues to given Z80 instruction address
n:Mega drive tstates until next Z80 instruction
F:Free run (at ~4.5MHz)      M:Mega provide clock (at ~80Hz)
G:Mega take Z80 bus (BUSREQ) R:Mega release Z80 bus
I:Mega take IO map           i:Hardware take IO map
J:Mega take memory map       j:Hardware take memory map
r:reset Z80
1:assert reset               0:deassert reset
b:Breakpoint                 B:Toggle breakpoint
-:Display trace              =:Display II Trace
X:Assert INT x:desaart INT   Y:Assert NMI  y:Deassert NMI
return: drive half a clock   q:quit menu
trace> 

This is the trace menu. At the top of the display we have:

Bus state:Idle
Addr:8000  Data:00

This is a display of the Z80 bus state (there's a state machine in the Mega sketch that follows the Z80 bus state so that we can do things with the bus). Following this is the current bus address value (value of the address lines A0 to A15) and the data bus value (data lines D0 to D7). The address and data bus values are read from the hardwre lines on the Z80.

Below this is a display of the current state of the bus control lines:

BUSREQ: 1      (Mega --> Z80)
BUSACK: 0      (Z80  --> Mega)    - Asserted, means Z80 acknowledges it's not in control of its buses
    M1: 1      (Z80  --> Mega)
  MREQ: 0      (Z80  --> Mega)    - Asserted, means address bus holds a memory address for a read or write
 IOREQ: 0      (Z80  --> Mega)    - Asserted, means lower half of address bus holds an IO address for a read or write
  RFSH: 1      (Z80  --> Mega)
    WR: 0      (Z80  --> Mega)    - Asserted, means the data bus holds a value to be written
    RD: 1      (Z80  --> Mega)
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 0      (Mega --> Z80)     - Asserted, means Z80 is in the second half of a T-state
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 1      (Z80  --> Mega)

The direction of the control line is shown and a description of what the current state means. The Z80 datasheet and data book describe this in detail.

Then there's a section that shows the output from the bus state machine. This is a decode of the states that the bus has passed through. This won't mean a lot unless you are familiar with the Z80 bus states. that's all described in the Z80 data books.

In this case the bus has asserted MREQ:

  MREQ ASSERT
  
  which is a memory access:
  
State: Memory Access

The Mega is allowing the device at this address (the RAM chip) to put data on the data bus:

Allowing RAM to put data on bus8000: 0 IOREQ ASSERT

Then WR was asserted:

    WR ASSERT

Which is a (memory) write access:

State: Memory Write Access

then the clock was assered:


CLK ASSERT

To start the example code running you have to reset the Z80. You can do that manually using the 

To drive the Z80 through T states you use the '1' and '0' menu options, but you have to manually clock some T states when doing that (see the databook). Or you can use the 'r' menu option which will do the reset sequence for you:

trace> r
------------------
Z80 has been reset
------------------
trace> 

The Z80 is now reset and ready to load the first instruction. Press ENTER and this appears:

trace> Bus state:Memory Access
Addr:8000  Data:AB
BUSREQ: 1      (Mega --> Z80)
BUSACK: 1      (Z80  --> Mega)
    M1: 1      (Z80  --> Mega)
  MREQ: 1      (Z80  --> Mega)
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 1      (Z80  --> Mega)
    WR: 1      (Z80  --> Mega)
    RD: 1      (Z80  --> Mega)
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 1      (Mega --> Z80)
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 1      (Z80  --> Mega)

   CLK DEASSERT
Trace Menu
==========
t:Mega drive n tstates       f:Mega drive tstates forever
c:Mega drive tstates, continues to given Z80 instruction address
n:Mega drive tstates until next Z80 instruction
F:Free run (at ~4.5MHz)      M:Mega provide clock (at ~80Hz)
G:Mega take Z80 bus (BUSREQ) R:Mega release Z80 bus
I:Mega take IO map           i:Hardware take IO map
J:Mega take memory map       j:Hardware take memory map
r:reset Z80
1:assert reset               0:deassert reset
b:Breakpoint                 B:Toggle breakpoint
-:Display trace              =:Display II Trace
X:Assert INT x:desaart INT   Y:Assert NMI  y:Deassert NMI
return: drive half a clock   q:quit menu
trace> 

The bus signals are all de-asserted and the Z80 is accessing some meaningless address (0x8000 in this case). The data bus value is also meaningless. The Z80 needs a few clock cycles to get going, so hit enter until you see something other than the clock changing. You will see the clock change state every time enter is pressed. This is the signal that drives the Z80 on the shield and causes it to process instructions. It is 1 to start.

Next press enter, CLK is low:

   CLK: 0      (Mega --> Z80)     - Asserted, means Z80 is in the second half of a T-state
   
then more presses:

   CLK: 1      (Mega --> Z80)

and so on until the Z80 starts an instruction access:

trace> Bus state:Memory Access
PC:0000  Data:00
BUSREQ: 1      (Mega --> Z80)
BUSACK: 1      (Z80  --> Mega)
    M1: 0      (Z80  --> Mega)    - Asserted, means Z80 is doing an opcode fetch cycle
  MREQ: 1      (Z80  --> Mega)
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 1      (Z80  --> Mega)
    WR: 1      (Z80  --> Mega)
    RD: 1      (Z80  --> Mega)
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 1      (Mega --> Z80)
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 0      (Z80  --> Mega)    - Asserted, means Z80 is doing an opcode fetch cycle

    M1 ASSERT
   CLK DEASSERT
    X1 ASSERT
    
M1 is now asserted (the address has been set to 0000 which is the Z80 reset vector). This is the signal that the Z80 is fetching an opcode from memory. Press enter:

PC:0000  Data:31
BUSREQ: 1      (Mega --> Z80)
BUSACK: 1      (Z80  --> Mega)
    M1: 0      (Z80  --> Mega)    - Asserted, means Z80 is doing an opcode fetch cycle
  MREQ: 0      (Z80  --> Mega)    - Asserted, means address bus holds a memory address for a read or write
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 1      (Z80  --> Mega)
    WR: 1      (Z80  --> Mega)
    RD: 0      (Z80  --> Mega)    - Asserted, means the Z80 wants to read data from external device
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 0      (Mega --> Z80)     - Asserted, means Z80 is in the second half of a T-state
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 0      (Z80  --> Mega)    - Asserted, means Z80 is doing an opcode fetch cycle

  MREQ ASSERT
    RD ASSERT
State: Memory Read Access
   CLK ASSERT
   
Now the MREQ and RD lines are asserted. This means that the Z80 is reading data from the memory address space. The data bus now has 0x31 on it. If you look at the example code that is the first instruction opcode. The Z80 starts executing instructions at 0x0000 when reset and you can see it doing that here.
The Mega sketch has recognised that this is a memory access and has allowed the flash chip to put the data at address 0000 on the data bus. That is the data we programmed into the flash earlier. (The Mega can emulate flash if required by changing the sketch, but by default it is set up to let the real flash and memory chips supply data).

Press enter again:

trace> Bus state:Memory Read Access
PC:0000  Data:31
BUSREQ: 1      (Mega --> Z80)
BUSACK: 1      (Z80  --> Mega)
    M1: 0      (Z80  --> Mega)    - Asserted, means Z80 is doing an opcode fetch cycle
  MREQ: 0      (Z80  --> Mega)    - Asserted, means address bus holds a memory address for a read or write
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 1      (Z80  --> Mega)
    WR: 1      (Z80  --> Mega)
    RD: 0      (Z80  --> Mega)    - Asserted, means the Z80 wants to read data from external device
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 1      (Mega --> Z80)
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 0      (Z80  --> Mega)    - Asserted, means Z80 is doing an opcode fetch cycle

   CLK DEASSERT
   
   Just the clock has changed as the Z80 is allowing time for the data on the bus to be set up. At the speed we are running at this isn't necessary, but at full speed it could be.
Press enter again:
PC:0000  Data:31
BUSREQ: 1      (Mega --> Z80)
BUSACK: 1      (Z80  --> Mega)
    M1: 0      (Z80  --> Mega)    - Asserted, means Z80 is doing an opcode fetch cycle
  MREQ: 0      (Z80  --> Mega)    - Asserted, means address bus holds a memory address for a read or write
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 1      (Z80  --> Mega)
    WR: 1      (Z80  --> Mega)
    RD: 0      (Z80  --> Mega)    - Asserted, means the Z80 wants to read data from external device
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 0      (Mega --> Z80)     - Asserted, means Z80 is in the second half of a T-state
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 0      (Z80  --> Mega)    - Asserted, means Z80 is doing an opcode fetch cycle

   CLK ASSERT
   
Still just a clock change. Enter again:
ddr:0000  Data:31
BUSREQ: 1      (Mega --> Z80)
BUSACK: 1      (Z80  --> Mega)
    M1: 1      (Z80  --> Mega)
  MREQ: 1      (Z80  --> Mega)
 IOREQ: 1      (Z80  --> Mega)
  RFSH: 0      (Z80  --> Mega)    - Asserted, means Z80 is in refresh state
    WR: 1      (Z80  --> Mega)
    RD: 1      (Z80  --> Mega)
   NMI: 1      (Mega --> Z80)
   INT: 1      (Mega --> Z80)
  WAIT: 1      (Mega --> Z80)
   CLK: 1      (Mega --> Z80)
   RES: 1      (Mega --> Z80)
MAPRQM: 0      (Mega --> Shield)  - Mega is not providing memory (Flash and RAM) contents (real hardware is mapped)
MAPRQI: 0      (Mega --> Shield)  - Mega is not providing IO (GPIO, CTC) contents (real hardware is mapped)
    X1: 1      (Z80  --> Mega)

    M1 DEASSERT
  MREQ DEASSERT
State: Memory Read Access END
  RFSH ASSERT
    RD DEASSERT
State: Idle
   CLK DEASSERT
    X1 DEASSERT

Now the data on the bus has been clocked into the Z80. RD and MREQ are de-asserted but RFSH has been asserted. The Z80 performs a memory refressh cycle after every opcode fetch, which is designed for dynamic RAM refresh. We have static RAM so don't need this, but we have to step past it to get to the next part of the instruction fetch. 

Press enter a few times until the RFSH line isn't asserted any more.
