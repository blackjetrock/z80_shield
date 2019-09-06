#!/usr/bin/tclsh
#
# Upload binary file to serial port
#
#

set filename [lindex $argv 0]
set device   [lindex $argv 1]

# Read the hex file

set f [open $filename]
set txt [read $f]
close $f

# Open the hex file and send it to the sketch
set f [open $device r+]
#fconfigure $f -mode 9600,n,8,1
fconfigure $f -handshake none
fconfigure $f -blocking 0

if { 0 } {
    while { 1 } {
	puts $f "S\n"
	set d [read $f]
	if { [string length $d] > 0 } {
	    puts $d
	}
    }
}

# Flush
while { [string length [read $f]] > 0 } {
}


# Signal that we have started
puts "Starting"

puts $f "S"
puts $f "S"

puts "Sending $filename"

foreach line [split $txt "\n"] {

    # We terminate line with a '-'
    puts "Sending line '$line'"
    puts $f "$line-"
    
    # Wait for a '+' that signals the next line can be sent
    set rx_tick 0
    puts "Waiting for tick"
    
    while {!$rx_tick} {
	
	set rdata [read $f]

	if { [string length $rdata] > 0 } {
	    puts "Got '$rdata'"
	    switch $rdata {
		"+" {
		    # Tick received, move on
		    set rx_tick 1
		}
	    }
	}
    }
}

