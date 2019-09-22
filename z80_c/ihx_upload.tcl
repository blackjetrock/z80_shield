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
puts "Opening $device"
set f [open $device r+]
#set f [open /dev/tty  r+]
#fconfigure $f -mode 9600,n,8,1
#fconfigure $f -handshake none
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
puts "Flushing..."
set done 0
while { !$done } {
    set tx [read $f]
    if { [string length $tx] == 0 } {
	set done 1
    } else {
	puts $tx
    }
}


# Signal that we have started
puts "Starting"

puts $f "S"
flush $f

after 2000

# Flush
puts "Flushing..."
set done 0
while { !$done } {
    set tx  [read $f]
    if { [string length $tx ] == 0 } {
	set done 1
    } else {
	puts "Flushed '$tx '"
    }
    
    
}

puts "Sending $filename"

foreach line [split $txt "\n"] {
    if { [string length $line] == 0 } {
	break
    }
    
    # We terminate line with a '-'
    puts "Sending line '$line'"
    puts $f "$line-"
    flush $f
    
    # Wait for a '+' that signals the next line can be sent
    set rx_tick 0
    puts "Waiting for tick"
    
    while {!$rx_tick} {
	
	set rdata [read $f]

	if { [string length $rdata] > 0 } {
	    if { [string first "+" $rdata] != -1 } {
		puts "Got '$rdata'"
		# Tick received, move on
		set rx_tick 1
		
	    }
	}
    }
}


