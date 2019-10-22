#!/usr/bin/wish
#
# Terminal emulator for Z80 shield
#
#

set device   [lindex $argv 0]

puts "Opening $device"
set f [open $device r+]

fconfigure $f -blocking 0
fconfigure $f -mode 115200,n,8,1
fconfigure $f -handshake none -buffering none -translation binary
fileevent $f readable "read_data $f"
#fileevent $f writable "write_data %A"

proc write_data {txt} {
    global f
    puts "W:$txt"
    puts -nonewline $f $txt
    flush $f
}

# Any data received we display, unless it is from a data channel
# data channels allow hidden data to be sent to the terminal, these are displayed in
# separate windows

proc read_data {f} {
    set tx [read $f]
    puts "R:"

    # We process some characters differently
    set chars [split $tx ""]
    
    foreach char $chars {
	switch $char {
	    "\n" {
	    }
	    
	    "\r" {
		puts "nl"
		.t.text insert end "\n"
	    }
	    default {
		puts -nonewline $char
		.t.text insert end $char
		.t.text yview end
	    }
	}
    }
}

####################################################################################################
#
# Open window for a data channel
#

proc open_data_channel_window {w} {
     frame $w
     eval {text $w.text \
         -xscrollcommand [list $w.xscroll set] \
         -yscrollcommand [list $w.yscroll set]} -width 80 -height 25
     scrollbar $w.xscroll -orient horizontal \
         -command [list $w.text xview]
     scrollbar $w.yscroll -orient vertical \
         -command [list $w.text yview]
     grid $w.text $w.yscroll -sticky news
     grid $w.xscroll -sticky news
     grid rowconfigure    $w 0 -weight 1
     grid columnconfigure $w 0 -weight 1

    return $w.text
}

####################################################################################################
#
# Send .IHX file to mega
#

proc send_ihx_file {filename device} {
    
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
}

####################################################################################################
#
# Sends an IHX file to the Mega
# Requires upload menu to be selected before executing 
#

proc send_ihx_file_dialog {} {
    global device
    
    set types {
	{{IHX Files} {.ihx}}
    }
    
    set filename [tk_getOpenFile -initialdir ../z80_c -filetypes $types ]

    if { $filename != "" } {
	send_ihx_file $filename $device
    }
}


####################################################################################################
#
# Open a window for terminal interaction
#

proc open_terminal_window {w} {
    frame $w
    eval {text $w.text \
	      -xscrollcommand [list $w.xscroll set] \
	      -yscrollcommand [list $w.yscroll set]} -width 80 -height 25
    scrollbar $w.xscroll -orient horizontal \
	-command [list $w.text xview]
    scrollbar $w.yscroll -orient vertical \
	-command [list $w.text yview]
    grid $w.text $w.yscroll -sticky news
    grid $w.xscroll -sticky news
    grid rowconfigure    $w 0 -weight 1
    grid columnconfigure $w 0 -weight 1
    
    bind $w.text <Any-Key> "write_data %A"
    
    
    #Main menu
    menu $w.menu -tearoff 0
    
    $w.menu add cascade -label "File"  -menu $w.menu.file -underline 0
    $w.menu add cascade -label "Z80" -menu $w.menu.z80 -underline 0
    $w.menu add cascade -label "About" -menu $w.menu.about -underline 0
    
    menu $w.menu.file -tearoff 0
    menu $w.menu.z80 -tearoff 0
    menu $w.menu.about -tearoff 0
    
    set m $w.menu.file
    $m add command -label "Send IHX File" -command {send_ihx_file_dialog}
    $m add command -label "Exit" -command exit
    
    set m $w.menu.z80
    $m add command -label "Registers" -command {display_registers}
    
    set m $w.menu.about
    $m add command -label "Z80 Shield Terminal Emulator"
    $m add command -label "Version 1.0"
    
    . configure -menu $w.menu
    
    return $w.text
}

# fconfigure $chan -mode $Term(Mode) -translation binary -buffering none -blocking 0


open_terminal_window .t
pack .t -side top -fill both -expand true

open_data_channel_window .regs
pack .t -side top -fill both -expand true

# Drop into event loop...
write_data "\n"


