#!/usr/bin/wish
#
# Terminal emulator for Z80 shield
#
#

set ::REGISTER_LIST {PC SP AF BC DE HL AFt BCt DEt HLt IX IY}

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

set ::DC_START 0
set ::DC_NUM   0
set ::DC_DATA  0
set ::DC_STREAM ""

proc read_data {f} {
    set tx [read $f]
    puts "R:"

    # We process some characters differently
    set chars [split $tx ""]
    
    foreach char $chars {
	if { $::DC_START } {
	    # Next character is data channel number
	    set ::DC_NUM $char
	    set ::DC_START 0
	    set ::DC_DATA 1
	    set ::DC_STREAM ""
	    continue
	}

	
	switch $char {
	    "^" {
		# data channel start marker, this can be changed to chars with top bit set later
		set ::DC_START 1
		continue
	    }
	    "$" {
		# End of data channel
		set ::DC_DATA 0
		data$::DC_NUM $::DC_STREAM		
		continue
	    }
	    
	    "\n" {
	    }
	    
	    "\r" {
		puts "nl"
		.t.text insert end "\n"
	    }
	    default {
		if { $::DC_DATA } {
		    # Characters have to go to data channel stream
		    set ::DC_STREAM "$::DC_STREAM$char"
		} else {
		    puts -nonewline $char
		    .t.text insert end $char
		    .t.text yview end
		}
	    }
	}
    }
}

####################################################################################################
#
# Open window for a data channel
#

toplevel .data

proc open_data_channel_window {w} {
    
    frame $w
    text $w.text -xscrollcommand [list $w.xscroll set] -yscrollcommand [list $w.yscroll set] -width 80 -height 25
    scrollbar $w.xscroll -orient horizontal -command [list $w.text xview]
    scrollbar $w.yscroll -orient vertical -command [list $w.text yview]
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

proc send_ihx_file {filename f} {
    
    # Read the hex file
    
    set g [open $filename]
    set txt [read $g]
    close $g
    
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
    global f
    
    set types {
	{{IHX Files} {.ihx}}
    }
    
    set filename [tk_getOpenFile -initialdir ../z80_c -filetypes $types]

    if { $filename != "" } {
	send_ihx_file $filename $f
    }
}

####################################################################################################
#
# Data channel window handlers
#
#

# Register value window

proc data0 {data} {
    puts "**data 0 : '$data' **"
    set w .data.0.text
    
    set i 1
    foreach {register} $::REGISTER_LIST {
	if { [regexp -- "$register:...." $data] } {
	    $w delete $i.0 $i.end
	    $w insert $i.0 "$data"
	}

	incr i 1
    }
    
    if {0} {
	switch -regexp $data {
	    "PC:...." {
		$w delete 1.0 1.end
		$w insert 1.0 "$data"
	    }
	    "AF:...." {
		$w delete 2.0 2.end
		$w insert 2.0 "$data"
	    }
	    "BC:...." {
		$w delete 3.0 3.end
		$w insert 3.0 "$data"
	    }
	    "DE:...." {
		$w delete 4.0 4.end
		$w insert 4.0 "$data"
	    }
	    "HL:...." {
		$w delete 5.0 5.end
		$w insert 5.0 "$data"
	    }
	    "IX:...." {
		$w delete 6.0 6.end
		$w insert 6.0 "$data"
	    }
	    "IY:...." {
		$w delete 7.0 7.end
		$w insert 7.0 "$data"
	    }
	    "SP:...." {
		$w delete 8.0 8.end
		$w insert 8.0 "$data"
	    }
	    
	    "AFt:...." {
		$w delete 10.0 10.end
		$w insert 10.0 "$data"
	    }
	    "BCt:...." {
		$w delete 11.0 11.end
		$w insert 11.0 "$data"
	    }
	    "DEt:...." {
		$w delete 12.0 12.end
		$w insert 12.0 "$data"
	    }
	    "HLt:...." {
		$w delete 13.0 13.end
		$w insert 13.0 "$data"
	    }
	    "IXt:...." {
		$w delete 14.0 14.end
		$w insert 14.0 "$data"
	    }
	    "IYt:...." {
		$w delete 15.0 15.end
		$w insert 15.0 "$data"
	    }
	}
	
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
	      -yscrollcommand [list $w.yscroll set]} -width 135 -height 40
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

open_terminal_window .t
pack .t -side top -fill both -expand true

# Open register value display window
open_data_channel_window .data.0
pack .data.0 -side top -fill both -expand true

foreach {register} $::REGISTER_LIST {
    .data.0.text  insert end "$register:....\n"
}


# Drop into event loop...
write_data "\n"


