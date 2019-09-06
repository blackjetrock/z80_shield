#!/usr/bin/tclsh
#
# Upload binary file to serial port
#
#

set filename [lindex $argv 0]
set device   [lindex $argv 1]

set f [open $filename]
set txt [read $f]
close $f

set f [open 
foreach line [split $txt "\n"] {
    
}
