#!/usr/bin/tclsh
#
#
# Converts a Z80 lst file to a arduino array that can be pasted into the
# shield sketch
#
#

# Filename to convert
set fn [lindex $argv 0]

# Array to replace in code
set replace [lindex $argv 1]

# File to embed in
set embed_fn [lindex $argv 2]

# Convert the file to C array structure

set f [open $fn]
set txt [read $f]
close $f

set array_code ""

append array_code "BYTE $replace\[\] =\n"
append array_code "  \{\n"

foreach line [split $txt "\n"] {

    # Ignore comment lines
    if { [regexp -- {([0-9A-Fa-f]{4})(( [0-9a-f]{2})+)(.*)} $line all addr data byte asm] } {
       puts "'$data'"
	if { [expr 0x$addr < 0x8000] } {
	    set data [string trim $data]
	    set cdata ""
	    foreach byte [split $data " "] {
		set cdata "$cdata 0x$byte, "
	    }
	    
	    append array_code "[format "%40s" $cdata] //   $addr : $asm\n"
	}
    }
}

append array_code "  \};\n"

# Now find the replace array in the replaced file and replace the code
# in the file with the new code

set f [open $embed_fn]
set rt [read $f]
close $f


set start [string first "BYTE $replace\[\] =" $rt]
set end   [expr [string first "\};" $rt $start]+1]

puts "S:$start e:$end"

puts [string range $rt $start $end]

set newrt [string replace $rt $start $end $array_code]

if {1} {
    set f [open $embed_fn w]
    puts $f $newrt
    close $f
    
    set f [open convert_array_text.txt w]
    puts $f $array_code
    close $f
    
}

