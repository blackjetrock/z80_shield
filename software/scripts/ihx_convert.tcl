#!/usr/bin/tclsh
#
#
# Converts an ihx file to a arduino array that can be pasted into the
# shield sketch
#
# e.g.
# Create a IHX file from an ASM source file with:
#
# mii
#
# Then run this script to convert the contents of that to a C array called
# inter_inst_code_regdump[] in z80_shield.ino
#
#  ./z80_convert.tcl ii.ihx inter_inst_code_regdump z80_shield.ino
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

    # Pull hex out of any valid line
    if { [regexp -- {:([0-9a-fA-F]{2})([0-9a-fA-F]+)} $line all length hexbytes] } {
	puts "Length: $length"
	puts "Hex: $hexbytes"
	if { [expr 0x$length] == 0 } {
	    continue
	} else {
	    # Put bytes in array
	    # Drop address type and checksum
	    set hexbytes [string range $hexbytes 6 end-2]
	    
	    while { [string length $hexbytes] > 0 } {
		set byte [string range $hexbytes 0 1]
		append array_code "    0x$byte,\n"
		set hexbytes [string range $hexbytes 2 end]
	    }
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

