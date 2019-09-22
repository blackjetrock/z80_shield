#!/usr/bin/tclsh
#
#
# Converts a Z80 lst file to a arduino array that can be pasted into the
# shield sketch
#
# e.g.
# Create a LIS file from an ASM source file with:
#
#  z80asm --list example_code_df_test.asm
#
# Then run this script to convert the contents of that to a C array called
# example_code_df_test[] in z80_shield.ino
#
#  ./z80_convert.tcl example_code_df_test.lis example_code_df_test z80_shield.ino
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

    # Pick out any valid line. AM and DF appear to use different versions of z80asm so there's
    # a selection of regexes to find the lines in whatever format they happen to be
    #
    unset -nocomplain am_all addr data byte asm
    unset -nocomplain label_all line_num addr label tail
    unset -nocomplain df_all line_num addr data byte asm
    
    if { [regexp -- {([0-9A-Fa-f]{4})(( [0-9a-f]{2})+)(.*)}               $line am_all    addr data byte asm]          ||
         [regexp -- {^(\d+ +)?([0-9A-Fa-f]{4}) +(.*\:)(.*)}               $line label_all line_num addr label tail]    ||
	 [regexp -- {^(\d+ +)?([0-9A-Fa-f]{4}) +(( [0-9A-Fa-f]{2})+)(.*)} $line df_all    line_num addr data byte asm] } {

	if { [info exists label_all] } {
	    append array_code "[string repeat " " 40] //   $addr : $label $tail\n"
	} else {
	    #puts "'$data'"
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

