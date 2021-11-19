# setup name of the clock in your design.
set clkname clk

# set variable "modname" to the name of topmost module in design
set modname MyDesign

# set variable "RTL_DIR" to the HDL directory w.r.t synthesis directory
set RTL_DIR    ../v

# set variable "type" to a name that distinguishes this synthesis run
set type tut1

#set the number of digits to be used for delay results
set report_default_significant_digits 4

set CLK_PER 10
#------------------------------------------------------------
#
# Basic Synthesis Script (TCL format)
#
# Revision History
#   1/15/03  : Author Shane T. Gehring - from class example
#   2/09/07  : Author Zhengtao Yu      - from class example
#   12/14/07 : Author Ravi Jenkal      - updated to 180 nm & tcl
#   10/7/20  : P Franzon - Project specific script
#
#------------------------------------------------------------
