# Gowin FPGA Build Script for Tang Nano 9K

set_device GW1NR-LV9QN88PC6/I5

# Add source files
add_file top.vhd

# Set top module
set_option -top_module top

# Add constraints
add_file tang_nano_9k.cst

# Set synthesis options
set_option -vhdl_std vhd2008
set_option -output_base_name fpga_out

# Run synthesis
run all