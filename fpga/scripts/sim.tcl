if { $argc < 1 } {
    puts "ERROR: No testbench top module specified."
    exit 1
}

set tb_top [lindex $argv 0]

open_project ./project/vivado_sim.xpr

set_property top $tb_top [get_filesets sim_1]
update_compile_order -fileset sim_1

launch_simulation -mode behavioral

exit
