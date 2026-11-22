if { $argc < 1 } {
    puts "ERROR: No testbench top module specified."
    exit 1
}

set tb_top [lindex $argv 0]

open_project ./project/vivado_sim.xpr

set_property top $tb_top [get_filesets sim_1]
update_compile_order -fileset sim_1

launch_simulation -mode behavioral

if { [string length [current_wave_config]] == 0 } {
    puts "No active wave configuration found."
    create_wave_config
}

if { [llength [get_objects]] > 0} {
    add_wave /
    set_property needs_save false [current_wave_config]
}

restart
run -all
