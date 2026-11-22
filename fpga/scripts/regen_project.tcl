set projectName "vivado_sim"
set projectDir "./project"
set targetPart "xc7s50csga324-1"

if {[file exists $projectDir]} {
    file delete -force $projectDir
}

create_project $projectName $projectDir -part $targetPart

set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]

if {[file exists "./src/hdl"]} {
    add_files -fileset sources_1 -norecurse [glob "./src/hdl/*.sv"]
}

if {[file exists "./src/constraints"]} {
    add_files -fileset constrs_1 -norecurse [glob "./src/constraints/*.xdc"]
}

if {[file exists "./sim/testbench"]} {
    add_files -fileset sim_1 -norecurse [glob "./sim/testbench/*.sv"]   
}

if {[file exists "./src/ip"]} {
    set ipFiles [glob -nocomplain "./src/ip/*/*.xci"]
    if {[llength $ipFiles] > 0} {
        import_files -fileset sources_1 $ipFiles
        
        upgrade_ip [get_ips *] -quiet
        generate_target all [get_ips *]
    }
}

if {[file exists "./src/bd/system_bd.tcl"]} {
    source "./src/bd/system_bd.tcl"
    
    update_compile_order -fileset sources_1
}

update_compile_order -fileset sources_1

puts "========================================================"
puts " Project $projectName regenerated successfully!"
puts "========================================================"
