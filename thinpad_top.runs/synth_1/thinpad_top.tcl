# 
# Synthesis run script generated by Vivado
# 

set TIME_start [clock seconds] 
proc create_report { reportName command } {
  set status "."
  append status $reportName ".fail"
  if { [file exists $status] } {
    eval file delete [glob $status]
  }
  send_msg_id runtcl-4 info "Executing : $command"
  set retval [eval catch { $command } msg]
  if { $retval != 0 } {
    set fp [open $status w]
    close $fp
    send_msg_id runtcl-5 warning "$msg"
  }
}
create_project -in_memory -part xc7a100tfgg676-2L

set_param project.singleFileAddWarning.threshold 0
set_param project.compositeFile.enableAutoGeneration 0
set_param synth.vivado.isSynthRun true
set_property webtalk.parent_dir F:/cod/CPU/thinpad_top.cache/wt [current_project]
set_property parent.project_path F:/cod/CPU/thinpad_top.xpr [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property target_language Verilog [current_project]
set_property ip_output_repo f:/cod/CPU/thinpad_top.cache/ip [current_project]
set_property ip_cache_permissions {read write} [current_project]
read_verilog -library xil_defaultlib {
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/ALU.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/ALUControl.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/Control.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/EX_MEM.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/ID_EX.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/IF_ID.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/MEM_WB.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/RegisterFile.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/SEG7_LUT.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/nram.v
  F:/cod/CPU/thinpad_top.srcs/sources_1/new/thinpad_top.v
}
# Mark all dcp files as not used in implementation to prevent them from being
# stitched into the results of this synthesis run. Any black boxes in the
# design are intentionally left as such for best results. Dcp files will be
# stitched into the design at a later time, either when this synthesis run is
# opened, or when it is stitched into a dependent implementation run.
foreach dcp [get_files -quiet -all -filter file_type=="Design\ Checkpoint"] {
  set_property used_in_implementation false $dcp
}
read_xdc F:/cod/CPU/thinpad_top.srcs/constrs_1/new/thinpad_top.xdc
set_property used_in_implementation false [get_files F:/cod/CPU/thinpad_top.srcs/constrs_1/new/thinpad_top.xdc]

set_param ips.enableIPCacheLiteLoad 1
close [open __synthesis_is_running__ w]

synth_design -top thinpad_top -part xc7a100tfgg676-2L


# disable binary constraint mode for synth run checkpoints
set_param constraints.enableBinaryConstraints false
write_checkpoint -force -noxdef thinpad_top.dcp
create_report "synth_1_synth_report_utilization_0" "report_utilization -file thinpad_top_utilization_synth.rpt -pb thinpad_top_utilization_synth.pb"
file delete __synthesis_is_running__
close [open __synthesis_is_complete__ w]
