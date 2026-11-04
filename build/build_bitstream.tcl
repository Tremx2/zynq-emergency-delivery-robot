# =============================================================================
# build_bitstream.tcl —— 综合 → 实现 → 比特流，并导出报告与 PYNQ overlay 文件
#
# 用法（先运行 create_project.tcl）:
#   vivado -mode batch -source build/build_bitstream.tcl
#
# 输出:
#   build/output/robot_system_wrapper.bit   比特流
#   build/output/robot_system_wrapper.hwh   PYNQ 硬件描述（与 .bit 同名，Overlay 加载需要）
#   build/output/reports/*.rpt              资源 / 时序 / 功耗 / DRC 报告
# 与仓库中参考结果对比: board/overlay/、build/reports/
# =============================================================================
set root     [file normalize [file join [file dirname [info script]] ..]]
set proj_dir [file join $root build vivado]
set out_dir  [file join $root build output]
set jobs     4

open_project [file join $proj_dir robot_system.xpr]

reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "impl_1 未完成: [get_property STATUS [get_runs impl_1]]"
}

file mkdir [file join $out_dir reports]
open_run impl_1
report_utilization        -file [file join $out_dir reports utilization_routed.rpt]
report_timing_summary     -max_paths 10 -file [file join $out_dir reports timing_summary_routed.rpt]
report_power              -file [file join $out_dir reports power_routed.rpt]
report_drc                -file [file join $out_dir reports drc_routed.rpt]
report_route_status       -file [file join $out_dir reports route_status.rpt]

set impl_dir [get_property DIRECTORY [get_runs impl_1]]
file copy -force [file join $impl_dir robot_system_wrapper.bit] [file join $out_dir robot_system_wrapper.bit]
set hwh [lindex [glob -nocomplain [file join $proj_dir robot_system.gen sources_1 bd robot_system hw_handoff robot_system.hwh]] 0]
if {$hwh eq ""} { error "找不到 robot_system.hwh" }
file copy -force $hwh [file join $out_dir robot_system_wrapper.hwh]

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "=== BUILD_DONE  WNS = $wns ns  ->  $out_dir ==="
