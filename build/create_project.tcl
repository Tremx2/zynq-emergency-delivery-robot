# =============================================================================
# create_project.tcl —— 从仓库源码重建 Vivado 工程（Vivado 2022.2）
#
# 用法（仓库根目录下）:
#   vivado -mode batch -source build/create_project.tcl
#
# 输入:  src/ip_repo/robot_motor_axi_1_0   自定义 IP（v1.3，含看门狗与 tof_xshut）
#        src/bd/robot_system.bd             Block Design（PS7 + AXI IIC + robot_motor_axi）
#        src/constraints/robot_system.xdc   引脚约束
# 输出:  build/vivado/robot_system.xpr      （生成目录，已在 .gitignore 中忽略）
# =============================================================================
set root     [file normalize [file join [file dirname [info script]] ..]]
set proj_dir [file join $root build vivado]
set part     xc7z020clg400-1

create_project robot_system $proj_dir -part $part -force
set_property target_language Verilog [current_project]
set_property default_lib xil_defaultlib [current_project]

# 自定义 IP 仓库
set_property ip_repo_paths [list [file join $root src ip_repo]] [current_project]
update_ip_catalog -rebuild

# Block Design：复制进工程目录，避免 Vivado 改写仓库里的源文件
import_files -norecurse [file join $root src bd robot_system.bd]
set bd_file [get_files robot_system.bd]
open_bd_design $bd_file
validate_bd_design
save_bd_design
close_bd_design [current_bd_design]
generate_target all $bd_file

# HDL wrapper 作为顶层
set wrapper [make_wrapper -files $bd_file -top]
add_files -norecurse $wrapper
set_property top robot_system_wrapper [current_fileset]

# 约束
add_files -fileset constrs_1 -norecurse [file join $root src constraints robot_system.xdc]

update_compile_order -fileset sources_1
puts "=== CREATE_PROJECT_DONE: [file join $proj_dir robot_system.xpr] ==="
