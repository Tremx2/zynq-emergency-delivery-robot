#!/usr/bin/env bash
# 一键重建 Vivado 工程并生成比特流（Linux）。需要 Vivado 2022.2 已 source settings64.sh。
set -euo pipefail
cd "$(dirname "$0")/.."
vivado -mode batch -nojournal -log build/create_project.log -source build/create_project.tcl
vivado -mode batch -nojournal -log build/build_bitstream.log -source build/build_bitstream.tcl
echo "输出: build/output/robot_system_wrapper.bit / .hwh"
