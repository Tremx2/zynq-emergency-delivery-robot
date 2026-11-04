# build/ — 可复现构建

| 文件 | 作用 |
|---|---|
| `create_project.tcl` | 用 `src/` 新建 Vivado 工程（`build/vivado/`），包括 IP 仓库、BD、wrapper 和约束 |
| `build_bitstream.tcl` | 依次跑综合、实现、比特流，把 .bit、.hwh 和报告导出到 `build/output/` |
| `build.bat` / `build.sh` | 一键执行以上两步（Windows / Linux） |
| `reports/` | **当前板上比特流**的综合与实现报告（2026-11-02，Vivado 2022.2.2），作为参考结果 |

```bat
build\build.bat
```
默认使用 `D:\apps\Vivado\2022.2\bin\vivado.bat`。如果 Vivado 装在别处，先执行：`set VIVADO_BAT=<你的路径>\vivado.bat`。

参考结果来自 `reports/` 下的报告：

| 指标 | 数值 | 来源 |
|---|---|---|
| WNS / WHS | +0.850 ns / +0.035 ns，0 失败端点 | `timing_summary_routed.rpt` |
| LUT / FF | 3358（6.31%）/ 3145（2.96%） | `utilization_placed.rpt` |
| DSP / BRAM | 0 / 0 | `utilization_placed.rpt` |
| IOB | 19（15.20%） | `utilization_placed.rpt` |
| 片上功耗 / 结温 | 1.681 W / 44.4 ℃ | `power_routed.rpt` |

`vivado/` 和 `output/` 是生成目录，不纳入版本控制。


