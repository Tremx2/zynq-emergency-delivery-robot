# data/ — 测试数据与参考结果

| 路径 | 内容 | 采集时间 |
|---|---|---|
| `sensors/sensor_data_live_20261102_191216.json` + `realtime_log_…txt` | 实时回传：IMU 100 Hz、ToF 5 Hz，持续 8 s | 2026-11-02 |
| `sensors/sensor_data_live_20261103_202412.json` + `realtime_log_…txt` | `collect_all` 全量采集：I2C 扫描（0x29/0x40/0x69），IMU 200 帧，ToF 50/50 次有效（56.3±1.6 mm），电池 11.656 V | 2026-11-03 |
| `imu_calib/` | IMU 六面法标定：6 个面的原始采样 CSV（每面 1000 组）、每面统计、解算结果、报告，以及最终的 `imu_calib.json`。说明见该目录下的 README.md | 2026-11-04 |
| `navigation_sim/` | 导航仿真参考运行（去 A 点，D* Lite，风险权重 8，含看门狗演示）：50 ms 遥测 CSV 和汇总 JSON | 2026-10 |

> 板子的系统时钟不准，部分文件名和 JSON 里的时间戳显示为 2025-05-05，实际采集日期以上表为准。
