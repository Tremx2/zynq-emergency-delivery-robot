# IMU 六面法静态标定数据（ICM-42688-P）

- 实际日期：**2026-11-04**。板子的系统时钟不准，会话目录名和 JSON 里的 `2025-05-05` 是板子时间。
- 采集脚本：`src/ps/imu_six_face_calib.py`，在板上 `/home/xilinx/jupyter_notebooks/` 目录运行。
- 下载工具：`board/host_tools/pynq_get.py`。数据从板上拷回电脑，18 个文件逐个做过 md5 校验，全部一致。
- 第一次 rear 采集的数据原为 `.bak` 后缀，已改名为 `face_rear_try1*`，内容不变。
- 板上原件：`/home/xilinx/jupyter_notebooks/imu_calib_data/session_20250505_073256/`

## 采集
- 每面采 1000 组，100 Hz，0 失败；温度 18.8~19.6 °C。
- 六面主重力轴：flat Z-，flipped Z+，left X-，right X+，front Y-，rear Y+。IMU 是倒装的，所以 flat 面的主轴为 Z-。
- flipped / left / right 三面静止检查未通过，原因是轻微晃动。按用户要求保留这三面。
- rear 第一次采集时车体歪了 7.9° 且在晃动，已重采（重采后歪 4.1°，静止检查通过）。第一次的数据保留为 `face_rear_try1_*.bak`。

## 结果（主结果为方法 A：经典六面法）
| | X | Y | Z |
|---|---|---|---|
| 加计零偏 (mg) | -0.334 | -7.187 | -6.442 |
| 加计标度 | 0.99752 | 1.00041 | 0.99974 |
| 陀螺零偏 (dps，6 面均值) | +0.1922 | +1.0761 | -0.6906 |
| 陀螺零偏 6 面极差 (dps) | 0.0175 | 0.0797 | 0.0246 |

- 各面 |a| 与 1 g 的偏差（RMS）：标定前 5.03 mg，标定后 **1.07 mg**。
- 方法 B（12 参数最小二乘）的 RMS 为 5.77 mg，比方法 A 差。原因是只有 6 个面、每面都是手工摆放，带 1~4° 的倾斜，最小二乘把这些倾斜误当成了轴间耦合。方法 B 的结果仅作参考保留。
- 标定模型：`a_cal[i] = (a_raw[i] - bias[i]) / scale[i]`

## 文件
- `imu_calib.json`：最终标定文件，板上程序读取的就是它。其中的 accel 是本次结果，gyro 和 mounting 保留了之前的水平标定。
- `imu_calib_data/session_.../face_<面>_raw.csv`：每面的全部原始采样。
- `face_<面>.json`：每面的统计量。
- `calib_result.json`：两种方法的完整解算结果。
- `report.txt`：可读报告。
- `imu_calib_backup.json`：解算前的 imu_calib.json 备份。
