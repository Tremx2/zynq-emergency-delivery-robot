# board/ — 上板工程

| 路径 | 内容 |
|---|---|
| `overlay/robot_system_wrapper.bit`<br>`overlay/robot_system_wrapper.hwh` | 当前板上运行的 overlay，md5 为 `754a2106…` / `e907837f…`，两个文件必须同名 |
| `tests/test_1_forward.py` … `test_5_rotate.py` | 底盘五个动作测试：前进、后退、左弧、右弧、原地旋转（`right` 参数表示右转）。不限时，按 Ctrl+C 停车 |
| `tests/chassis_motion_demo.ipynb` | 六动作演示，50 Hz 记录四轮速度、PWM、位置和 IMU，输出 2 张表、6 张图，并保存 CSV/JSON/PNG |
| `tests/watchdog_timeout_test.ipynb` | 看门狗超时断电测试，包括预检查（停车互锁）以及 A 停止心跳、B kill -9、C 错误魔数三种场景。用软件时间戳测量断电时刻 |
| `systemd/pynq-motor-disable.service` | 开机自启服务：加载 overlay，并把四路电机主动关断（防止上电自转） |
| `host_tools/deploy.py` | PC 端一键部署：上传 overlay、`src/ps` 和 `tests`，逐文件做 md5 校验并 sync |
| `host_tools/pynq_ssh.py` | SSH 小工具（基于 paramiko）：`run "<cmd>"` 执行命令，`put <local> <remote>` 上传文件 |
| `host_tools/pynq_get.py` | 从板上打包下载文件或目录，逐文件做 md5 校验 |
| `host_tools/build_*_nb.py` | 两个 notebook 的生成脚本，修改 notebook 时改这里再重新生成 |
| `outputs/` | 上板实测输出的存放位置：notebook 结果目录可以用 `pynq_get.py` 拉回这里 |

## 运行要点
- 板上运行目录是 `/home/xilinx/jupyter_notebooks/`。用到 overlay 的程序需要 root 权限，并设置 `XILINX_XRT=/usr`。Jupyter 内核本身以 root 运行。
- 用 `sudo` 以外的方式执行 MMIO 会因为权限不足而失败。
- 运行电机测试前，请关闭其他正在使用底盘的 notebook 内核，否则它们的后台喂狗线程会干扰看门狗测试。
- 拉回实测结果的示例：
  ```bash
  python board/host_tools/pynq_get.py board/outputs /home/xilinx/jupyter_notebooks watchdog_test_results
  ```
