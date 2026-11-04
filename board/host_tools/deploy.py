"""把 overlay、PS 侧程序和测试脚本部署到 PYNQ 板（逐文件 md5 校验，最后 sync）。

用法（仓库根目录下，PC 与板子同网段）:
  python board/host_tools/deploy.py              # 部署 overlay + src/ps + board/tests
  python board/host_tools/deploy.py --no-overlay # 只更新 Python 程序
  python board/host_tools/deploy.py --service    # 另外安装开机关断电机的 systemd 服务（需要 sudo 密码 xilinx）

板端目录: /home/xilinx/jupyter_notebooks/（PYNQ 默认 Jupyter 工作目录）
"""
import argparse
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from pynq_ssh import connect, put  # noqa: E402

REMOTE = "/home/xilinx/jupyter_notebooks"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-overlay", action="store_true")
    ap.add_argument("--service", action="store_true")
    a = ap.parse_args()

    files = []
    if not a.no_overlay:
        files += sorted(glob.glob(os.path.join(ROOT, "board", "overlay", "robot_system_wrapper.*")))
    files += sorted(glob.glob(os.path.join(ROOT, "src", "ps", "*.py")))
    files += sorted(glob.glob(os.path.join(ROOT, "board", "tests", "*.py")))
    files += sorted(glob.glob(os.path.join(ROOT, "board", "tests", "*.ipynb")))

    c = connect()
    bad = 0
    for f in files:
        bad += put(c, f, f"{REMOTE}/{os.path.basename(f)}")
    if a.service:
        svc = os.path.join(ROOT, "board", "systemd", "pynq-motor-disable.service")
        bad += put(c, svc, "/tmp/pynq-motor-disable.service")
        cmd = ("echo xilinx | sudo -S sh -c 'cp /tmp/pynq-motor-disable.service /etc/systemd/system/ && "
               "systemctl daemon-reload && systemctl enable pynq-motor-disable.service'")
        _, out, err = c.exec_command(cmd)
        print(out.read().decode(errors="replace"), err.read().decode(errors="replace"))
    c.exec_command("sync")[1].channel.recv_exit_status()
    c.close()
    print(f"{len(files)} files deployed, {bad} md5 mismatches")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
