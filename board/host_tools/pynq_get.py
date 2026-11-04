"""从 PYNQ 板打包下载文件/目录到本地（无 SFTP，走 exec_command 二进制流，逐文件 md5 校验）。

用法:
  python pynq_get.py <本地目标目录> <板上基目录> <相对路径1> [相对路径2 ...]
例:
  python board/host_tools/pynq_get.py data/imu_calib /home/xilinx/jupyter_notebooks imu_calib.json imu_calib_data/session_xxx
"""
import hashlib
import io
import os
import sys
import tarfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pynq_ssh import connect


def main():
    dst, base, rels = sys.argv[1], sys.argv[2], sys.argv[3:]
    os.makedirs(dst, exist_ok=True)
    c = connect()
    quoted = " ".join(f"'{r}'" for r in rels)
    _, out, err = c.exec_command(f"cd '{base}' && tar czf - {quoted}")
    data = out.read()
    rc = out.channel.recv_exit_status()
    if rc != 0:
        sys.exit(f"remote tar failed rc={rc}: {err.read().decode(errors='replace')}")
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tf:
        names = [m.name for m in tf.getmembers() if m.isfile()]
        tf.extractall(dst, filter="data")
    _, out, _ = c.exec_command(f"cd '{base}' && md5sum " + " ".join(f"'{n}'" for n in names))
    remote = dict(reversed(l.split(None, 1)) for l in out.read().decode().splitlines() if l.strip())
    bad = 0
    for n in names:
        local_md5 = hashlib.md5(open(os.path.join(dst, n), "rb").read()).hexdigest()
        ok = remote.get(n) == local_md5
        bad += not ok
        print(f"{'OK ' if ok else 'BAD'} {n}")
    print(f"{len(names)} files, {bad} mismatches -> {os.path.abspath(dst)}")
    c.close()
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
