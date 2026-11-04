"""PYNQ 板 SSH 小工具 (密码登录, 不依赖 SFTP).

用法:
  python pynq_ssh.py run "<cmd>"
  python pynq_ssh.py put <local_file> <remote_path>
"""
import hashlib
import sys

import paramiko

HOST, USER, PWD = "192.168.2.99", "xilinx", "xilinx"


def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PWD, timeout=10,
              look_for_keys=False, allow_agent=False)
    return c


def run(c, cmd, timeout=120):
    _, out, err = c.exec_command(cmd, timeout=timeout, get_pty=True)
    for line in iter(out.readline, ""):
        sys.stdout.write(line)
        sys.stdout.flush()
    return out.channel.recv_exit_status()


def put(c, local, remote):
    data = open(local, "rb").read()
    stdin, out, _ = c.exec_command(f"cat > '{remote}'")
    stdin.write(data)
    stdin.channel.shutdown_write()
    out.channel.recv_exit_status()
    _, out, _ = c.exec_command(f"md5sum '{remote}'")
    remote_md5 = out.read().decode().split()[0]
    local_md5 = hashlib.md5(data).hexdigest()
    ok = remote_md5 == local_md5
    print(f"{local} -> {remote}  md5 {'OK' if ok else 'MISMATCH'} ({local_md5})")
    return 0 if ok else 1


if __name__ == "__main__":
    c = connect()
    if sys.argv[1] == "run":
        code = run(c, sys.argv[2], timeout=int(sys.argv[3]) if len(sys.argv) > 3 else 120)
    else:
        code = put(c, sys.argv[2], sys.argv[3])
    c.close()
    sys.exit(code)
