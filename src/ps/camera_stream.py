#!/usr/bin/env python3
"""USB 摄像头 MJPEG 推流 (板上运行, 电脑用浏览器 / OpenCV 看)

摄像头 (DF100-720P, UVC) 本身输出 MJPEG, 这里不解码, 原样把 JPEG 帧转发出去, ARM 几乎不占 CPU。

板上运行 (需要 root 才能访问摄像头设备):
  sudo /usr/local/share/pynq-venv/bin/python3 camera_stream.py [--width 1280 --height 720 --port 8080 --dev /dev/videoN]

电脑上:
  浏览器打开   http://192.168.2.99:8080/
  单帧         http://192.168.2.99:8080/snapshot.jpg
  OpenCV       cv2.VideoCapture("http://192.168.2.99:8080/stream")
"""
import argparse
import glob
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = "frame"


def find_camera():
    """USB 掉线重连后 /dev/videoN 的编号会变, 所以按 by-id 找第一个摄像头的取像节点"""
    nodes = sorted(glob.glob("/dev/v4l/by-id/*-video-index0"))
    return nodes[0] if nodes else "/dev/video0"

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>PYNQ Camera</title></head>
<body style="background:#222;color:#ddd;font-family:sans-serif;text-align:center">
<h3>PYNQ USB Camera ({w}x{h})</h3>
<img src="/stream" style="max-width:100%">
<p><a style="color:#8cf" href="/snapshot.jpg">snapshot.jpg</a></p>
</body></html>"""


class Camera:
    """后台线程持续抓帧, 只保留最新一帧 JPEG"""

    REOPEN_AFTER_S = 1.0   # 连续这么久读不到帧就重开设备 (USB 掉线重连后旧句柄会永久失效)

    def __init__(self, dev, width, height, fps):
        self.dev, self.req = dev, (width, height, fps)
        self.cap = None
        if not self._open():
            raise RuntimeError(f"打不开摄像头 {dev or find_camera()}")

        self.frame = None
        self.seq = 0
        self.cond = threading.Condition()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        if self.cap is not None:
            self.cap.release()
        width, height, fps = self.req
        dev = self.dev if self.dev is not None else find_camera()
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            return False
        print(f"[camera] 打开 {dev}", flush=True)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # 不解码, 直接拿 MJPEG 原始字节
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)    # 少缓存, 降延迟
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return True

    def _loop(self):
        n, t0 = 0, time.monotonic()
        last_ok = time.monotonic()
        while self.running:
            ok, buf = self.cap.read()
            if not ok:
                if time.monotonic() - last_ok > self.REOPEN_AFTER_S:
                    print("[camera] 读帧失败, 重新打开摄像头 ...", flush=True)
                    if self._open():
                        print(f"[camera] 已重连 {self.width}x{self.height}", flush=True)
                    else:
                        time.sleep(0.5)
                    last_ok = time.monotonic()
                else:
                    time.sleep(0.05)
                continue
            last_ok = time.monotonic()
            jpg = buf.tobytes()
            if jpg[:2] != b"\xff\xd8":          # 万一驱动给的是已解码的 BGR, 自己编码
                ok, enc = cv2.imencode(".jpg", buf)
                if not ok:
                    continue
                jpg = enc.tobytes()
            with self.cond:
                self.frame = jpg
                self.seq += 1
                self.cond.notify_all()
            n += 1
            if time.monotonic() - t0 >= 10:
                print(f"[camera] {n / (time.monotonic() - t0):.1f} fps, "
                      f"{len(jpg) / 1024:.0f} KB/帧", flush=True)
                n, t0 = 0, time.monotonic()

    def wait_frame(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.seq, self.frame

    def close(self):
        self.running = False
        self.cap.release()


def make_handler(cam):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print(f"[http] {self.client_address[0]} {fmt % args}", flush=True)

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8",
                           PAGE.format(w=cam.width, h=cam.height).encode())
            elif path == "/snapshot.jpg":
                _, jpg = cam.wait_frame(-1)
                if jpg is None:
                    self._send(503, "text/plain", b"no frame yet")
                else:
                    self._send(200, "image/jpeg", jpg)
            elif path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        seq, jpg = cam.wait_frame(seq)
                        if jpg is None:
                            continue
                        self.wfile.write(
                            f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(jpg)}\r\n\r\n".encode() + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._send(404, "text/plain", b"not found")

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default=None, help="设备路径, 默认自动找 USB 摄像头")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    cam = Camera(args.dev, args.width, args.height, args.fps)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(cam))
    server.daemon_threads = True
    print(f"摄像头 {cam.width}x{cam.height} MJPEG, 推流地址 http://<板子IP>:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        cam.close()


if __name__ == "__main__":
    main()
