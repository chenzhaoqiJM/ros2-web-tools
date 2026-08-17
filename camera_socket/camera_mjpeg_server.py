#!/usr/bin/env python3
import argparse
import socket
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


# Same camera parameters as person_follow_simple/capture_video11.py
CAMERA_INDEX = 11
WIDTH = 640
HEIGHT = 400
FPS = 30
FOURCC = "YUYV"

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8080
JPEG_QUALITY = 85


class CameraStreamer:
    def __init__(self, camera_index, width, height, fps, fourcc, jpeg_quality):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.jpeg_quality = jpeg_quality

        self._lock = threading.Lock()
        self._frame = None
        self._last_error = ""
        self._running = threading.Event()
        self._thread = None
        self._cap = None

    def start(self):
        self._running.set()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()

    def get_jpeg(self):
        with self._lock:
            return self._frame, self._last_error

    def _open_camera(self):
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open camera: /dev/video{self.camera_index}")

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _capture_loop(self):
        try:
            self._cap = self._open_camera()
            actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
            print(
                f"Opened /dev/video{self.camera_index}: "
                f"{actual_w}x{actual_h}, fps={actual_fps:.2f}, fourcc={self.fourcc}",
                flush=True,
            )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            print(f"Camera open error: {exc}", file=sys.stderr, flush=True)
            return

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

        while self._running.is_set():
            ok, frame = self._cap.read()
            if not ok:
                with self._lock:
                    self._last_error = "Failed to read frame"
                time.sleep(0.05)
                continue

            ok, encoded = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                with self._lock:
                    self._last_error = "Failed to encode frame"
                continue

            with self._lock:
                self._frame = encoded.tobytes()
                self._last_error = ""


class CameraHttpHandler(BaseHTTPRequestHandler):
    server_version = "CameraMJPEG/1.0"

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send_index()
        elif self.path == "/stream.mjpg":
            self._send_stream()
        elif self.path == "/snapshot.jpg":
            self._send_snapshot()
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def log_message(self, fmt, *args):
        print(f"{self.client_address[0]} - {fmt % args}", flush=True)

    def _send_index(self):
        html = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>/dev/video{self.server.streamer.camera_index}</title>
  <style>
    body {{
      margin: 0;
      min-height: 100vh;
      background: #101418;
      color: #e8eef5;
      font-family: Arial, "Microsoft YaHei", sans-serif;
      display: grid;
      grid-template-rows: auto 1fr;
    }}
    header {{
      padding: 14px 18px;
      background: #182029;
      border-bottom: 1px solid #293441;
      font-size: 16px;
    }}
    main {{
      display: grid;
      place-items: center;
      padding: 16px;
    }}
    img {{
      width: min(100%, 960px);
      height: auto;
      background: #000;
      border: 1px solid #293441;
    }}
  </style>
</head>
<body>
  <header>/dev/video{self.server.streamer.camera_index} live stream</header>
  <main><img src="/stream.mjpg" alt="camera stream"></main>
</body>
</html>
"""
        body = html.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_snapshot(self):
        frame, error = self.server.streamer.get_jpeg()
        if frame is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, error or "Camera frame not ready")
            return

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def _send_stream(self):
        self.send_response(HTTPStatus.OK)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        while True:
            frame, error = self.server.streamer.get_jpeg()
            if frame is None:
                if error:
                    print(f"Waiting for camera frame: {error}", flush=True)
                time.sleep(0.1)
                continue

            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / max(1, self.server.streamer.fps))
            except (BrokenPipeError, ConnectionResetError):
                break


class CameraHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address, handler_class, streamer):
        super().__init__(server_address, handler_class)
        self.streamer = streamer


def get_lan_ip():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def parse_args():
    parser = argparse.ArgumentParser(description="Serve /dev/video17 as an MJPEG browser stream.")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Bind host, default: 0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Bind port, default: 8080")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX, help="Camera index, default: 17")
    parser.add_argument("--width", type=int, default=WIDTH, help="Frame width, default: 640")
    parser.add_argument("--height", type=int, default=HEIGHT, help="Frame height, default: 400")
    parser.add_argument("--fps", type=int, default=FPS, help="Frame rate, default: 30")
    parser.add_argument("--fourcc", default=FOURCC, help="Camera FOURCC, default: YUYV")
    parser.add_argument("--quality", type=int, default=JPEG_QUALITY, help="JPEG quality 1-100, default: 85")
    return parser.parse_args()


def main():
    args = parse_args()
    streamer = CameraStreamer(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
        jpeg_quality=args.quality,
    )
    streamer.start()

    server = CameraHttpServer((args.host, args.port), CameraHttpHandler, streamer)
    lan_ip = get_lan_ip()

    print("Camera MJPEG server started", flush=True)
    print(f"Local: http://127.0.0.1:{args.port}/", flush=True)
    print(f"LAN:   http://{lan_ip}:{args.port}/", flush=True)
    print("Press Ctrl+C to stop", flush=True)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping...", flush=True)
    finally:
        server.shutdown()
        server.server_close()
        streamer.stop()


if __name__ == "__main__":
    main()
