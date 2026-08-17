#!/usr/bin/env python3
import argparse
import socket
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


DEFAULT_TOPIC = "/face_tracker_demo/debug_image"
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8081
DEFAULT_FPS = 30
JPEG_QUALITY = 85


class RosImageStreamer:
    def __init__(self, topic, jpeg_quality, stream_fps):
        self.topic = topic
        self.jpeg_quality = jpeg_quality
        self.stream_fps = stream_fps

        self._lock = threading.Lock()
        self._frame = None
        self._last_error = "Waiting for ROS image"
        self._last_stamp = ""
        self._frame_count = 0
        self._last_frame_time = 0.0
        self._running = threading.Event()
        self._thread = None
        self._node = None

    def start(self):
        self._running.set()
        self._node = rclpy.create_node("ros_image_mjpeg_server")
        # qos = QoSProfile(
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1,
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        # )
        self._node.create_subscription(Image, self.topic, self._on_image, 10)
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None

    def get_jpeg(self):
        with self._lock:
            stats = {
                "topic": self.topic,
                "frame_count": self._frame_count,
                "last_stamp": self._last_stamp,
                "age_sec": time.monotonic() - self._last_frame_time
                if self._last_frame_time
                else None,
            }
            return self._frame, self._last_error, stats

    def _spin_loop(self):
        print(f"Subscribing ROS image topic: {self.topic}", flush=True)
        while self._running.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"ROS spin error: {exc}"
                print(self._last_error, file=sys.stderr, flush=True)
                time.sleep(0.2)

    def _on_image(self, msg):
        try:
            frame = image_msg_to_bgr8(msg)
        except Exception as exc:
            with self._lock:
                self._last_error = f"Failed to convert ROS image: {exc}"
            return

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        ok, encoded = cv2.imencode(".jpg", frame, encode_params)
        if not ok:
            with self._lock:
                self._last_error = "Failed to encode frame"
            return

        stamp = msg.header.stamp
        with self._lock:
            self._frame = encoded.tobytes()
            self._last_error = ""
            self._last_stamp = f"{stamp.sec}.{stamp.nanosec:09d}"
            self._frame_count += 1
            self._last_frame_time = time.monotonic()


def image_msg_to_bgr8(msg):
    encoding = msg.encoding.lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if height <= 0 or width <= 0:
        raise ValueError(f"invalid image size: {width}x{height}")

    if encoding in ("bgr8", "rgb8"):
        channels = 3
        expected_step = width * channels
        image = _reshape_image(data, height, width, channels, step, expected_step)
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    if encoding in ("bgra8", "rgba8"):
        channels = 4
        expected_step = width * channels
        image = _reshape_image(data, height, width, channels, step, expected_step)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    if encoding in ("mono8", "8uc1"):
        image = _reshape_image(data, height, width, 1, step, width)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    raise ValueError(f"unsupported encoding: {msg.encoding}")


def _reshape_image(data, height, width, channels, step, expected_step):
    if step < expected_step:
        raise ValueError(f"image step {step} is smaller than expected {expected_step}")

    needed = step * height
    if data.size < needed:
        raise ValueError(f"image data too small: {data.size} < {needed}")

    rows = data[:needed].reshape((height, step))
    packed = rows[:, :expected_step]
    if channels == 1:
        return packed.reshape((height, width)).copy()
    return packed.reshape((height, width, channels)).copy()


class RosImageHttpHandler(BaseHTTPRequestHandler):
    server_version = "RosImageMJPEG/1.0"

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
  <title>{self.server.streamer.topic}</title>
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
  <header>{self.server.streamer.topic} live stream</header>
  <main><img src="/stream.mjpg" alt="ROS image stream"></main>
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
        frame, error, _ = self.server.streamer.get_jpeg()
        if frame is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, error or "ROS image not ready")
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
            frame, error, _ = self.server.streamer.get_jpeg()
            if frame is None:
                if error:
                    print(f"Waiting for ROS image: {error}", flush=True)
                time.sleep(0.1)
                continue

            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / max(1, self.server.streamer.stream_fps))
            except (BrokenPipeError, ConnectionResetError):
                break


class RosImageHttpServer(ThreadingHTTPServer):
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
    parser = argparse.ArgumentParser(
        description="Serve a ROS sensor_msgs/Image topic as an MJPEG browser stream."
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="Bind host, default: 0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Bind port, default: 8081")
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help="ROS image topic, default: /face_tracker_demo/debug_image",
    )
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Browser stream FPS, default: 30")
    parser.add_argument("--quality", type=int, default=JPEG_QUALITY, help="JPEG quality 1-100, default: 85")
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    rclpy.init(args=sys.argv)
    streamer = RosImageStreamer(
        topic=args.topic,
        jpeg_quality=args.quality,
        stream_fps=args.fps,
    )
    streamer.start()

    server = RosImageHttpServer((args.host, args.port), RosImageHttpHandler, streamer)
    lan_ip = get_lan_ip()

    print("ROS image MJPEG server started", flush=True)
    print(f"Topic: {args.topic}", flush=True)
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
