#!/usr/bin/env python3
"""ROS 2 multi-sensor camera viewer with WebRTC streaming."""

import argparse
import asyncio
import fractions
import html
import json
import os
import socket
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path

try:
    import cv2
    import numpy as np
    import rclpy
    from aiohttp import web
    from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
    from av import VideoFrame
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import CompressedImage, Image
except ImportError as error:
    print(
        f"Missing dependency: {error}. Source your ROS 2 environment and run "
        "'python3 -m pip install -r requirements.txt'.",
        file=sys.stderr,
    )
    raise SystemExit(2) from error


WEB_DIR = Path(__file__).with_name("web")
IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/Image"}
COMPRESSED_TYPES = {"sensor_msgs/msg/CompressedImage", "sensor_msgs/CompressedImage"}
TIME_BASE = fractions.Fraction(1, 90000)


def sensor_kind(topic, encoding=""):
    """Infer RGB/depth/infrared from conventional topic and encoding names."""
    topic_value, encoding_value = topic.lower(), encoding.lower()
    if "depth" in topic_value:
        return "depth"
    if any(word in topic_value for word in ("infra", "_ir", "/ir", "thermal")):
        return "infrared"
    if any(word in encoding_value for word in ("depth", "16uc1", "32fc1")):
        return "depth"
    if "mono16" in encoding_value:
        return "infrared"
    return "rgb"


def _packed(message, dtype, channels=1):
    item_size = np.dtype(dtype).itemsize
    row_values = int(message.step) // item_size
    needed = row_values * int(message.height)
    data = np.frombuffer(message.data, dtype=dtype, count=needed)
    rows = data.reshape(int(message.height), row_values)
    width_values = int(message.width) * channels
    if row_values < width_values:
        raise ValueError(f"image step {message.step} is too small")
    result = rows[:, :width_values]
    shape = (int(message.height), int(message.width), channels) if channels > 1 else (
        int(message.height), int(message.width))
    return result.reshape(shape)


def _normalize_mono(image, low_percentile=1, high_percentile=99):
    finite = image[np.isfinite(image)]
    finite = finite[finite > 0]
    if not finite.size:
        return np.zeros(image.shape, np.uint8), None
    low, high = np.percentile(finite, (low_percentile, high_percentile))
    if high <= low:
        high = low + 1
    normalized = np.clip((image.astype(np.float32) - low) * (255.0 / (high - low)), 0, 255)
    normalized[~np.isfinite(image)] = 0
    return normalized.astype(np.uint8), (float(low), float(high))


def image_to_bgr(message, kind):
    encoding = message.encoding.lower()
    big = bool(message.is_bigendian)
    if encoding in ("bgr8", "rgb8"):
        image = _packed(message, np.uint8, 3)
        return image.copy() if encoding == "bgr8" else cv2.cvtColor(image, cv2.COLOR_RGB2BGR), None
    if encoding in ("bgra8", "rgba8"):
        image = _packed(message, np.uint8, 4)
        code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(image, code), None
    if encoding in ("mono8", "8uc1"):
        gray = _packed(message, np.uint8)
        if kind == "depth":
            return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO), None
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), None
    if encoding in ("mono16", "16uc1"):
        dtype = np.dtype(">u2" if big else "<u2")
        image = _packed(message, dtype).astype(np.float32)
        gray, limits = _normalize_mono(image)
        if kind == "depth":
            return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO), limits
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), limits
    if encoding == "32fc1":
        dtype = np.dtype(">f4" if big else "<f4")
        image = _packed(message, dtype).astype(np.float32)
        gray, limits = _normalize_mono(image)
        return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO), limits
    bayer_codes = {
        "bayer_rggb8": cv2.COLOR_BayerBG2BGR, "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
        "bayer_gbrg8": cv2.COLOR_BayerGR2BGR, "bayer_grbg8": cv2.COLOR_BayerGB2BGR,
    }
    if encoding in bayer_codes:
        return cv2.cvtColor(_packed(message, np.uint8), bayer_codes[encoding]), None
    raise ValueError(f"unsupported encoding: {message.encoding}")


@dataclass
class CameraSource:
    topic: str
    message_type: str
    kind: str
    subscription: object = None
    frame: object = None
    encoding: str = ""
    width: int = 0
    height: int = 0
    frame_count: int = 0
    last_frame_at: float = 0.0
    error: str = "等待图像"
    value_range: object = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, frame, encoding, value_range=None):
        with self.lock:
            self.frame = np.ascontiguousarray(frame)
            self.height, self.width = frame.shape[:2]
            self.encoding = encoding
            self.kind = sensor_kind(self.topic, encoding)
            self.frame_count += 1
            self.last_frame_at = time.monotonic()
            self.error = ""
            self.value_range = value_range

    def set_error(self, error):
        with self.lock:
            self.error = str(error)

    def get_frame(self):
        with self.lock:
            return self.frame, self.frame_count

    def info(self):
        with self.lock:
            age = time.monotonic() - self.last_frame_at if self.last_frame_at else None
            return {
                "topic": self.topic, "message_type": self.message_type, "kind": self.kind,
                "encoding": self.encoding, "width": self.width, "height": self.height,
                "frame_count": self.frame_count, "age": age, "error": self.error,
                "range": self.value_range,
            }


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_web")
        self.sources = {}
        self.sources_lock = threading.RLock()
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_timer(2.0, self.discover)
        self.discover()

    def discover(self):
        advertised = dict(self.get_topic_names_and_types())
        supported = {}
        for topic, types in advertised.items():
            message_type = next((value for value in types if value in IMAGE_TYPES | COMPRESSED_TYPES), None)
            if message_type:
                supported[topic] = message_type
        with self.sources_lock:
            for topic, message_type in supported.items():
                if topic in self.sources:
                    continue
                source = CameraSource(topic, message_type, sensor_kind(topic))
                message_class = CompressedImage if message_type in COMPRESSED_TYPES else Image
                source.subscription = self.create_subscription(
                    message_class, topic, lambda msg, item=source: self.on_image(item, msg), self.qos)
                self.sources[topic] = source
                self.get_logger().info(f"Discovered {message_type}: {topic}")
            for topic in set(self.sources) - set(supported):
                source = self.sources.pop(topic)
                self.destroy_subscription(source.subscription)
                self.get_logger().info(f"Image topic disappeared: {topic}")

    def on_image(self, source, message):
        try:
            if isinstance(message, CompressedImage):
                buffer = np.frombuffer(message.data, dtype=np.uint8)
                frame = cv2.imdecode(buffer, cv2.IMREAD_UNCHANGED)
                if frame is None:
                    raise ValueError("compressed image decode failed")
                if frame.ndim == 2:
                    kind = sensor_kind(source.topic, message.format)
                    gray, limits = _normalize_mono(frame) if frame.dtype != np.uint8 else (frame, None)
                    frame = (cv2.applyColorMap(gray, cv2.COLORMAP_TURBO) if kind == "depth"
                             else cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
                elif frame.shape[2] == 4:
                    frame, limits = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR), None
                else:
                    limits = None
                source.update(frame, message.format or "compressed", limits)
            else:
                kind = sensor_kind(source.topic, message.encoding)
                frame, limits = image_to_bgr(message, kind)
                source.update(frame, message.encoding, limits)
        except Exception as error:
            source.set_error(error)

    def source(self, topic):
        with self.sources_lock:
            return self.sources.get(topic)

    def snapshot(self):
        with self.sources_lock:
            values = [source.info() for source in self.sources.values()]
        values.sort(key=lambda item: ("rgb depth infrared".find(item["kind"]), item["topic"]))
        return values


class CameraVideoTrack(MediaStreamTrack):
    kind = "video"

    PROFILES = {
        "low": (640, 360, 12),
        "balanced": (1280, 720, 24),
        "high": (1920, 1080, 30),
    }

    def __init__(self, source, profile="balanced"):
        super().__init__()
        self.source = source
        self.profile = profile if profile in self.PROFILES else "balanced"
        self.started = time.monotonic()
        self.next_frame_at = self.started
        self.last_count = -1
        self.last_frame = None

    def set_profile(self, profile):
        if profile in self.PROFILES:
            self.profile = profile

    async def recv(self):
        max_width, max_height, fps = self.PROFILES[self.profile]
        interval = 1.0 / fps
        delay = self.next_frame_at - time.monotonic()
        if delay > 0:
            await asyncio.sleep(delay)
        self.next_frame_at = max(self.next_frame_at + interval, time.monotonic())
        frame, count = self.source.get_frame()
        if frame is not None and (count != self.last_count or self.last_frame is None):
            height, width = frame.shape[:2]
            factor = min(1.0, max_width / width, max_height / height)
            if factor < 1:
                frame = cv2.resize(frame, (round(width * factor), round(height * factor)),
                                   interpolation=cv2.INTER_AREA)
            self.last_frame = frame.copy()
            self.last_count = count
        if self.last_frame is None:
            self.last_frame = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(self.last_frame, "Waiting for ROS image", (145, 185),
                        cv2.FONT_HERSHEY_SIMPLEX, .75, (120, 140, 150), 2, cv2.LINE_AA)
        video = VideoFrame.from_ndarray(self.last_frame, format="bgr24")
        video.pts = int((time.monotonic() - self.started) * 90000)
        video.time_base = TIME_BASE
        return video


class CameraWebApp:
    def __init__(self, node, domain_id):
        self.node = node
        self.domain_id = domain_id
        self.peers = {}
        self.app = web.Application(client_max_size=1024 * 1024)
        self.app.add_routes([
            web.get("/", self.index), web.get("/app.js", self.asset),
            web.get("/style.css", self.asset), web.get("/api/topics", self.topics),
            web.post("/api/offer", self.offer),
            web.post("/api/peers/{peer_id}/quality", self.quality),
            web.delete("/api/peers/{peer_id}", self.close_peer),
        ])
        self.app.on_shutdown.append(self.shutdown)

    async def index(self, _request):
        body = (WEB_DIR / "index.html").read_text(encoding="utf-8")
        body = body.replace("{{ROS_DOMAIN_ID}}", html.escape(self.domain_id))
        return web.Response(text=body, content_type="text/html")

    async def asset(self, request):
        name = request.path.lstrip("/")
        content_type = "text/javascript" if name.endswith(".js") else "text/css"
        return web.Response(body=(WEB_DIR / name).read_bytes(), content_type=content_type,
                            headers={"Cache-Control": "no-store"})

    async def topics(self, _request):
        return web.json_response({"topics": self.node.snapshot(), "time": time.time()})

    async def offer(self, request):
        payload = await request.json()
        topic = str(payload.get("topic", ""))
        source = self.node.source(topic)
        if source is None:
            raise web.HTTPNotFound(text=json.dumps({"error": "图像话题不存在"}),
                                   content_type="application/json")
        description = RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])
        peer = RTCPeerConnection()
        peer_id = uuid.uuid4().hex
        track = CameraVideoTrack(source, payload.get("quality", "balanced"))
        self.peers[peer_id] = (peer, track)

        @peer.on("connectionstatechange")
        async def state_changed():
            if peer.connectionState in ("failed", "closed"):
                await self._close(peer_id)

        await peer.setRemoteDescription(description)
        peer.addTrack(track)
        answer = await peer.createAnswer()
        await peer.setLocalDescription(answer)
        return web.json_response({"sdp": peer.localDescription.sdp,
                                  "type": peer.localDescription.type, "peer_id": peer_id})

    async def quality(self, request):
        value = self.peers.get(request.match_info["peer_id"])
        if value is None:
            raise web.HTTPNotFound()
        payload = await request.json()
        value[1].set_profile(payload.get("quality", "balanced"))
        return web.json_response({"quality": value[1].profile})

    async def close_peer(self, request):
        await self._close(request.match_info["peer_id"])
        return web.Response(status=204)

    async def _close(self, peer_id):
        value = self.peers.pop(peer_id, None)
        if value:
            value[1].stop()
            await value[0].close()

    async def shutdown(self, _app):
        await asyncio.gather(*(self._close(peer_id) for peer_id in list(self.peers)))


def lan_urls(port):
    addresses = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                addresses.add(ip)
    except socket.gaierror:
        pass
    return [f"http://{ip}:{port}" for ip in sorted(addresses)]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8768)
    return parser.parse_args(remove_ros_args(sys.argv)[1:])


def main():
    args = parse_args()
    rclpy.init(args=sys.argv)
    node = CameraNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    camera_app = CameraWebApp(node, os.environ.get("ROS_DOMAIN_ID", "0"))
    node.get_logger().info("Browser UI: " + (", ".join(lan_urls(args.port)) or f"port {args.port}"))
    try:
        web.run_app(camera_app.app, host=args.host, port=args.port, print=None)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
