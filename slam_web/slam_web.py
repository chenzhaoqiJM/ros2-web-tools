#!/usr/bin/env python3
"""ROS 2 SLAM map, laser scan, TF and trajectory browser visualizer."""

import argparse
import base64
import gzip
import html
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

WEB_DIR = Path(__file__).with_name("web")


def frame_name(value):
    return value.lstrip("/")


def tf_record(item):
    t, q = item.transform.translation, item.transform.rotation
    return {"parent": frame_name(item.header.frame_id), "child": frame_name(item.child_frame_id),
            "translation": [t.x, t.y, t.z], "rotation": [q.x, q.y, q.z, q.w]}


class SharedState:
    def __init__(self):
        self.condition = threading.Condition(threading.RLock())
        self.transforms = {}
        self.map = None
        self.map_version = 0
        self.scan = None
        self.trajectory = []
        self.sequence = 0

    def notify(self):
        with self.condition:
            self.sequence += 1
            self.condition.notify_all()

    def update_tf(self, message):
        with self.condition:
            for item in message.transforms:
                record = tf_record(item)
                if record["child"]:
                    self.transforms[record["child"]] = record
        self.notify()

    def update_map(self, message):
        info = message.info
        values = bytes((value + 1 if value >= 0 else 0 for value in message.data))
        compressed = base64.b64encode(gzip.compress(values, compresslevel=6)).decode("ascii")
        with self.condition:
            self.map = {"frame": frame_name(message.header.frame_id), "width": info.width,
                        "height": info.height, "resolution": info.resolution,
                        "origin": {"position": [info.origin.position.x, info.origin.position.y],
                                   "rotation": [info.origin.orientation.z, info.origin.orientation.w]},
                        "data": compressed}
            self.map_version += 1
        self.notify()

    def update_scan(self, message):
        with self.condition:
            ranges = [value if math.isfinite(value) else None for value in message.ranges]
            self.scan = {"frame": frame_name(message.header.frame_id), "angle_min": message.angle_min,
                         "angle_increment": message.angle_increment, "range_min": message.range_min,
                         "range_max": message.range_max, "ranges": ranges}
        self.notify()

    def update_markers(self, message):
        points = []
        for marker in message.markers:
            if marker.ns != "Trajectory" or marker.type not in (4, 5):
                continue
            points.extend([[p.x, p.y, p.z] for p in marker.points])
        with self.condition:
            self.trajectory = points
        self.notify()

    def snapshot(self):
        with self.condition:
            return self.sequence, json.dumps({"time": time.time(), "map_version": self.map_version,
                "transforms": list(self.transforms.values()), "scan": self.scan,
                "trajectory": self.trajectory}, separators=(",", ":"), allow_nan=False)

    def map_snapshot(self):
        with self.condition:
            return self.map_version, json.dumps(self.map, separators=(",", ":"), allow_nan=False) if self.map else "null"


class SlamNode(Node):
    def __init__(self, state, map_topic, scan_topic, trajectory_topic, cmd_vel_topic):
        super().__init__("slam_web")
        self.state = state
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        dynamic_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf", state.update_tf, dynamic_qos)
        self.create_subscription(TFMessage, "/tf_static", state.update_tf, static_qos)
        self.create_subscription(OccupancyGrid, map_topic, state.update_map, map_qos)
        self.create_subscription(LaserScan, scan_topic, state.update_scan, dynamic_qos)
        self.create_subscription(MarkerArray, trajectory_topic, state.update_markers, dynamic_qos)
        self.last_cmd_time = 0.0
        self.create_timer(0.1, self.stop_if_stale)
        self.cmd_vel_topic = cmd_vel_topic
        self.get_logger().info(f"Listening to {map_topic}, {scan_topic}, TF and {trajectory_topic}")

    def publish_cmd_vel(self, linear_x, angular_z):
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.cmd_vel_publisher.publish(message)
        self.last_cmd_time = time.monotonic()

    def stop_if_stale(self):
        if self.last_cmd_time and time.monotonic() - self.last_cmd_time > 0.5:
            self.publish_cmd_vel(0.0, 0.0)
            self.last_cmd_time = 0.0


class Handler(BaseHTTPRequestHandler):
    state = None
    node = None
    domain_id = "0"

    def log_message(self, fmt, *args):
        print(f"[web] {self.client_address[0]} {fmt % args}", flush=True)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/events":
            return self.events()
        if path == "/api/map":
            _, data = self.state.map_snapshot()
            return self.send_bytes(data.encode(), "application/json; charset=utf-8")
        files = {"/": "index.html", "/app.js": "app.js", "/style.css": "style.css"}
        if path not in files:
            self.send_error(404)
            return
        try:
            body = (WEB_DIR / files[path]).read_bytes()
        except OSError:
            self.send_error(500, "Web assets are missing")
            return
        if path == "/":
            body = body.decode().replace("{{ROS_DOMAIN_ID}}", html.escape(self.domain_id)).encode()
        content_type = {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}[Path(files[path]).suffix]
        self.send_bytes(body, content_type + "; charset=utf-8")

    def do_POST(self):
        if self.path.split("?", 1)[0] != "/api/cmd_vel":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length))
            linear_x = float(payload.get("linear_x", 0.0))
            angular_z = float(payload.get("angular_z", 0.0))
            if not math.isfinite(linear_x) or not math.isfinite(angular_z):
                raise ValueError("velocity must be finite")
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
            self.node.publish_cmd_vel(linear_x, angular_z)
            body = json.dumps({"linear_x": linear_x, "angular_z": angular_z}).encode()
            self.send_bytes(body, "application/json; charset=utf-8")
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.send_error(400, str(error))

    def send_bytes(self, body, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        last = -1
        try:
            while True:
                with self.state.condition:
                    self.state.condition.wait_for(lambda: self.state.sequence != last, timeout=10)
                    seq, data = self.state.snapshot()
                if seq == last:
                    self.wfile.write(b": keepalive\n\n")
                else:
                    self.wfile.write(f"id: {seq}\ndata: {data}\n\n".encode())
                    last = seq
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--trajectory-topic", default="/trajectory_node_list")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--max-linear-speed", type=float, default=0.5)
    parser.add_argument("--max-angular-speed", type=float, default=1.5)
    args = parser.parse_args()
    rclpy.init()
    state = SharedState()
    node = SlamNode(state, args.map_topic, args.scan_topic, args.trajectory_topic, args.cmd_vel_topic)
    Handler.state = state
    Handler.node = node
    Handler.max_linear_speed = max(0.0, args.max_linear_speed)
    Handler.max_angular_speed = max(0.0, args.max_angular_speed)
    Handler.domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info("Browser UI: " + (", ".join(lan_urls(args.port)) or f"port {args.port}"))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
