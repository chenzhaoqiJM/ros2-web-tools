#!/usr/bin/env python3
"""ROS 2 TF tree and PoseStamped browser visualizer (stdlib HTTP server)."""

import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


WEB_DIR = Path(__file__).with_name("web")


def clean_frame(value: str) -> str:
    return value.lstrip("/")


def transform_dict(item):
    t, q = item.transform.translation, item.transform.rotation
    return {
        "parent": clean_frame(item.header.frame_id),
        "child": clean_frame(item.child_frame_id),
        "translation": [t.x, t.y, t.z],
        "rotation": [q.x, q.y, q.z, q.w],
    }


class SharedState:
    def __init__(self):
        # RLock permits snapshot() while the SSE sender already owns the condition.
        self.condition = threading.Condition(threading.RLock())
        self.transforms = {}
        self.target = None
        self.planned_path = None
        self.task_status = "unknown"
        self.current_pose = None
        self.sequence = 0
        self.clients = 0

    def update_tf(self, message):
        with self.condition:
            for item in message.transforms:
                record = transform_dict(item)
                if record["child"]:
                    self.transforms[record["child"]] = record

    def update_target(self, message):
        p, q = message.pose.position, message.pose.orientation
        with self.condition:
            self.target = {
                "frame": clean_frame(message.header.frame_id),
                "position": [p.x, p.y, p.z],
                "rotation": [q.x, q.y, q.z, q.w],
                "received_at": time.time(),
            }

    @staticmethod
    def pose_record(message):
        p, q = message.pose.position, message.pose.orientation
        return {"position": [p.x, p.y, p.z], "rotation": [q.x, q.y, q.z, q.w]}

    def update_path(self, message):
        with self.condition:
            self.planned_path = {
                "frame": clean_frame(message.header.frame_id),
                "points": [self.pose_record(pose) for pose in message.poses],
                "received_at": time.time(),
            }

    def update_status(self, message):
        with self.condition:
            self.task_status = message.data

    def update_current_pose(self, message):
        with self.condition:
            self.current_pose = {
                "frame": clean_frame(message.header.frame_id),
                **self.pose_record(message),
                "received_at": time.time(),
            }

    def publish(self):
        with self.condition:
            self.sequence += 1
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return self.sequence, json.dumps({
                "time": time.time(),
                "transforms": list(self.transforms.values()),
                "target": self.target,
                "planned_path": self.planned_path,
                "task_status": self.task_status,
                "current_pose": self.current_pose,
            }, separators=(",", ":"), allow_nan=False)


class TfWebNode(Node):
    def __init__(self, state, target_topic):
        super().__init__("remote_tf_web")
        self.state = state
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(TFMessage, "/tf", state.update_tf, tf_qos)
        self.create_subscription(TFMessage, "/tf_static", state.update_tf, static_qos)
        self.create_subscription(PoseStamped, target_topic, state.update_target, pose_qos)
        self.create_subscription(RosPath, "/grasp_task/planned_path", state.update_path, static_qos)
        self.create_subscription(String, "/grasp_task/status", state.update_status, static_qos)
        self.create_subscription(PoseStamped, "/grasp_task/current_pose", state.update_current_pose, pose_qos)
        self.create_timer(0.05, state.publish)
        self.get_logger().info(
            f"Listening to TF, {target_topic}, /grasp_task/planned_path and task feedback")


class WebHandler(BaseHTTPRequestHandler):
    state = None

    def log_message(self, fmt, *args):
        print(f"[web] {self.client_address[0]} {fmt % args}")

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/events":
            return self.serve_events()
        if path == "/api/status":
            seq, data = self.state.snapshot()
            return self.send_bytes(data.encode(), "application/json; charset=utf-8")
        files = {"/": "index.html", "/app.js": "app.js", "/style.css": "style.css"}
        if path not in files:
            self.send_error(404)
            return
        content_type = {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}[Path(files[path]).suffix]
        try:
            body = (WEB_DIR / files[path]).read_bytes()
        except OSError:
            self.send_error(500, "Web assets are missing")
            return
        self.send_bytes(body, content_type + "; charset=utf-8")

    def send_bytes(self, body, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def serve_events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        last = -1
        with self.state.condition:
            self.state.clients += 1
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
        finally:
            with self.state.condition:
                self.state.clients -= 1


def local_addresses(port):
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
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--target-topic", default="/grasp_task/target_pose")
    args = parser.parse_args()

    rclpy.init()
    state = SharedState()
    node = TfWebNode(state, args.target_topic)
    WebHandler.state = state
    server = ThreadingHTTPServer((args.host, args.port), WebHandler)
    thread = threading.Thread(target=server.serve_forever, name="http-server", daemon=True)
    thread.start()
    node.get_logger().info("Browser UI: " + (", ".join(local_addresses(args.port)) or f"port {args.port}"))
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
