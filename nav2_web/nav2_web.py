#!/usr/bin/env python3
"""ROS 2 Nav2 browser map viewer and goal publisher."""

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
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

WEB_DIR = Path(__file__).with_name("web")


def frame_name(value):
    return value.lstrip("/")


def pose_record(pose):
    p, q = pose.position, pose.orientation
    return {"position": [p.x, p.y, p.z], "rotation": [q.x, q.y, q.z, q.w]}


def tf_record(item):
    t, q = item.transform.translation, item.transform.rotation
    return {"parent": frame_name(item.header.frame_id), "child": frame_name(item.child_frame_id),
            "translation": [t.x, t.y, t.z], "rotation": [q.x, q.y, q.z, q.w]}


def grid_record(message):
    info = message.info
    values = bytes(value + 1 if value >= 0 else 0 for value in message.data)
    return {"frame": frame_name(message.header.frame_id), "width": info.width, "height": info.height,
            "resolution": info.resolution,
            "origin": {"position": [info.origin.position.x, info.origin.position.y],
                       "rotation": [info.origin.orientation.z, info.origin.orientation.w]},
            "data": base64.b64encode(gzip.compress(values)).decode("ascii")}


class SharedState:
    def __init__(self):
        self.condition = threading.Condition(threading.RLock())
        self.transforms = {}
        self.map = None
        self.global_costmap = None
        self.local_costmap = None
        self.plan = None
        self.local_plan = None
        self.goal = None
        self.status = "等待目标"
        self.goal_active = False
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

    def update_grid(self, message, name):
        with self.condition:
            setattr(self, name, grid_record(message))
        self.notify()

    def update_path(self, message, name):
        with self.condition:
            setattr(self, name, {"frame": frame_name(message.header.frame_id),
                "points": [pose_record(pose.pose) for pose in message.poses]})
        self.notify()

    def update_goal(self, message):
        with self.condition:
            self.goal = {"frame": frame_name(message.header.frame_id), **pose_record(message.pose)}
        self.notify()

    def snapshot(self):
        with self.condition:
            data = {"time": time.time(), "transforms": list(self.transforms.values()),
                    "goal": self.goal, "status": self.status, "goal_active": self.goal_active,
                    "plan": self.plan, "local_plan": self.local_plan}
            return self.sequence, json.dumps(data, separators=(",", ":"), allow_nan=False)

    def map_snapshot(self):
        with self.condition:
            return json.dumps({"map": self.map, "global_costmap": self.global_costmap,
                               "local_costmap": self.local_costmap}, separators=(",", ":"),
                              allow_nan=False)


class Nav2Node(Node):
    def __init__(self, state, map_topic, global_costmap_topic, local_costmap_topic,
                 plan_topic, local_plan_topic, action_name):
        super().__init__("nav2_web")
        self.state = state
        reliable_static = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        dynamic = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(TFMessage, "/tf", state.update_tf, dynamic)
        self.create_subscription(TFMessage, "/tf_static", state.update_tf, reliable_static)
        self.create_subscription(OccupancyGrid, map_topic, lambda m: state.update_grid(m, "map"), reliable_static)
        self.create_subscription(OccupancyGrid, global_costmap_topic,
                                 lambda m: state.update_grid(m, "global_costmap"), reliable_static)
        self.create_subscription(OccupancyGrid, local_costmap_topic,
                                 lambda m: state.update_grid(m, "local_costmap"), reliable_static)
        self.create_subscription(RosPath, plan_topic, lambda m: state.update_path(m, "plan"), reliable)
        self.create_subscription(RosPath, local_plan_topic, lambda m: state.update_path(m, "local_plan"), reliable)
        self.action_client = ActionClient(self, NavigateToPose, action_name)
        self.goal_handle = None
        self.get_logger().info(f"Listening to {map_topic}, {plan_topic}, {action_name}")

    def send_goal(self, x, y, yaw, frame):
        if not self.action_client.server_is_ready():
            with self.state.condition:
                self.state.status = "Nav2 action 不可用"
            self.state.notify()
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)
        with self.state.condition:
            self.state.goal = {"frame": frame, "position": [x, y, 0.0],
                               "rotation": [0.0, 0.0, goal.pose.pose.orientation.z, goal.pose.pose.orientation.w]}
            self.state.status, self.state.goal_active = "发送中", True
        self.state.notify()
        return True

    def goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                with self.state.condition:
                    self.state.status, self.state.goal_active = "目标被拒绝", False
                self.state.notify()
                return
            self.goal_handle = handle
            with self.state.condition:
                self.state.status = "导航执行中"
            self.state.notify()
            result_future = handle.get_result_async()
            result_future.add_done_callback(self.goal_result)
        except Exception as error:
            with self.state.condition:
                self.state.status, self.state.goal_active = f"发送失败: {error}", False
            self.state.notify()

    def goal_result(self, future):
        status = future.result().status
        labels = {GoalStatus.STATUS_SUCCEEDED: "导航完成", GoalStatus.STATUS_CANCELED: "已取消",
                  GoalStatus.STATUS_ABORTED: "导航失败"}
        with self.state.condition:
            self.state.status = labels.get(status, f"导航结束 ({status})")
            self.state.goal_active = False
        self.state.notify()

    def cancel_goal(self):
        if self.goal_handle is None:
            return False
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _: self.state.notify())
        with self.state.condition:
            self.state.status = "取消中"
        self.state.notify()
        return True


class Handler(BaseHTTPRequestHandler):
    state = None
    node = None
    domain_id = "0"
    fixed_frame = "map"

    def log_message(self, fmt, *args):
        print(f"[web] {self.client_address[0]} {fmt % args}", flush=True)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/events":
            return self.events()
        if path == "/api/map":
            return self.send_bytes(self.state.map_snapshot().encode(), "application/json; charset=utf-8")
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
            body = body.decode().replace("{{ROS_DOMAIN_ID}}", html.escape(self.domain_id))
            body = body.replace("{{FIXED_FRAME}}", html.escape(self.fixed_frame)).encode()
        content_type = {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}[Path(files[path]).suffix]
        return self.send_bytes(body, content_type + "; charset=utf-8")

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length)) if length else {}
            if path == "/api/goal":
                x, y, yaw = (float(payload[key]) for key in ("x", "y", "yaw"))
                frame = frame_name(str(payload.get("frame", self.fixed_frame))) or self.fixed_frame
                if not all(math.isfinite(value) for value in (x, y, yaw)):
                    raise ValueError("坐标必须是有限数")
                ok = self.node.send_goal(x, y, yaw, frame)
                return self.json_response({"accepted": ok})
            if path == "/api/cancel":
                return self.json_response({"accepted": self.node.cancel_goal()})
            self.send_error(404)
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.send_error(400, str(error))

    def json_response(self, value):
        return self.send_bytes(json.dumps(value).encode(), "application/json; charset=utf-8")

    def send_bytes(self, body, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
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
            if not info[4][0].startswith("127."):
                addresses.add(info[4][0])
    except socket.gaierror:
        pass
    return [f"http://{ip}:{port}" for ip in sorted(addresses)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--global-costmap-topic", default="/global_costmap/costmap")
    parser.add_argument("--local-costmap-topic", default="/local_costmap/costmap")
    parser.add_argument("--plan-topic", default="/plan")
    parser.add_argument("--local-plan-topic", default="/local_plan")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--fixed-frame", default="map")
    args = parser.parse_args()
    rclpy.init()
    state = SharedState()
    node = Nav2Node(state, args.map_topic, args.global_costmap_topic, args.local_costmap_topic,
                    args.plan_topic, args.local_plan_topic, args.action_name)
    Handler.state, Handler.node = state, node
    Handler.domain_id, Handler.fixed_frame = os.environ.get("ROS_DOMAIN_ID", "0"), args.fixed_frame
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
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
