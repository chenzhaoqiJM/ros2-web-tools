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
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog, ParticleCloud
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
        self.grid_versions = {"map": 0, "global_costmap": 0, "local_costmap": 0}
        self.plan = None
        self.local_plan = None
        self.amcl_pose = None
        self.particles = None
        self.footprints = {"global": None, "local": None}
        self.bt = {"stage": "等待导航", "status": "IDLE", "recovery": None}
        self.active_bt_nodes = {}
        self.navigation = {"distance_remaining": None, "estimated_time_remaining": None,
                           "navigation_time": None, "recoveries": 0, "replans": 0}
        self.plan_signature = None
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
            self.grid_versions[name] += 1
        self.notify()

    def update_path(self, message, name):
        with self.condition:
            path = {"frame": frame_name(message.header.frame_id),
                    "points": [pose_record(pose.pose) for pose in message.poses]}
            setattr(self, name, path)
            if name == "plan" and self.goal_active and path["points"]:
                points = path["points"]
                signature = (len(points),
                    tuple(round(value, 2) for value in points[0]["position"][:2]),
                    tuple(round(value, 2) for value in points[len(points) // 2]["position"][:2]),
                    tuple(round(value, 2) for value in points[-1]["position"][:2]))
                if self.plan_signature is not None and signature != self.plan_signature:
                    self.navigation["replans"] += 1
                self.plan_signature = signature
        self.notify()

    def update_amcl_pose(self, message):
        covariance = message.pose.covariance
        with self.condition:
            self.amcl_pose = {"frame": frame_name(message.header.frame_id),
                **pose_record(message.pose.pose),
                "covariance": [covariance[0], covariance[1], covariance[6], covariance[7]]}
        self.notify()

    def update_particles(self, message):
        # Limit browser payloads while retaining the distribution shape.
        particles = message.particles
        stride = max(1, math.ceil(len(particles) / 1000))
        with self.condition:
            self.particles = {"frame": frame_name(message.header.frame_id),
                "points": [{**pose_record(p.pose), "weight": p.weight}
                           for p in particles[::stride]], "total": len(particles)}
        self.notify()

    def update_footprint(self, message, name):
        with self.condition:
            self.footprints[name] = {"frame": frame_name(message.header.frame_id),
                "points": [[point.x, point.y, point.z] for point in message.polygon.points]}
        self.notify()

    def update_bt(self, message):
        recovery_words = ("recovery", "spin", "backup", "back_up", "wait",
                          "clear", "assistedteleop", "driveonheading")
        with self.condition:
            for event in message.event_log:
                name, status = event.node_name, event.current_status
                if status == "RUNNING":
                    self.active_bt_nodes[name] = time.time()
                    self.bt.update(stage=name, status=status)
                    if any(word in name.lower() for word in recovery_words):
                        self.bt["recovery"] = name
                else:
                    self.active_bt_nodes.pop(name, None)
                    if self.bt["stage"] == name:
                        self.bt["status"] = status
                    if self.bt["recovery"] == name:
                        self.bt["recovery"] = None
            if self.active_bt_nodes:
                self.bt["stage"] = max(self.active_bt_nodes, key=self.active_bt_nodes.get)
                self.bt["status"] = "RUNNING"
        self.notify()

    def update_feedback(self, feedback):
        duration = lambda value: value.sec + value.nanosec / 1e9
        with self.condition:
            self.navigation.update(
                distance_remaining=float(feedback.distance_remaining),
                estimated_time_remaining=duration(feedback.estimated_time_remaining),
                navigation_time=duration(feedback.navigation_time),
                recoveries=int(feedback.number_of_recoveries))
        self.notify()

    def reset_navigation(self):
        with self.condition:
            self.navigation = {"distance_remaining": None, "estimated_time_remaining": None,
                               "navigation_time": 0.0, "recoveries": 0, "replans": 0}
            self.plan_signature = None
            self.active_bt_nodes.clear()
            self.bt = {"stage": "等待行为树", "status": "IDLE", "recovery": None}

    def update_goal(self, message):
        with self.condition:
            self.goal = {"frame": frame_name(message.header.frame_id), **pose_record(message.pose)}
        self.notify()

    def snapshot(self):
        with self.condition:
            data = {"time": time.time(), "transforms": list(self.transforms.values()),
                    "goal": self.goal, "status": self.status, "goal_active": self.goal_active,
                    "plan": self.plan, "local_plan": self.local_plan,
                    "amcl_pose": self.amcl_pose, "particles": self.particles,
                    "footprints": self.footprints, "bt": self.bt,
                    "navigation": self.navigation, "grid_versions": self.grid_versions}
            return self.sequence, json.dumps(data, separators=(",", ":"), allow_nan=False)

    def map_snapshot(self):
        with self.condition:
            return json.dumps({"map": self.map, "global_costmap": self.global_costmap,
                               "local_costmap": self.local_costmap}, separators=(",", ":"),
                              allow_nan=False)


class Nav2Node(Node):
    def __init__(self, state, map_topic, global_costmap_topic, local_costmap_topic,
                 plan_topic, local_plan_topic, amcl_pose_topic, particle_topic,
                 global_footprint_topic, local_footprint_topic, bt_log_topic, action_name):
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
        self.create_subscription(PoseWithCovarianceStamped, amcl_pose_topic, state.update_amcl_pose, reliable)
        self.create_subscription(ParticleCloud, particle_topic, state.update_particles, dynamic)
        self.create_subscription(PolygonStamped, global_footprint_topic,
                                 lambda m: state.update_footprint(m, "global"), reliable)
        self.create_subscription(PolygonStamped, local_footprint_topic,
                                 lambda m: state.update_footprint(m, "local"), reliable)
        self.create_subscription(BehaviorTreeLog, bt_log_topic, state.update_bt, reliable)
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
        self.state.reset_navigation()
        future = self.action_client.send_goal_async(goal, feedback_callback=self.goal_feedback)
        future.add_done_callback(self.goal_response)
        with self.state.condition:
            self.state.goal = {"frame": frame, "position": [x, y, 0.0],
                               "rotation": [0.0, 0.0, goal.pose.pose.orientation.z, goal.pose.pose.orientation.w]}
            self.state.status, self.state.goal_active = "发送中", True
        self.state.notify()
        return True

    def goal_feedback(self, message):
        self.state.update_feedback(message.feedback)

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
            self.state.bt["stage"] = self.state.status
            self.state.bt["status"] = "SUCCESS" if status == GoalStatus.STATUS_SUCCEEDED else "IDLE"
            self.state.bt["recovery"] = None
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
    parser.add_argument("--amcl-pose-topic", default="/amcl_pose")
    parser.add_argument("--particle-topic", default="/particle_cloud")
    parser.add_argument("--global-footprint-topic", default="/global_costmap/published_footprint")
    parser.add_argument("--local-footprint-topic", default="/local_costmap/published_footprint")
    parser.add_argument("--bt-log-topic", default="/behavior_tree_log")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--fixed-frame", default="map")
    args = parser.parse_args()
    rclpy.init()
    state = SharedState()
    node = Nav2Node(state, args.map_topic, args.global_costmap_topic, args.local_costmap_topic,
                    args.plan_topic, args.local_plan_topic, args.amcl_pose_topic,
                    args.particle_topic, args.global_footprint_topic,
                    args.local_footprint_topic, args.bt_log_topic, args.action_name)
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
