"""ROS 2 node with a local HTTP/SSE gateway; no dashboard-only simulation."""

import json
import math
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from .state import TelemetryStore, finite, image_png
from .contracts import validate_payload

JSON_TOPICS = (
    "/hri/observations", "/hri/human_intent", "/hri/belief", "/hri/warning",
    "/hri/routes", "/hri/status", "/hri/events", "/hri/navigation",
    "/hri/operator_belief", "/hri/operator_warning", "/hri/operator_routes",
    "/hri/uav_status", "/forest/routes", "/forest/fire_state",
    "/hri/decision", "/hri/rescue_route", "/hri/robot_route", "/hri/acknowledgement",
    "/hri/facility", "/hri/operator_actions", "/hri/operator_intent",
    "/hri/tom_evidence", "/hri/route_observations", "/hri/operator_receipt",
    "/hri/equipment_registry",
)


class EmergencyStopController:
    """Async SetBool adapter. Service results never synthesize latch telemetry."""

    def __init__(self, store, client, request_factory, clock=time.monotonic):
        self.store, self.client = store, client
        self.request_factory, self.clock = request_factory, clock
        self.sequence = 0
        self.pending = {}

    def report(self, sequence, enabled, state, success=None, message=""):
        if sequence != self.sequence:
            return  # An older response must not overwrite a newer request.
        self.store.update("/dashboard/emergency_stop_result", {
            "source": "/robot/emergency_stop", "request_id": sequence,
            "requested_enabled": enabled, "state": state,
            "success": success, "message": message,
        })

    def request(self, enabled):
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a boolean")
        self.sequence += 1
        sequence = self.sequence
        if not self.client.service_is_ready():
            self.report(sequence, enabled, "unavailable", False, "Emergency-stop service is unavailable")
            return
        self.report(sequence, enabled, "requesting", message="Awaiting ROS service response and latch telemetry")
        try:
            future = self.client.call_async(self.request_factory(enabled))
            self.pending[sequence] = (future, enabled, self.clock())
            future.add_done_callback(lambda result: self.completed(sequence, enabled, result))
        except Exception as exc:
            self.pending.pop(sequence, None)
            self.report(sequence, enabled, "failed", False, str(exc))

    def completed(self, sequence, enabled, future):
        if self.pending.pop(sequence, None) is None:
            return
        try:
            response = future.result()
            self.report(sequence, enabled, "applied" if response.success else "rejected",
                        bool(response.success), response.message)
        except Exception as exc:
            self.report(sequence, enabled, "failed", False, str(exc))

    def poll(self):
        for sequence, (_, enabled, requested_at) in list(self.pending.items()):
            if self.clock() - requested_at > 5.0:
                self.pending.pop(sequence, None)
                self.report(sequence, enabled, "response_timeout", False,
                            "No service response within 5 seconds; latch state must be checked separately")


def make_handler(store, commands, emergency_commands=None, uav_store=None):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def reply(self, status, value, content_type="application/json"):
            body = value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/state":
                self.reply(200, store.snapshot())
            elif path == "/api/health":
                self.reply(200, {"service": "hri_dashboard", "source": "ROS 2", "synthetic": False})
            elif path == "/api/camera.png":
                png = store.camera()
                self.reply(200, png, "image/png") if png else self.reply(503, {"error": "No recent ROS camera frame"})
            elif path == "/api/uav-camera.png":
                png = uav_store.camera() if uav_store else None
                self.reply(200, png, "image/png") if png else self.reply(503, {"error": "No recent UAV camera frame"})
            elif path == "/api/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                try:
                    while not getattr(self.server, "stopping", False):
                        payload = json.dumps(store.snapshot(), allow_nan=False)
                        self.wfile.write(f"data: {payload}\n\n".encode())
                        self.wfile.flush()
                        time.sleep(0.5)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                self.close_connection = True
            else:
                self.reply(404, {"error": "Unknown endpoint"})

        def do_POST(self):
            if urlparse(self.path).path != "/api/control":
                self.reply(404, {"error": "Unknown endpoint"})
                return
            # Browser requests must be JSON, so other origins cannot submit a form.
            # No CORS headers are supplied; Vite proxies the same-origin API.
            if self.headers.get("Content-Type", "").split(";", 1)[0] != "application/json":
                self.reply(415, {"error": "Expected application/json"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096:
                    raise ValueError("Invalid request size")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict) or data.get("command") not in ("pause", "resume", "inject", "waypoint", "cancel_waypoint", "acknowledge_warning", "emergency_stop","approve_route","select_destination","uav_waypoint","dismiss_warning"):
                    raise ValueError("Unknown command")
                allowed = ({"waypoint": ("command", "x", "y"),
                            "uav_waypoint": ("command", "x", "y", "z"),
                            "dismiss_warning": ("command", "warning_id"),
                            "select_destination": ("command", "destination", "via"),
                            "inject": ("command", "hazard_id"),
                            "emergency_stop": ("command", "enabled"),
                            "approve_route": ("command","revision"),
                            "acknowledge_warning": ("command", "hazard_id", "warning_id")}
                           .get(data["command"], ("command",)))
                if any(key not in allowed for key in data):
                    raise ValueError("Unexpected command field")
                if data["command"] == "emergency_stop" and not isinstance(data.get("enabled"), bool):
                    raise ValueError("enabled must be a boolean")
                if data['command']=='select_destination':
                    for key in ('destination','via'):
                        if key=='via' and key not in data:continue
                        value=data.get(key)
                        if not isinstance(value,str) or not 1<=len(value)<=64 or not value.replace('_','').isalnum():
                            raise ValueError('Invalid facility waypoint or corridor ID')
                if data['command']=='approve_route' and (type(data.get('revision')) is not int or data['revision']<1):
                    raise ValueError('A positive route revision is required')
                if data["command"] in ("waypoint", "uav_waypoint"):
                    if any(not isinstance(data.get(key), (int, float)) or isinstance(data.get(key), bool)
                           or not math.isfinite(data[key]) or abs(data[key]) > 100 for key in ("x", "y")):
                        raise ValueError("Waypoint coordinates must be finite and within 100 metres")
                if data["command"] == "uav_waypoint" and (type(data.get("z")) not in (int,float) or not 6 <= data["z"] <= 12 or not -5 <= data["x"] <= 20 or not -6 <= data["y"] <= 12):
                    raise ValueError("UAV waypoint outside scripted controller bounds")
                if "warning_id" in data or data["command"] == "dismiss_warning":
                    if not isinstance(data.get("warning_id"),str) or not 1<=len(data["warning_id"])<=128:
                        raise ValueError("Invalid warning ID")
                if "hazard_id" in data or data["command"] == "acknowledge_warning":
                    hazard_id = data.get("hazard_id")
                    if (not isinstance(hazard_id, str) or not 1 <= len(hazard_id) <= 128
                            or not hazard_id.strip() or any(ord(c) < 32 or ord(c) == 127 for c in hazard_id)):
                        raise ValueError("Invalid hazard ID")
                snapshot = store.snapshot()
                seen = any(t in snapshot["topics"] for t in ("/clock", "/robot/odom", "/human/odom"))
                # Resuming is possible after a paused simulation stops its clock.
                if not snapshot["connected"] and not (data["command"] in ("resume", "dismiss_warning", "acknowledge_warning") and seen) and data["command"] != "emergency_stop":
                    self.reply(409, {"error": "No current simulation telemetry"})
                    return
                target = emergency_commands if data["command"] == "emergency_stop" and emergency_commands is not None else commands
                target.put_nowait(data)
            except queue.Full:
                self.reply(429, {"error": "Command queue full"})
                return
            except (ValueError, json.JSONDecodeError):
                self.reply(400, {"error": "Invalid control request"})
                return
            note = ("Await the ROS service result and confirmed emergency-stop latch telemetry"
                    if data["command"] == "emergency_stop" else "Await scenario event for execution result")
            self.reply(202, {"queued": data, "note": note})

    return Handler


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


def main(args=None):
    import signal
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String, Bool
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, LaserScan, Imu
    from rosgraph_msgs.msg import Clock

    class DashboardNode(Node):
        def __init__(self):
            super().__init__("dashboard_server")
            self.declare_parameter("port", 8080)
            self.declare_parameter("host", "0.0.0.0")
            self.store = TelemetryStore()
            self.uav_store = TelemetryStore()
            self.commands = queue.Queue(maxsize=32)
            self.emergency_commands = queue.Queue(maxsize=8)
            self.control = self.create_publisher(String, "/hri/control", 10)
            self.receipts = self.create_publisher(String, "/hri/operator_receipt", 10)
            self.uav_goal = self.create_publisher(PoseStamped, "/uav/observation_goal", 10)
            self.emergency_client = self.create_client(SetBool, "/robot/emergency_stop")
            self.emergency = EmergencyStopController(
                self.store, self.emergency_client, lambda enabled: SetBool.Request(data=enabled))
            self.subscriptions_owned = []
            latch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscriptions_owned.append(self.create_subscription(
                Bool, "/robot/emergency_stop_active", lambda msg: self.store.update(
                    "/robot/emergency_stop_active", {"active": bool(msg.data), "stamp":self.get_clock().now().nanoseconds/1e9, "timestamp_basis":"gateway receipt in ROS simulation time"}), latch_qos))
            for topic in JSON_TOPICS:
                self.subscriptions_owned.append(self.create_subscription(
                    String, topic, lambda msg, t=topic: self.on_json(t, msg), 10))
            for entity in ("robot", "human", "uav"):
                topic = f"/{entity}/odom"
                self.subscriptions_owned.append(self.create_subscription(
                    Odometry, topic, lambda msg, t=topic: self.on_odom(t, msg), qos_profile_sensor_data))
            self.subscriptions_owned.append(self.create_subscription(
                Clock, "/clock", lambda msg: self.store.update("/clock", {
                    "seconds": msg.clock.sec + msg.clock.nanosec / 1e9}), qos_profile_sensor_data))
            self.subscriptions_owned.append(self.create_subscription(
                LaserScan, "/robot/scan", self.on_scan, qos_profile_sensor_data))
            self.subscriptions_owned.append(self.create_subscription(
                Image, "/robot/camera/image", self.on_image, qos_profile_sensor_data))
            self.subscriptions_owned.append(self.create_subscription(
                Image, "/uav/camera/image", self.on_uav_image, qos_profile_sensor_data))
            self.subscriptions_owned.append(self.create_subscription(
                Imu, '/robot/imu', lambda msg:self.store.update('/robot/imu', {'stamp':stamp(msg.header),
                    'frame_id':msg.header.frame_id, 'source':'Simulated inertial sensor'}), qos_profile_sensor_data))
            # A wall clock keeps controls responsive while /clock is paused.
            from rclpy.clock import Clock as ROSClock
            from rclpy.clock import ClockType
            self.timer = self.create_timer(0.1, self.drain, clock=ROSClock(clock_type=ClockType.STEADY_TIME))
            self.http = ThreadingHTTPServer((self.get_parameter("host").value,
                                            self.get_parameter("port").value),
                                           make_handler(self.store, self.commands, self.emergency_commands, self.uav_store))
            self.http.daemon_threads = True
            self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
            self.thread.start()
            self.get_logger().info(f"Live ROS HTTP gateway on port {self.http.server_port}")
            self.last_image_wall = 0.0

        def on_json(self, topic, msg):
            try:
                if len(msg.data)>2_000_000: raise ValueError("Payload size limit")
                value = validate_payload(topic, json.loads(msg.data))
                if not isinstance(value, dict):
                    raise ValueError("Expected a JSON object")
            except (ValueError, TypeError, AttributeError) as exc:
                self.store.rejected(topic,exc)
                self.get_logger().warning(f"Rejected malformed JSON on {topic}")
                return
            self.store.update(topic, value)

        def on_odom(self, topic, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            self.store.update(topic, {"stamp": stamp(msg.header), "frame_id": msg.header.frame_id,
                                     "child_frame_id": msg.child_frame_id,
                                     "position": [finite(p.x), finite(p.y), finite(p.z)], "yaw": finite(yaw),
                                     "linear_speed": finite(msg.twist.twist.linear.x),
                                     "angular_speed": finite(msg.twist.twist.angular.z)})

        def on_scan(self, msg):
            ranges = [finite(r) for r in msg.ranges]
            valid = [r for r in ranges if r is not None and msg.range_min <= r <= msg.range_max]
            self.store.update("/robot/scan", {"stamp": stamp(msg.header), "frame_id": msg.header.frame_id,
                                             "samples": len(ranges), "valid_returns": len(valid),
                                             "min_range": min(valid) if valid else None,
                                             "range_max": finite(msg.range_max)})

        def on_uav_image(self, msg):
            try:
                self.uav_store.set_camera(image_png(msg.width,msg.height,msg.encoding,msg.step,msg.data))
                self.store.update("/uav/camera/image", {"stamp":stamp(msg.header),"width":msg.width,"height":msg.height})
            except ValueError as exc:
                self.store.update("/uav/camera/image", {"error":str(exc)})

        def on_image(self, msg):
            now = time.monotonic()
            if now - self.last_image_wall < 0.3:
                return
            self.last_image_wall = now
            try:
                png = image_png(msg.width, msg.height, msg.encoding, msg.step, msg.data)
            except ValueError as exc:
                self.store.update("/robot/camera/image", {"error": str(exc)})
                return
            self.store.set_camera(png)
            self.store.update("/robot/camera/image", {"stamp": stamp(msg.header),
                                                     "frame_id": msg.header.frame_id,
                                                     "width": msg.width, "height": msg.height,
                                                     "encoding": msg.encoding})

        def drain(self):
            self.emergency.poll()
            # Stop service requests are independent of a paused simulation clock
            # and are handled before ordinary scenario/navigation commands.
            for _ in range(8):
                try:
                    data = self.emergency_commands.get_nowait()
                except queue.Empty:
                    break
                self.emergency.request(data["enabled"])
            for _ in range(32):
                try:
                    data = self.commands.get_nowait()
                except queue.Empty:
                    break
                if data['command']=='uav_waypoint':
                    goal=PoseStamped();goal.header.frame_id='world';goal.header.stamp=self.get_clock().now().to_msg()
                    goal.pose.position.x=float(data['x']);goal.pose.position.y=float(data['y']);goal.pose.position.z=float(data['z']);goal.pose.orientation.w=1.
                    self.uav_goal.publish(goal)
                elif data['command'] in ('acknowledge_warning','dismiss_warning') and 'warning_id' in data:
                    now=self.get_clock().now().nanoseconds/1e9
                    try:
                        warning=self.store.warning_action(data['warning_id'],data['command'],now)
                        receipt={'stamp':now,'warning_id':str(warning['id']),'hazard_id':warning['hazard_id'],
                            'action':data['command'],'source':'dashboard_supervisor',
                            'meaning':'Supervisor receipt only; does not establish crew knowledge or clear a hazard'}
                        self.receipts.publish(String(data=json.dumps(receipt)))
                        if data['command']=='acknowledge_warning': self.control.publish(String(data=json.dumps(data)))
                    except ValueError as exc:self.store.rejected('/api/control',exc)
                else:self.control.publish(String(data=json.dumps(data)))

        def close(self):
            self.http.stopping = True
            self.http.shutdown()
            self.http.server_close()
            self.thread.join(timeout=2)

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DashboardNode()
    stopping=False
    def request_stop(*_):
        nonlocal stopping
        stopping=True
    previous={s:signal.signal(s,request_stop) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        while rclpy.ok() and not stopping:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s,handler in previous.items():signal.signal(s,handler)


if __name__ == "__main__":
    main()
