"""Thread-safe telemetry and PNG encoding; importable without ROS."""

import copy
import math
import struct
import threading
import time
import zlib
import uuid
from collections import deque
from .contracts import provenance


def finite(value):
    """JSON cannot represent a lidar infinity; absent return is explicit null."""
    return float(value) if math.isfinite(value) else None


def image_png(width, height, encoding, step, data):
    """Encode real ROS image pixels. Respect row padding and channel ordering."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    if encoding not in channels:
        raise ValueError(f"Unsupported ROS image encoding: {encoding}")
    if not 0 < width <= 8192 or not 0 < height <= 8192:
        raise ValueError("Invalid image dimensions")
    stride = width * channels[encoding]
    if step < stride or len(data) < step * height:
        raise ValueError("Image payload is shorter than declared dimensions")
    raw = bytearray()
    data = bytes(data)
    for row in range(height):
        raw.append(0)  # PNG scanline filter: none
        pixels = bytearray(data[row * step:row * step + stride])
        if encoding.startswith("bgr"):
            for i in range(0, stride, channels[encoding]):
                pixels[i], pixels[i + 2] = pixels[i + 2], pixels[i]
        raw.extend(pixels)

    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))

    color = {1: 0, 3: 2, 4: 6}[channels[encoding]]
    header = struct.pack(">IIBBBBB", width, height, 8, color, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b"")


class TelemetryStore:
    def __init__(self, stale_seconds=5.0, clock=time.monotonic, wall_clock=time.time):
        self._clock = clock
        self._wall_clock = wall_clock
        self._lock = threading.Lock()
        self._topics = {}
        self._observed_hazards = {}
        self._operator_beliefs = {}
        self._timeline = deque(maxlen=150)
        self._camera = None
        self._camera_received = None
        self.epoch = str(uuid.uuid4())
        self._warnings = {}
        self._validation_errors = deque(maxlen=20)
        self.stale_seconds = stale_seconds

    def update(self, topic, data):
        with self._lock:
            now = self._clock()
            wall = self._wall_clock()
            if topic == '/hri/warning' and data.get('id') is not None:
                key=str(data['id'])
                self._warnings.setdefault(key, {'warning':copy.deepcopy(data), 'acknowledged':False, 'dismissed':False})
                if len(self._warnings)>200: self._warnings.pop(next(iter(self._warnings)))
            if topic == "/hri/operator_belief" and isinstance(data, dict):
                hazard_id = data.get("hazard_id")
                if isinstance(hazard_id, str) and hazard_id:
                    self._operator_beliefs[hazard_id] = {
                        "data": copy.deepcopy(data), "monotonic": now, "received_at": wall,
                    }
            if topic == "/hri/observations" and isinstance(data, dict):
                for hazard in data.get("hazards", []):
                    if not isinstance(hazard, dict) or not isinstance(hazard.get("id"), str):
                        continue
                    self._observed_hazards[hazard["id"]] = {
                        "data": {**copy.deepcopy(hazard), "source": data.get("source"),
                                 "last_seen_stamp": data.get("stamp"), "last_seen_received_at": wall},
                        "monotonic": now,
                    }
                data = {**data, "hazards": [item["data"] for item in self._observed_hazards.values()]}
            if topic == "/hri/status" and isinstance(data, dict):
                # Several ROS nodes report status on this public topic. Preserve
                # source-specific metadata across updates without fabricating it.
                prior = self._topics.get(topic, {}).get("data", {})
                sources = copy.deepcopy(prior.get("sources", {})) if isinstance(prior, dict) else {}
                source = data.get("source")
                if isinstance(source, str):
                    sources[source] = copy.deepcopy(data)
                data = {**data, "sources": sources}
            self._topics[topic] = {"data": copy.deepcopy(data), "received_at": wall, "monotonic": now}
            if topic in ("/hri/events", "/hri/warning", "/hri/navigation", "/hri/operator_warning", "/dashboard/emergency_stop_result"):
                self._timeline.append({"topic": topic, "received_at": wall, "data": copy.deepcopy(data)})

    def warning_action(self, identifier, action, sim_stamp):
        with self._lock:
            entry=self._warnings.get(str(identifier))
            if entry is None: raise ValueError('Unknown warning ID')
            entry['acknowledged' if action=='acknowledge_warning' else 'dismissed']=True
            entry['operator_stamp']=sim_stamp
            return copy.deepcopy(entry['warning'])

    def rejected(self, topic, reason):
        with self._lock:
            self._validation_errors.append({'topic':topic,'reason':str(reason),'received_at':self._wall_clock()})

    def set_camera(self, png):
        with self._lock:
            self._camera = bytes(png)
            self._camera_received = self._clock()

    def camera(self):
        with self._lock:
            if self._camera_received is None or self._clock() - self._camera_received > self.stale_seconds:
                return None
            return self._camera

    def snapshot(self):
        with self._lock:
            now = self._clock()
            topics = {
                topic: {"data": copy.deepcopy(entry["data"]), "received_at": entry["received_at"],
                        "age_seconds": max(0.0, now - entry["monotonic"]),
                        "source": "simulation_truth" if topic in ("/hri/warning","/hri/decision") and entry["data"].get("policy")=="oracle" else provenance(topic), "sim_stamp":entry['data'].get('stamp',entry['data'].get('seconds')),
                        "stale": now - entry["monotonic"] > self.stale_seconds}
                for topic, entry in self._topics.items()
            }
            if "/hri/observations" in topics:
                # Missing from a later camera frame means unobserved, not clear.
                topics["/hri/observations"]["data"]["hazards"] = [
                    {**copy.deepcopy(item["data"]),
                     "observation_age_seconds": max(0.0, now-item["monotonic"]),
                     "observation_stale": now-item["monotonic"] > self.stale_seconds}
                    for item in self._observed_hazards.values()
                ]
            if "/hri/operator_belief" in topics:
                # Independent incident posteriors share one ROS topic. Retain
                # each by ID so the UI cannot pair one hazard's warning with
                # another hazard's belief or acknowledgment state.
                topics["/hri/operator_belief"]["data"]["beliefs"] = {
                    hazard_id: {**copy.deepcopy(item["data"]),
                                "last_update_received_at": item["received_at"],
                                "last_update_age_seconds": max(0.0, now-item["monotonic"]),
                                "last_update_stale": now-item["monotonic"] > self.stale_seconds}
                    for hazard_id, item in self._operator_beliefs.items()
                }
            # A server alone is not proof of a running simulation.
            connected = any(topic in topics and not topics[topic]["stale"]
                            for topic in ("/clock", "/robot/odom", "/human/odom"))
            return {"schema_version":1, "epoch":self.epoch, "connected": connected, "updated_at": self._wall_clock(),
                    "topics": topics, "timeline": copy.deepcopy(list(self._timeline)),
                    "warnings":copy.deepcopy(list(self._warnings.values())), "validation_errors":list(self._validation_errors)}
