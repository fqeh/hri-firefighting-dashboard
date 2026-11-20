import http.client
import json
import math
from pathlib import Path
import queue
import struct
import sys
import threading
import unittest
import zlib
from concurrent.futures import Future
from types import SimpleNamespace
from http.server import ThreadingHTTPServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hri_dashboard.state import TelemetryStore, finite, image_png
from hri_dashboard.server import EmergencyStopController, make_handler


class GatewayTests(unittest.TestCase):
    def test_emergency_service_result_does_not_invent_confirmed_latch_state(self):
        store = TelemetryStore()
        future = Future()
        requests = []
        client = SimpleNamespace(service_is_ready=lambda: True,
                                 call_async=lambda request: requests.append(request) or future)
        control = EmergencyStopController(store, client, lambda enabled: SimpleNamespace(data=enabled))
        control.request(True)
        self.assertTrue(requests[0].data)
        snapshot = store.snapshot()
        self.assertEqual(snapshot["topics"]["/dashboard/emergency_stop_result"]["data"]["state"], "requesting")
        self.assertNotIn("/robot/emergency_stop_active", snapshot["topics"])
        future.set_result(SimpleNamespace(success=True, message="Guard latched"))
        snapshot = store.snapshot()
        self.assertTrue(snapshot["topics"]["/dashboard/emergency_stop_result"]["data"]["success"])
        self.assertNotIn("/robot/emergency_stop_active", snapshot["topics"])
        store.update("/robot/emergency_stop_active", {"active": True})
        self.assertTrue(store.snapshot()["topics"]["/robot/emergency_stop_active"]["data"]["active"])

    def test_emergency_service_failure_and_timeout_preserve_observed_latch(self):
        now = [1.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.update("/robot/emergency_stop_active", {"active": True})
        client = SimpleNamespace(service_is_ready=lambda: False, call_async=lambda request: None)
        control = EmergencyStopController(store, client, lambda enabled: SimpleNamespace(data=enabled), clock=lambda: now[0])
        control.request(False)
        self.assertEqual(store.snapshot()["topics"]["/dashboard/emergency_stop_result"]["data"]["state"], "unavailable")
        future = Future()
        client.service_is_ready = lambda: True
        client.call_async = lambda request: future
        control.request(False)
        future.set_exception(RuntimeError("Service connection lost"))
        self.assertEqual(store.snapshot()["topics"]["/dashboard/emergency_stop_result"]["data"]["state"], "failed")
        future = Future()
        control.request(False)
        now[0] = 8.0
        control.poll()
        snapshot = store.snapshot()
        self.assertEqual(snapshot["topics"]["/dashboard/emergency_stop_result"]["data"]["state"], "response_timeout")
        self.assertTrue(snapshot["topics"]["/robot/emergency_stop_active"]["data"]["active"])
        self.assertTrue(snapshot["topics"]["/robot/emergency_stop_active"]["stale"])

    def test_emergency_service_reports_release_rejection_and_ignores_old_response(self):
        store, futures = TelemetryStore(), []
        def call(request):
            future = Future()
            futures.append((request, future))
            return future
        client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
        control = EmergencyStopController(store, client, lambda enabled: SimpleNamespace(data=enabled))
        control.request(False)
        self.assertFalse(futures[0][0].data)
        futures[0][1].set_result(SimpleNamespace(success=False, message="Release refused"))
        self.assertEqual(store.snapshot()["topics"]["/dashboard/emergency_stop_result"]["data"]["state"], "rejected")
        control.request(False)
        control.request(True)
        futures[2][1].set_result(SimpleNamespace(success=True, message="Stop applied"))
        futures[1][1].set_result(SimpleNamespace(success=True, message="Earlier release response"))
        result = store.snapshot()["topics"]["/dashboard/emergency_stop_result"]["data"]
        self.assertTrue(result["requested_enabled"])
        self.assertEqual(result["message"], "Stop applied")

    def test_http_emergency_control_requires_boolean_and_works_with_stale_simulation(self):
        store, commands, emergency = TelemetryStore(), queue.Queue(), queue.Queue()
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(store, commands, emergency))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        headers = {"Content-Type": "application/json"}
        def post(payload):
            connection.request("POST", "/api/control", json.dumps(payload), headers)
            response = connection.getresponse()
            body = json.loads(response.read())
            return response.status, body
        try:
            for enabled in (True, False):
                status, body = post({"command": "emergency_stop", "enabled": enabled})
                self.assertEqual(status, 202)
                self.assertIn("queued", body)
                self.assertEqual(emergency.get_nowait(), {"command": "emergency_stop", "enabled": enabled})
                self.assertNotIn("/robot/emergency_stop_active", store.snapshot()["topics"])
            for enabled in (None, 0, 1, "false", "true", [], {}):
                with self.subTest(enabled=enabled):
                    self.assertEqual(post({"command": "emergency_stop", "enabled": enabled})[0], 400)
            self.assertEqual(post({"command": "emergency_stop"})[0], 400)
            self.assertEqual(post({"command": "emergency_stop", "enabled": False, "hazard_id": "x"})[0], 400)
            self.assertTrue(commands.empty())
            self.assertTrue(emergency.empty())
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_observed_telemetry_expires_without_inventing_a_pose(self):
        now = [1.0]
        store = TelemetryStore(clock=lambda: now[0], wall_clock=lambda: 100.0)
        self.assertFalse(store.snapshot()["connected"])
        self.assertEqual(store.snapshot()["topics"], {})
        store.update("/robot/odom", {"position": [2, 3, 0]})
        snapshot = store.snapshot()
        self.assertTrue(snapshot["connected"])
        snapshot["topics"]["/robot/odom"]["data"]["position"][0] = 99
        self.assertEqual(store.snapshot()["topics"]["/robot/odom"]["data"]["position"][0], 2)
        now[0] += 6
        self.assertFalse(store.snapshot()["connected"])
        self.assertTrue(store.snapshot()["topics"]["/robot/odom"]["stale"])

    def test_png_channel_order_and_padded_rows(self):
        png = image_png(1, 2, "bgr8", 4, bytes([1, 2, 3, 255, 4, 5, 6, 255]))
        self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")
        cursor = 8
        pixels = b""
        while cursor < len(png):
            length = struct.unpack(">I", png[cursor:cursor+4])[0]
            if png[cursor+4:cursor+8] == b"IDAT":
                pixels += png[cursor+8:cursor+8+length]
            cursor += 12 + length
        self.assertEqual(zlib.decompress(pixels), bytes([0, 3, 2, 1, 0, 6, 5, 4]))
        with self.assertRaises(ValueError):
            image_png(2, 2, "rgb8", 6, b"short")
        self.assertIsNone(finite(math.inf))

    def test_stale_camera_is_unavailable(self):
        now = [0.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.set_camera(b"pixels")
        self.assertEqual(store.camera(), b"pixels")
        now[0] = 10
        self.assertIsNone(store.camera())

    def test_status_metadata_is_retained_by_source(self):
        store = TelemetryStore()
        store.update("/hri/status", {"source": "scenario", "scenario": "forest", "seed": 7})
        store.update("/hri/status", {"source": "human", "state": "moving"})
        status = store.snapshot()["topics"]["/hri/status"]["data"]
        self.assertEqual(status["source"], "human")
        self.assertEqual(status["sources"]["scenario"]["seed"], 7)
        self.assertEqual(status["sources"]["human"]["state"], "moving")

    def test_operator_advisory_retains_separate_belief_and_warning(self):
        store = TelemetryStore()
        store.update("/hri/belief", {"posterior": {"unaware": 0.2, "aware": 0.8}})
        store.update("/hri/operator_belief", {"hazard_id": "gas", "posterior": {"unaware": 0.7, "aware": 0.3}})
        store.update("/hri/operator_warning", {"hazard_id": "gas", "action": "explicit"})
        snapshot = store.snapshot()
        self.assertEqual(snapshot["topics"]["/hri/belief"]["data"]["posterior"]["unaware"], 0.2)
        self.assertEqual(snapshot["topics"]["/hri/operator_belief"]["data"]["posterior"]["unaware"], 0.7)
        self.assertEqual(snapshot["timeline"][0]["topic"], "/hri/operator_warning")

    def test_hazard_history_survives_partial_views_and_tracks_last_seen_age(self):
        now = [1.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.update("/hri/observations", {"stamp": 5, "source": "robot", "hazards": [{"id": "gas", "position": [2, 3]}]})
        now[0] = 8
        store.update("/hri/observations", {"stamp": 12, "source": "uav", "hazards": [{"id": "tree", "position": [4, 5]}]})
        hazards = {h["id"]: h for h in store.snapshot()["topics"]["/hri/observations"]["data"]["hazards"]}
        self.assertEqual(set(hazards), {"gas", "tree"})
        self.assertEqual(hazards["gas"]["last_seen_stamp"], 5)
        self.assertEqual(hazards["gas"]["source"], "robot")
        self.assertTrue(hazards["gas"]["observation_stale"])
        self.assertFalse(hazards["tree"]["observation_stale"])
        store.update("/hri/observations", {"stamp": 13, "source": "robot", "hazards": []})
        self.assertEqual(len(store.snapshot()["topics"]["/hri/observations"]["data"]["hazards"]), 2)

    def test_operator_beliefs_are_retained_and_acknowledged_per_hazard(self):
        now = [1.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.update("/hri/operator_belief", {"stamp": 10, "hazard_id": "hazard_fire",
                     "posterior": {"unaware": 0.8, "aware": 0.2}})
        now[0] = 8.0
        store.update("/hri/operator_belief", {"stamp": 17, "hazard_id": "hazard_gas",
                     "posterior": {"unaware": 0.6, "aware": 0.4}})
        topic = store.snapshot()["topics"]["/hri/operator_belief"]["data"]
        self.assertEqual(topic["hazard_id"], "hazard_gas")  # latest-field compatibility
        self.assertEqual(topic["beliefs"]["hazard_fire"]["posterior"]["unaware"], 0.8)
        self.assertTrue(topic["beliefs"]["hazard_fire"]["last_update_stale"])
        self.assertFalse(topic["beliefs"]["hazard_gas"]["last_update_stale"])
        store.update("/hri/operator_belief", {"stamp": 18, "hazard_id": "hazard_fire",
                     "posterior": {"unaware": 0.0, "aware": 1.0}, "acknowledged": True})
        beliefs = store.snapshot()["topics"]["/hri/operator_belief"]["data"]["beliefs"]
        self.assertTrue(beliefs["hazard_fire"]["acknowledged"])
        self.assertEqual(beliefs["hazard_fire"]["posterior"]["aware"], 1.0)
        self.assertEqual(beliefs["hazard_gas"]["posterior"]["unaware"], 0.6)
        self.assertNotIn("acknowledged", beliefs["hazard_gas"])
        beliefs["hazard_fire"]["posterior"]["aware"] = 0.0
        self.assertEqual(store.snapshot()["topics"]["/hri/operator_belief"]["data"]["beliefs"]["hazard_fire"]["posterior"]["aware"], 1.0)

    def test_acknowledgment_requires_bounded_hazard_id_and_live_simulation(self):
        store, commands = TelemetryStore(), queue.Queue()
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(store, commands))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        headers = {"Content-Type": "application/json"}
        def post(payload):
            connection.request("POST", "/api/control", json.dumps(payload), headers)
            response = connection.getresponse()
            response.read()
            return response.status
        try:
            self.assertEqual(post({"command": "acknowledge_warning", "hazard_id": "gas"}), 409)
            store.update("/clock", {"seconds": 2})
            self.assertEqual(post({"command": "acknowledge_warning", "hazard_id": "gas"}), 202)
            self.assertEqual(commands.get_nowait(), {"command": "acknowledge_warning", "hazard_id": "gas"})
            for hazard_id in (None, "", " ", "x" * 129, 123, True, "bad\nname"):
                with self.subTest(hazard_id=hazard_id):
                    self.assertEqual(post({"command": "acknowledge_warning", "hazard_id": hazard_id}), 400)
            self.assertEqual(post({"command": "acknowledge_warning"}), 400)
            self.assertTrue(commands.empty())
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_http_controls_are_validated_and_do_not_claim_execution(self):
        store, commands = TelemetryStore(), queue.Queue()
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(store, commands))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        try:
            connection.request("GET", "/api/state")
            response = connection.getresponse()
            self.assertFalse(json.loads(response.read())["connected"])
            headers = {"Content-Type": "application/json"}
            connection.request("POST", "/api/control", json.dumps({"command": "inject"}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 409)
            response.read()
            store.update("/clock", {"seconds": 1})
            connection.request("POST", "/api/control", json.dumps({"command": "inject"}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 202)
            self.assertIn("queued", json.loads(response.read()))
            self.assertEqual(commands.get_nowait(), {"command": "inject"})
            connection.request("POST", "/api/control", json.dumps({"command": "waypoint", "x": 2, "y": 4}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 202)
            response.read()
            self.assertEqual(commands.get_nowait(), {"command": "waypoint", "x": 2, "y": 4})
            connection.request("POST", "/api/control", json.dumps({"command": "cancel_waypoint"}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 202)
            response.read()
            self.assertEqual(commands.get_nowait(), {"command": "cancel_waypoint"})
            for revision, expected in ((3, 202), (True, 400), (0, 400), (-1, 400), (1.5, 400), ("3", 400), (None, 400)):
                connection.request("POST", "/api/control", json.dumps({"command": "approve_route", "revision": revision}), headers)
                response = connection.getresponse()
                self.assertEqual(response.status, expected)
                response.read()
            self.assertEqual(commands.get_nowait(), {"command": "approve_route", "revision": 3})
            self.assertTrue(commands.empty())

            connection.request("POST", "/api/control", json.dumps({"command": "waypoint", "x": 200, "y": 4}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 400)
            response.read()
            connection.request("POST", "/api/control", json.dumps({"command": "reset"}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 400)
            response.read()
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
