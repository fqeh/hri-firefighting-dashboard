"""Finish ROS callbacks before shutting down the context or raising an interrupt."""
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions


def run(cls):
    # Raising KeyboardInterrupt inside pybind11 take_message can surface as a
    # conversion RuntimeError. Signals request exit; they never interrupt a take.
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    previous = {s: signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}
    node = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = cls()
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in previous.items():
            signal.signal(s, handler)
