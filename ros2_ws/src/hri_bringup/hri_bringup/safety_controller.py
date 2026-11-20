"""Wall-clock command gate and latched software emergency stop.

The Gazebo bridge must accept commands only from /robot/cmd_vel_safe. ROS
producers continue to publish /robot/cmd_vel. This is a simulation software stop,
not an independent hardware stop or a guarantee against this process crashing.
"""
from __future__ import annotations
import math
import time

ZERO_COMMAND = (0.0,)*6


class CommandGate:
    """ROS-independent state machine; timestamps are monotonic wall seconds."""
    def __init__(self,timeout=.5,initially_stopped=False):
        if not math.isfinite(timeout) or timeout<=0:
            raise ValueError('Command timeout must be finite and positive')
        self.timeout=timeout
        self.stopped=bool(initially_stopped)
        self.command=ZERO_COMMAND
        self.received_at=None
        self.reason='emergency_stop' if self.stopped else 'awaiting_command'

    def set_stop(self,active):
        self.stopped=bool(active)
        # Release never resurrects a command buffered before/during the stop.
        self.command=ZERO_COMMAND
        self.received_at=None
        self.reason='emergency_stop' if self.stopped else 'awaiting_command'

    def accept(self,values,now):
        try:
            values=tuple(float(value) for value in values)
        except (ValueError,TypeError):
            values=()
        if len(values)!=6 or not all(math.isfinite(v) for v in values) or not math.isfinite(now):
            self.command=ZERO_COMMAND
            self.received_at=None
            self.reason='invalid_command'
            return False
        if self.stopped:
            self.command=ZERO_COMMAND
            self.received_at=None
            self.reason='emergency_stop'
            return False
        self.command=values
        self.received_at=now
        self.reason='forwarding'
        return True

    def output(self,now):
        if self.stopped:
            self.reason='emergency_stop'
            return ZERO_COMMAND
        if self.received_at is None:
            return ZERO_COMMAND
        if not math.isfinite(now) or not 0<=now-self.received_at<self.timeout:
            self.command=ZERO_COMMAND
            self.received_at=None
            self.reason='command_watchdog'
            return ZERO_COMMAND
        return self.command


def main(args=None):
    import signal
    import rclpy
    from rclpy.clock import Clock,ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    class SafetyController(Node):
        def __init__(self):
            super().__init__('robot_safety_controller')
            self.declare_parameter('command_timeout',.5)
            self.declare_parameter('publish_rate',20.0)
            self.declare_parameter('initially_stopped',False)
            self.gate=CommandGate(float(self.get_parameter('command_timeout').value),
                                  self.get_parameter('initially_stopped').value)
            rate=float(self.get_parameter('publish_rate').value)
            if not math.isfinite(rate) or rate<=0:
                raise ValueError('Publish rate must be finite and positive')
            self.output_pub=self.create_publisher(Twist,'/robot/cmd_vel_safe',10)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
            self.active_pub=self.create_publisher(Bool,'/robot/emergency_stop_active',qos)
            self.command_sub=self.create_subscription(Twist,'/robot/cmd_vel',self.receive,1)
            self.create_service(SetBool,'/robot/emergency_stop',self.set_stop)
            # The timer and watchdog both ignore ROS /clock, including pauses.
            self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1/rate,self.tick,clock=self.steady_clock)
            self.tick()

        def publish_command(self,values):
            message=Twist()
            message.linear.x,message.linear.y,message.linear.z=values[:3]
            message.angular.x,message.angular.y,message.angular.z=values[3:]
            self.output_pub.publish(message)

        def receive(self,message):
            values=(message.linear.x,message.linear.y,message.linear.z,
                    message.angular.x,message.angular.y,message.angular.z)
            self.gate.accept(values,time.monotonic())
            self.publish_command(self.gate.output(time.monotonic()))

        def set_stop(self,request,response):
            if not request.data:
                # Twist has no production timestamp. Replace the volatile DDS
                # reader to discard queued pre-release requests, not merely the
                # command already stored in the gate. New publisher samples are
                # required after this reader rejoins; active controllers may resume.
                self.destroy_subscription(self.command_sub)
            self.gate.set_stop(request.data)
            if not request.data:
                self.command_sub=self.create_subscription(Twist,'/robot/cmd_vel',self.receive,1)
            self.tick()  # Enforce zero before acknowledging the service request.
            response.success=True
            response.message=('Emergency stop latched; zero commands enforced.' if request.data else
                              'Emergency stop released; awaiting a fresh command.')
            return response

        def tick(self):
            self.publish_command(self.gate.output(time.monotonic()))
            self.active_pub.publish(Bool(data=self.gate.stopped))

    # Keep the context valid until the finally block has published its stop.
    # rclpy's asynchronous SIGINT context shutdown otherwise races this publish.
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=SafetyController()
    stopping=False
    def request_stop(*_):
        nonlocal stopping
        stopping=True
    previous={s:signal.signal(s,request_stop) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        while rclpy.ok() and not stopping:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        if rclpy.ok():
            node.gate.set_stop(True)
            node.tick()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for s,handler in previous.items():signal.signal(s,handler)
