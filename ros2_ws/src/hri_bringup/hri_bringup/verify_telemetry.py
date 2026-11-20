"""Fail-closed verification of the real simulator/ROS vertical slice.

This sends a short velocity command in the minimal world and independently
checks actual Gazebo world pose, wheel odometry and lidar distance change.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Image
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class Probe(Node):
    def __init__(self):
        super().__init__('telemetry_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.latest = {}
        self.counts = {}
        self.stamps = {}
        self.pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stop_client = self.create_client(SetBool, '/robot/emergency_stop')
        self.stopped = None
        self.safe_command = None
        self.safe_command_count = 0
        self.safe_nonzero_count = 0
        self.safe_command_received = None
        self.mission = None
        self.trajectory = []
        self.phase = 'startup'
        self.create_subscription(Bool, '/robot/emergency_stop_active',
                                 lambda msg: setattr(self, 'stopped', msg.data), 10)
        self.create_subscription(Twist, '/robot/cmd_vel_safe', self.receive_safe_command, 10)
        self.create_subscription(String, '/robot/mission_status', self.receive_mission, 10)
        for name, cls, topic in [
            ('clock', Clock, '/clock'), ('odom', Odometry, '/robot/odom'),
            ('world_pose', Odometry, '/evaluation/robot_pose'),
            ('scan', LaserScan, '/robot/scan'), ('imu', Imu, '/robot/imu'),
            ('image', Image, '/robot/camera/image')]:
            self.create_subscription(cls, topic, lambda msg, n=name: self.receive(n, msg),
                                     qos_profile_sensor_data)

    def receive(self, name, msg):
        self.latest[name] = msg
        self.counts[name] = self.counts.get(name, 0) + 1
        stamp = msg.clock if name == 'clock' else msg.header.stamp
        self.stamps.setdefault(name, []).append(stamp.sec + stamp.nanosec / 1e9)
        self.stamps[name] = self.stamps[name][-200:]
        if name == 'world_pose':
            self.trajectory.append([self.stamps[name][-1], self.phase, *position(msg)])

    def receive_mission(self, msg):
        self.mission = json.loads(msg.data)

    def receive_safe_command(self, msg):
        self.safe_command = msg
        self.safe_command_count += 1
        self.safe_nonzero_count += int(msg.linear.x > 0)
        self.safe_command_received = time.monotonic()

    def simulation_time(self):
        stamp = self.latest['clock'].clock
        return stamp.sec + stamp.nanosec / 1e9

    def run_for(self, seconds, speed=None, timeout=120):
        start = self.simulation_time()
        pose_samples = self.counts.get('world_pose', 0)
        safe_samples = self.safe_command_count
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if speed is not None:
                self.command(speed)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.simulation_time() - start >= seconds:
                assert self.counts.get('world_pose', 0) - pose_samples >= 3, 'physical pose stream stopped'
                assert abs(self.simulation_time() - self.stamps['world_pose'][-1]) < 0.25, 'physical pose stale'
                assert self.safe_command_count - safe_samples >= 2, 'safety command stream stopped'
                assert time.monotonic() - self.safe_command_received < 0.5, 'safety command stale'
                return
        raise AssertionError(f'simulation failed to advance {seconds} seconds')

    def set_stop(self, active):
        assert self.stop_client.wait_for_service(timeout_sec=10), 'emergency-stop service unavailable'
        request = SetBool.Request()
        request.data = active
        future = self.stop_client.call_async(request)
        assert self.spin_until(future.done, 10), 'emergency-stop response timed out'
        response = future.result()
        assert response.success, response.message
        assert self.spin_until(lambda: self.stopped is active, 10), 'stop latch telemetry disagrees'

    def spin_until(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return True
        return False

    def command(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)


def position(msg):
    point = msg.pose.pose.position
    return [point.x, point.y, point.z]


def front_range(scan):
    index = round(-scan.angle_min / scan.angle_increment)
    values = [v for v in scan.ranges[max(0, index-2):index+3] if math.isfinite(v)]
    return min(values) if values else None


def verify(node, timeout):
    required = {'clock', 'odom', 'world_pose', 'scan', 'imu', 'image'}
    if not node.spin_until(lambda: required <= node.latest.keys()
                          and all(node.counts.get(k, 0) >= 3 for k in required), timeout):
        raise AssertionError(f'telemetry missing/insufficient: counts={node.counts}')
    for key in required:
        stamps = node.stamps[key]
        assert stamps[-1] > stamps[0], f'{key} timestamps did not advance'
        assert all(b >= a for a, b in zip(stamps, stamps[1:])), f'{key} time reversed'
    scan = node.latest['scan']
    assert len(scan.ranges) >= 90, 'insufficient lidar samples'
    before_range = front_range(scan)
    assert before_range is not None and 2 < before_range < 7, f'no expected front obstacle: {before_range}'
    image = node.latest['image']
    assert image.width > 0 and image.height > 0 and len(image.data) >= image.height * image.step
    assert len(set(image.data)) > 8, 'camera image has no scene detail'
    imu = node.latest['imu']
    assert all(math.isfinite(v) for v in [imu.linear_acceleration.x, imu.linear_acceleration.y,
                                        imu.linear_acceleration.z, imu.angular_velocity.z])
    before = position(node.latest['world_pose'])
    assert all(math.isfinite(v) for v in before) and abs(before[2]) < 0.1, 'invalid physical robot pose'
    pose = node.latest['world_pose'].pose.pose
    assert abs(sum(v*v for v in [pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w]) - 1) < 0.01
    assert node.latest['odom'].header.frame_id == 'odom'
    assert node.latest['odom'].child_frame_id == 'base_footprint'
    assert node.latest['scan'].header.frame_id == 'laser_frame'
    assert node.latest['image'].header.frame_id == 'camera_optical_frame'
    assert node.spin_until(lambda: all(node.tf_buffer.can_transform('odom', frame, Time())
                                      for frame in ('base_footprint', 'base_link', 'laser_frame',
                                                    'camera_optical_frame')), 10), 'TF tree incomplete'
    transforms = {}
    for frame, expected in [('base_link', (0, 0, .24)), ('laser_frame', (.18, 0, .49)),
                            ('camera_optical_frame', (.41, 0, .41))]:
        transform = node.tf_buffer.lookup_transform('base_footprint', frame, Time()).transform
        actual = [transform.translation.x, transform.translation.y, transform.translation.z]
        assert math.dist(actual, expected) < 1e-6, f'{frame} TF does not match SDF: {actual}'
        transforms[frame] = actual
    clock = node.simulation_time()
    assert all(abs(clock - node.stamps[key][-1]) < 0.6 for key in required), 'sensor/clock timestamps disagree'
    node.phase = 'movement'
    before_odom = position(node.latest['odom'])
    start_sim = node.latest['clock'].clock.sec + node.latest['clock'].clock.nanosec / 1e9
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        node.command(0.35)
        rclpy.spin_once(node, timeout_sec=0.05)
        stamp = node.latest['clock'].clock
        if stamp.sec + stamp.nanosec / 1e9 - start_sim >= 3.0:
            break
    else:
        raise AssertionError('simulation did not advance three seconds during command')
    node.command(0.0)
    node.spin_until(lambda: False, 1.0)
    after = position(node.latest['world_pose'])
    after_odom = position(node.latest['odom'])
    after_range = front_range(node.latest['scan'])
    distance = math.dist(before, after)
    wheel_distance = math.dist(before_odom, after_odom)
    assert 0.3 < distance < 2.0, f'Gazebo robot did not physically travel expected distance: {distance}'
    assert abs(distance - wheel_distance) < 0.3, 'wheel odometry disagrees with physical pose'
    assert after_range is not None and before_range - after_range > 0.2, 'lidar did not observe approach'
    return {'status': 'passed', 'source': 'real Gazebo Harmonic + ros_gz + ROS 2 Jazzy',
            'samples': node.counts, 'physical_displacement_m': distance,
            'odometry_displacement_m': wheel_distance,
            'world_position_before': before, 'world_position_after': after,
            'front_range_before_m': before_range, 'front_range_after_m': after_range,
            'image_width': image.width, 'image_height': image.height,
            'image_encoding': image.encoding, 'sim_time_start': start_sim}


def verify_foundation(node, timeout):
    result = verify(node, timeout)
    result['tf'] = 'odom -> base_footprint -> base_link -> laser_frame / camera_optical_frame'
    node.phase = 'emergency_stop'
    node.run_for(1.0, speed=0.35, timeout=timeout)
    before_stop = position(node.latest['world_pose'])
    node.set_stop(True)
    # Keep issuing a nonzero request: the latch must still output zero.
    node.run_for(2.0, speed=0.35, timeout=timeout)
    settled = position(node.latest['world_pose'])
    node.run_for(1.0, speed=0.35, timeout=timeout)
    stopped_position = position(node.latest['world_pose'])
    assert node.safe_command is not None and node.safe_command.linear.x == 0.0
    stop_distance = math.dist(before_stop, settled)
    stopped_drift = math.dist(settled, stopped_position)
    assert stop_distance < 0.25, f'excessive stopping distance: {stop_distance}'
    assert stopped_drift < 0.02, f'robot moved while stop latched: {stopped_drift}'
    node.set_stop(False)
    node.run_for(0.5, timeout=timeout)
    assert node.safe_command.linear.x == 0, 'release replayed an old motion command'
    for _ in range(3):
        node.set_stop(True)
        node.run_for(0.5, speed=0.35, timeout=timeout)
        released_from = position(node.latest['world_pose'])
        node.set_stop(False)
        node.run_for(0.5, timeout=timeout)
        assert node.safe_command.linear.x == 0, 'release replayed queued requests in repeated check'
        assert math.dist(released_from, position(node.latest['world_pose'])) < 0.02, 'robot moved after release without a new request'
    node.phase = 'watchdog'
    node.run_for(1.0, speed=0.35, timeout=timeout)
    assert node.safe_command.linear.x > 0, 'no active command before watchdog test'
    assert node.spin_until(lambda: node.safe_command.linear.x == 0, 3), 'command watchdog did not stop'
    node.run_for(1.0, timeout=timeout)
    node.phase = 'collision'
    # This deliberately asks for forward motion into the solid target. It tests
    # Gazebo contact dynamics independently of a planner or lidar avoidance rule.
    node.run_for(12.0, speed=0.35, timeout=timeout)
    contact_before = position(node.latest['world_pose'])
    nonzero_before = node.safe_nonzero_count
    node.run_for(2.0, speed=0.35, timeout=timeout)
    contact_after = position(node.latest['world_pose'])
    node.command(0.0)
    assert 4.0 < contact_after[0] < 4.16, f'expected target contact at x=4.1: {contact_after}'
    assert abs(contact_after[1]) < 0.1, f'collision test left obstacle lane: {contact_after}'
    assert math.dist(contact_before, contact_after) < 0.03, 'robot did not remain blocked by obstacle'
    assert node.safe_command.linear.x > 0 and node.safe_nonzero_count - nonzero_before >= 3, 'no sustained forward command at contact'
    assert max(row[2] for row in node.trajectory) < 4.16, 'robot penetrated obstacle'
    result.update(emergency_stop={'stopping_distance_m': stop_distance, 'latched_drift_m': stopped_drift,
                                  'nonzero_requests_blocked': True, 'release_drops_old_command': True,
                                  'release_backlog_checks': 4},
                  command_watchdog='passed', collision={'position': contact_after,
                  'motion_under_forward_command_m': math.dist(contact_before, contact_after),
                  'obstacle_front_x_m': 4.5, 'robot_front_offset_m': 0.4})
    return result


def verify_mission(node, timeout):
    assert node.spin_until(lambda: node.mission is not None and
                           node.mission.get('state') in ('completed', 'failed'), timeout), 'mission timed out'
    assert node.mission['state'] == 'completed', f'mission failed: {node.mission}'
    node.run_for(0.7, timeout=timeout)
    assert len(node.trajectory) > 20, 'no physical mission trajectory'
    start, end = node.trajectory[0][2:], node.trajectory[-1][2:]
    assert math.dist(start, end) > 0.8, 'mission did not physically move robot'
    waypoints = node.mission['waypoints']
    assert node.mission['waypoint_index'] == len(waypoints) == 2, 'not all requested waypoints completed'
    assert all(math.dist(actual, expected) < 1e-5 for actual, expected in
               zip(waypoints, ((1, 0), (1, 1)))), 'mission goals differ from commanded test waypoints'
    # Wheel odometry can differ from world pose during turning. Check each goal
    # independently against physical pose, and report the tolerance explicitly.
    distances = []
    remaining = node.trajectory
    for goal in waypoints:
        candidates = [(math.dist(row[2:4], goal), index) for index, row in enumerate(remaining)]
        distance, index = min(candidates)
        assert distance < 0.35, f'physical robot missed waypoint {goal}: {distance}'
        distances.append(distance)
        remaining = remaining[index:]
    assert math.dist(end[:2], waypoints[-1]) < 0.35, 'robot did not finish near final waypoint'
    return {'status': 'passed', 'source': 'actual Gazebo pose and ROS mission status',
            'mission': node.mission, 'world_position_start': start, 'world_position_end': end,
            'physical_samples': len(node.trajectory), 'physical_goal_tolerance_m': 0.35,
            'physical_waypoint_closest_distances_m': distances}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--output', default='results/telemetry.json')
    parser.add_argument('--foundation', action='store_true')
    parser.add_argument('--mission', action='store_true')
    parser.add_argument('--trajectory', default=None)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    result = None
    try:
        result = (verify_mission if args.mission else verify_foundation if args.foundation else verify)(node, args.timeout)
    except Exception as exc:
        result = {'status': 'failed', 'error': str(exc), 'samples': node.counts}
    finally:
        node.command(0.0)
        node.destroy_node()
        rclpy.shutdown()
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + '\n')
    if args.trajectory:
        trajectory = Path(args.trajectory)
        trajectory.parent.mkdir(parents=True, exist_ok=True)
        with trajectory.open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_time_s', 'phase', 'world_x_m', 'world_y_m', 'world_z_m'])
            writer.writerows(node.trajectory)
    print(json.dumps(result, indent=2))
    if result['status'] != 'passed':
        raise SystemExit(1)
