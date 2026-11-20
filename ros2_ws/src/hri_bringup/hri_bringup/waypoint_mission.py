"""Opt-in minimal-world waypoint mission driven entirely by real ROS telemetry.

This intentionally small demonstration controller is not a Nav2 replacement.
Launch it only when Nav2 / manual command producers are disabled. The live
trajectory records received Gazebo wheel-odometry samples, never inferred poses.
"""
from __future__ import annotations
import csv
import json
import math
from pathlib import Path
import time


def parse_waypoints(value):
    points=json.loads(value) if isinstance(value,str) else value
    if not isinstance(points,list) or not points:
        raise ValueError('Waypoints must be a nonempty JSON array of [x,y] pairs')
    parsed=[]
    for point in points:
        if (not isinstance(point,(list,tuple)) or len(point)!=2
            or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in point)):
            raise ValueError('Each waypoint must contain two finite numeric coordinates')
        parsed.append((float(point[0]),float(point[1])))
    return parsed


def front_clearance(ranges,angle_min,angle_increment,range_min,range_max,half_angle=.45):
    """Conservative forward scan reduction; NaNs / missing beams are unknown."""
    if (not all(math.isfinite(v) for v in (angle_min,angle_increment,range_min,range_max,half_angle))
        or angle_increment==0 or range_min<0 or range_max<=range_min or half_angle<=0):
        return None
    seen=[]
    for index,value in enumerate(ranges):
        angle=angle_min+index*angle_increment
        angle=math.atan2(math.sin(angle),math.cos(angle))
        if abs(angle)>half_angle:continue
        if value==math.inf:
            seen.append(range_max)
        elif math.isfinite(value) and value>=0:
            seen.append(0.0 if value<range_min else min(value,range_max))
    return min(seen) if seen else None


class WaypointFollower:
    def __init__(self,points,relative=True,tolerance=.12,linear_speed=.35,angular_speed=1.0,stop_distance=.65):
        self.requested=parse_waypoints(points)
        if not all(math.isfinite(v) and v>0 for v in (tolerance,linear_speed,angular_speed,stop_distance)):
            raise ValueError('Mission tolerances and speed limits must be finite and positive')
        self.relative=relative
        self.tolerance,self.linear_speed,self.angular_speed,self.stop_distance=tolerance,linear_speed,angular_speed,stop_distance
        self.points=None
        self.index=0
        self.completed=False

    def anchor(self,pose):
        if self.points is not None:return
        x,y,yaw=pose
        if not all(math.isfinite(v) for v in pose):raise ValueError('Pose must be finite')
        self.points=([(x+math.cos(yaw)*px-math.sin(yaw)*py,
                       y+math.sin(yaw)*px+math.cos(yaw)*py) for px,py in self.requested]
                     if self.relative else list(self.requested))

    def step(self,pose,clearance):
        if not all(math.isfinite(v) for v in pose):return 0.0,0.0,'invalid_odometry'
        self.anchor(pose)
        x,y,yaw=pose
        while self.index<len(self.points) and math.dist((x,y),self.points[self.index])<=self.tolerance:
            self.index+=1
        if self.index>=len(self.points):
            self.completed=True
            return 0.0,0.0,'completed'
        if clearance is None or math.isnan(clearance) or clearance<0:
            return 0.0,0.0,'waiting_for_scan'
        if clearance<self.stop_distance:
            return 0.0,0.0,'obstacle_stop'
        tx,ty=self.points[self.index]
        distance=math.hypot(tx-x,ty-y)
        error=math.atan2(math.sin(math.atan2(ty-y,tx-x)-yaw),math.cos(math.atan2(ty-y,tx-x)-yaw))
        angular=max(-self.angular_speed,min(self.angular_speed,1.8*error))
        linear=min(self.linear_speed,.8*distance)*max(0.0,math.cos(error)) if abs(error)<.6 else 0.0
        return linear,angular,'running'


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String,Bool

    class WaypointMission(Node):
        def __init__(self):
            super().__init__('waypoint_mission')
            self.set_parameters([Parameter('use_sim_time',value=True)])
            defaults={'waypoints_json':'[[1.0,0.0],[1.0,1.0]]','relative_to_start':True,
                      'goal_tolerance':.12,'linear_speed':.35,'angular_speed':1.0,
                      'obstacle_stop_distance':.65,'scan_half_angle':.45,'sensor_timeout':1.0,
                      'receipt_timeout':2.0,'mission_timeout':120.0,'control_rate':20.0,
                      'trajectory_path':'results/waypoint_trajectory.csv','summary_path':''}
            for name,value in defaults.items():self.declare_parameter(name,value)
            self.options={name:self.get_parameter(name).value for name in defaults}
            for name in ('scan_half_angle','sensor_timeout','receipt_timeout','mission_timeout','control_rate'):
                if not math.isfinite(self.options[name]) or self.options[name]<=0:
                    raise ValueError(f'{name} must be finite and positive')
            self.follower=WaypointFollower(self.options['waypoints_json'],self.options['relative_to_start'],
                self.options['goal_tolerance'],self.options['linear_speed'],self.options['angular_speed'],
                self.options['obstacle_stop_distance'])
            self.pose=None
            self.odom_stamp=self.scan_stamp=None
            self.odom_received=self.scan_received=self.guard_received=None
            self.clearance=None
            self.stop_active=None
            self.started=None
            self.finished=None
            self.path_length=0.0
            self.last_position=None
            self.samples=0
            path=Path(self.options['trajectory_path'])
            path.parent.mkdir(parents=True,exist_ok=True)
            self.trajectory_path=path
            self.summary_path=Path(self.options['summary_path']) if self.options['summary_path'] else path.with_suffix('.json')
            self.stream=path.open('w',newline='',buffering=1)
            self.writer=csv.writer(self.stream)
            self.writer.writerow(['sim_stamp','x','y','z','yaw','linear_x','linear_y','angular_z','waypoint_index'])
            self.command_pub=self.create_publisher(Twist,'/robot/cmd_vel',10)
            status_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.status_pub=self.create_publisher(String,'/robot/mission_status',status_qos)
            self.create_subscription(Odometry,'/robot/odom',self.on_odom,qos_profile_sensor_data)
            self.create_subscription(LaserScan,'/robot/scan',self.on_scan,qos_profile_sensor_data)
            self.create_subscription(Bool,'/robot/emergency_stop_active',self.on_stop,status_qos)
            self.create_timer(1/self.options['control_rate'],self.tick)

        def stamp(self):return self.get_clock().now().nanoseconds/1e9

        def on_odom(self,message):
            p,q=message.pose.pose.position,message.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            if not all(math.isfinite(v) for v in (p.x,p.y,p.z,yaw)):
                self.pose=None
                return
            self.pose=(p.x,p.y,yaw)
            self.odom_stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
            self.odom_received=time.monotonic()
            self.follower.anchor(self.pose)
            position=(p.x,p.y)
            if self.last_position is not None:self.path_length+=math.dist(position,self.last_position)
            self.last_position=position
            velocity=message.twist.twist
            self.writer.writerow([self.odom_stamp,p.x,p.y,p.z,yaw,velocity.linear.x,velocity.linear.y,
                                  velocity.angular.z,self.follower.index])
            self.samples+=1

        def on_scan(self,message):
            self.clearance=front_clearance(message.ranges,message.angle_min,message.angle_increment,
                message.range_min,message.range_max,self.options['scan_half_angle'])
            self.scan_stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
            self.scan_received=time.monotonic()

        def on_stop(self,message):
            self.stop_active=message.data
            self.guard_received=time.monotonic()

        def fresh(self,stamp,received):
            return (stamp is not None and received is not None and -.1<=self.stamp()-stamp<=self.options['sensor_timeout']
                    and time.monotonic()-received<=self.options['receipt_timeout'])

        def publish_status(self,state):
            payload={'stamp':self.stamp(),'state':'failed' if state=='timed_out' else state,
                     'reason':'mission_timeout' if state=='timed_out' else state,'completed':self.follower.completed,
                     'waypoint_index':self.follower.index,'waypoints':self.follower.points or [],
                     'position':list(self.pose[:2]) if self.pose else None,'source_pose_stamp':self.odom_stamp,
                     'front_clearance':self.clearance if self.clearance is not None and math.isfinite(self.clearance) else None,
                     'pose_source':'/robot/odom','trajectory_path':str(self.trajectory_path),
                     'measured_path_length_m':self.path_length,'odom_samples':self.samples,
                     'elapsed_s':None if self.started is None else self.stamp()-self.started}
            self.status_pub.publish(String(data=json.dumps(payload,allow_nan=False)))
            if state in ('completed','timed_out') and self.finished is None:
                self.finished=state
                self.summary_path.parent.mkdir(parents=True,exist_ok=True)
                self.summary_path.write_text(json.dumps(payload,indent=2,allow_nan=False)+'\n')
                self.get_logger().info(f'Waypoint mission {state}; evidence: {self.summary_path}')

        def tick(self):
            linear=angular=0.0
            if self.finished is not None:
                state=self.finished
            elif self.guard_received is None or time.monotonic()-self.guard_received>self.options['receipt_timeout']:
                state='waiting_for_safety_controller'
            elif self.stop_active:
                state='emergency_stopped'
            elif self.pose is None or not self.fresh(self.odom_stamp,self.odom_received):
                state='waiting_for_odometry'
            elif not self.fresh(self.scan_stamp,self.scan_received):
                state='waiting_for_scan'
            else:
                if self.started is None:self.started=self.stamp()
                if self.stamp()-self.started>self.options['mission_timeout']:
                    state='timed_out'
                else:
                    linear,angular,state=self.follower.step(self.pose,self.clearance)
            command=Twist()
            command.linear.x=linear
            command.angular.z=angular
            self.command_pub.publish(command)
            self.publish_status(state)

        def close(self):
            if rclpy.ok():self.command_pub.publish(Twist())
            self.stream.close()

    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=WaypointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
