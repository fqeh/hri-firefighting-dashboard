"""Report the assigned lookout mission and freshness of actual UAV telemetry."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import LogicalCameraImage
from std_msgs.msg import String


class MissionMonitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        self.position = None
        self.pose_stamp = None
        self.detection_stamp = None
        self.pub = self.create_publisher(String, '/hri/uav_status', 10)
        self.create_subscription(Odometry, '/uav/odom', self.pose, qos_profile_sensor_data)
        self.create_subscription(LogicalCameraImage, '/uav/detections', self.detection, qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)

    def pose(self, msg):
        p=msg.pose.pose.position
        self.position=[p.x,p.y,p.z]
        self.pose_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def detection(self, msg):
        self.detection_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9
        pose_fresh=self.pose_stamp is not None and 0<=now-self.pose_stamp<2
        sensor_fresh=self.detection_stamp is not None and 0<=now-self.detection_stamp<2
        state='observing' if pose_fresh and sensor_fresh else ('waiting_for_sensor' if pose_fresh else 'telemetry_stale')
        self.pub.publish(String(data=json.dumps({'stamp':now,'state':state,'mission':'stationary_lookout',
            'constraint':'fixed hover; no autonomous flight controller','position':self.position,
            'pose_stamp':self.pose_stamp,'detection_stamp':self.detection_stamp,
            'pose_fresh':pose_fresh,'sensor_fresh':sensor_fresh},allow_nan=False)))


def main():
    rclpy.init()
    node=MissionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
