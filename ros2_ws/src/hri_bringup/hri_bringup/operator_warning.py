"""ROS integration for an actual waypoint operator, separate from responder trials."""
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path as NavPath
from ament_index_python.packages import get_package_share_directory
from .operator_model import OperatorModel


class OperatorWarning(Node):
    def __init__(self):
        super().__init__('operator_warning')
        self.declare_parameter('scenario', 'industrial')
        scenario = self.get_parameter('scenario').value
        config = Path(get_package_share_directory('hri_sim')) / 'config' / f'{scenario}.json'
        self.model = OperatorModel(json.loads(config.read_text()))
        self.publishers_by_topic = {}
        self.warning_count = 0
        self.goal_stamp = 0.
        self.create_subscription(Odometry,'/robot/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(NavPath,'/plan',self.plan,10)
        for topic, callback in [('/hri/observations',self.observe),('/hri/navigation',self.navigation),
                                ('/hri/control',self.control)]:
            self.create_subscription(String,topic,lambda m,c=callback:self.receive(m,c),10)
        self.create_timer(1.0,self.tick)

    def stamp(self):
        return self.get_clock().now().nanoseconds/1e9

    def publish(self, topic, data):
        if topic not in self.publishers_by_topic:
            self.publishers_by_topic[topic] = self.create_publisher(String,topic,10)
        self.publishers_by_topic[topic].publish(String(data=json.dumps({'stamp':self.stamp(),**data},allow_nan=False)))

    def receive(self,msg,callback):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):
                raise ValueError('expected object')
            callback(data)
        except (ValueError,KeyError,TypeError) as error:
            self.get_logger().warning(f'Invalid advisory input: {error}')

    def odom(self,msg):
        self.model.position=(msg.pose.pose.position.x,msg.pose.pose.position.y)

    def observe(self,data):
        self.model.observe(data.get('hazards',[]))

    def navigation(self,data):
        if data.get('state')=='requested':
            self.model.request(data['x'],data['y'])
            self.goal_stamp=self.stamp()
        elif data.get('state') in ('succeeded','canceled','failed','unavailable','goal_rejected'):
            self.model.active=False

    def plan(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if msg.header.frame_id=='odom' and stamp>=self.goal_stamp:
            self.model.plan=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]

    def control(self,data):
        if data.get('command')=='acknowledge_warning':
            accepted=self.model.acknowledge(data['hazard_id'])
            self.publish('/hri/events',{'source':'operator','event':'hazard_acknowledged' if accepted else 'unknown_hazard',
                                       'hazard_id':data['hazard_id']})
            self.tick()

    def tick(self):
        beliefs,warnings,route=self.model.evaluate(self.stamp())
        for belief in beliefs:
            self.publish('/hri/operator_belief',belief)
        for warning in warnings:
            self.warning_count+=1
            self.publish('/hri/operator_warning',{'id':self.warning_count,**warning})
        if route:
            self.publish('/hri/operator_routes',route)


def main():
    rclpy.init()
    node=OperatorWarning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
