#!/usr/bin/env python3
"""Read-only runtime evidence of the normal inference ROS input boundary."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters,ListParameters

p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();node=Node('audit_tom_boundary')
listing=node.create_client(ListParameters,'/bayesian_tom/list_parameters')
if not listing.wait_for_service(timeout_sec=30): raise RuntimeError('Inference node unavailable')
future=listing.call_async(ListParameters.Request())
rclpy.spin_until_future_complete(node,future,timeout_sec=10)
names=future.result().result.names
getter=node.create_client(GetParameters,'/bayesian_tom/get_parameters')
getter.wait_for_service(timeout_sec=10)
future=getter.call_async(GetParameters.Request(names=['public_map']))
rclpy.spin_until_future_complete(node,future,timeout_sec=10)
path=future.result().values[0].string_value
config=json.loads(Path(path).read_text())
deadline=time.monotonic()+10
subscriptions=[]
while time.monotonic()<deadline:
    if ('bayesian_tom','/') in node.get_node_names_and_namespaces():
        subscriptions=node.get_subscriber_names_and_types_by_node('bayesian_tom','/')
        if len(subscriptions)>=5: break
    rclpy.spin_once(node,timeout_sec=.2)
topics={name for name,_ in subscriptions}
allowed={'/clock','/parameter_events','/human/odom','/hri/human_intent','/hri/observations','/hri/warning'}
passed=topics<=allowed and {'/human/odom','/hri/observations','/hri/human_intent'}<=topics and 'config' not in names and set(config)=={'nodes','edges','start','goal'}
report={'status':'passed' if passed else 'failed','subscriptions':subscriptions,'parameters':names,'public_map':config}
Path(a.output).write_text(json.dumps(report,indent=2)+'\n')
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if passed else 1)
