#!/usr/bin/env python3
"""Audit deployed warning, inference and routing inputs without reading belief truth."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters,ListParameters

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--oracle',action='store_true');a=p.parse_args()
rclpy.init();node=Node('audit_warning_boundary');report={}
allowed={
    'bayesian_tom':{'/clock','/parameter_events','/human/odom','/hri/human_intent','/hri/observations','/hri/warning'},
    'warning_decision':{'/clock','/parameter_events','/hri/observations','/hri/belief','/hri/human_intent','/hri/rescue_route','/hri/acknowledgement'},
    'rescue_routing':{'/clock','/parameter_events','/hri/observations','/hri/route_observations','/human/odom','/robot/odom'}}
if a.oracle:allowed['warning_decision'].add('/evaluation/human_belief')
for name,topics_allowed in allowed.items():
    listing=node.create_client(ListParameters,f'/{name}/list_parameters')
    if not listing.wait_for_service(timeout_sec=25):raise RuntimeError(f'{name} is unavailable')
    future=listing.call_async(ListParameters.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10)
    names=future.result().result.names
    getter=node.create_client(GetParameters,f'/{name}/get_parameters');getter.wait_for_service(timeout_sec=10)
    future=getter.call_async(GetParameters.Request(names=['public_map']));rclpy.spin_until_future_complete(node,future,timeout_sec=10)
    config=json.loads(Path(future.result().values[0].string_value).read_text())
    until=time.monotonic()+10;subscriptions=[]
    while time.monotonic()<until:
        subscriptions=node.get_subscriber_names_and_types_by_node(name,'/')
        if len(subscriptions)>=len(topics_allowed)-1:break
        rclpy.spin_once(node,timeout_sec=.2)
    topics={topic for topic,_ in subscriptions}
    passed=(topics<=topics_allowed and '/hri/observations' in topics and 'config' not in names
        and set(config)=={'nodes','edges','start','goal'})
    if name=='warning_decision':passed=passed and (('/evaluation/human_belief' in topics)==a.oracle)
    report[name]={'passed':passed,'subscriptions':subscriptions,'parameters':names,'public_map':config}
passed=all(n['passed'] for n in report.values());report={'status':'passed' if passed else 'failed','oracle':a.oracle,'nodes':report}
Path(a.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown();raise SystemExit(0 if passed else 1)
