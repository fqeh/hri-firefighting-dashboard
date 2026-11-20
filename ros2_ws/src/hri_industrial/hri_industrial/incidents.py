"""Scheduled generic emergency aftermath; no weapon, combustion or dispersion model."""
import json,math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock,ClockType
from std_msgs.msg import String
from hri_reasoning.runtime import ScenarioManager
from hri_reasoning.forest_runtime import ForestManager
from hri_reasoning.lifecycle import run
from hri_industrial.registry import load_equipment_registry
from hri_industrial.incident_config import execute_tank_damage_aftermath


class IndustrialManager(ScenarioManager):
    def __init__(self):
        super().__init__()
        self.start_time=0.  # Absolute /clock schedule; independent of node discovery latency.
        self.route_visual=False;self.last_route=None
        registry_path=self.declare_parameter('equipment_registry','').value
        world_path=self.declare_parameter('world_file','').value
        if not registry_path or not world_path:
            raise ValueError('equipment_registry and world_file parameters are required')
        self.equipment_registry=load_equipment_registry(registry_path,world_path)
        self.industrial_model_root=Path(get_package_share_directory('hri_industrial'))/'models'
        self.spawned_entities=set()
        self.equipment_pub=self.create_publisher(String,'/hri/equipment_registry',10)
        self.publish_equipment_registry()
        self.subscribe_json('/hri/robot_route',self.display_route)
        self.create_timer(1.,self.hazard_status)
        self.create_timer(1.,self.publish_equipment_registry,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def control(self,data):
        if data.get('command') in ('select_destination','approve_route','waypoint','cancel_waypoint','acknowledge_warning'):return
        super().control(data)

    def inject(self,hazard):
        if hazard['id'] in self.spawned:return
        if hazard.get('event_type')=='tank_damage_aftermath':
            self.inject_tank_damage_aftermath(hazard)
            return
        model=self.model_root/hazard['model']/'model.sdf'
        edge=self.graph.edges[hazard['edge']];a,b=self.graph.nodes[edge.source],self.graph.nodes[edge.target]
        yaw=math.atan2(b[1]-a[1],b[0]-a[0]) if hazard['kind']=='debris' else 0.
        self.spawn_entity(model,hazard['id'],hazard['position'],yaw)
        self.spawned.add(hazard['id'])
        self.publish_json('/evaluation/truth',{'event':'hazard_created','hazard':hazard,'yaw':yaw})
        self.publish_json('/hri/events',{'source':'industrial','event':'incident_injected','id':hazard['id'],
            'edge':hazard['edge'],'kind':hazard['kind'],'detail':'Physical scene updated; robot knowledge requires sensor observation'})

    def spawn_entity(self,model_path,entity_name,position,yaw=0.):
        if entity_name in self.spawned_entities:return False
        model_path=Path(model_path)
        if not model_path.is_file():
            raise RuntimeError(f'Missing physical incident model: {model_path}')
        x,y,z=position
        request=(f'sdf_filename: {json.dumps(str(model_path))} name: {json.dumps(entity_name)} allow_renaming: false '
            f'pose: {{position: {{x: {x} y: {y} z: {z}}} orientation: '
            f'{{z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}}}')
        self.service('create','gz.msgs.EntityFactory',request)
        self.spawned_entities.add(entity_name)
        return True

    def inject_tank_damage_aftermath(self,hazard):
        edge=self.graph.edges[hazard['edge']]
        a,b=self.graph.nodes[edge.source],self.graph.nodes[edge.target]
        yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        created=execute_tank_damage_aftermath(
            hazard,self.industrial_model_root,self.model_root,self.spawn_entity,self.publish_json,self.spawned,yaw)
        if created:
            self.publish_json('/evaluation/truth',{'event':'tank_damage_aftermath_created',
                'incident_id':hazard['event_id'],'equipment_id':hazard['equipment_id'],
                'visual_entity':hazard['visual_entity'],'debris_entity':hazard['id'],
                'affected_edge':hazard['edge']})

    def display_route(self,plan):
        signature=(tuple(plan['recommended']),tuple(sorted(plan['edge_states'].items())))
        if signature==self.last_route:return
        try:ForestManager.show_route(self,plan);self.last_route=signature
        except Exception as exc:self.get_logger().error(f'Industrial route visual failed: {exc}')

    def hazard_status(self):
        self.publish_json('/evaluation/industrial_hazards',{'active':[self.hazards[h] for h in sorted(self.spawned)],
            'assumptions':self.config['hazard_assumptions']})

    def publish_equipment_registry(self):
        try:
            self.equipment_registry=load_equipment_registry(
                self.get_parameter('equipment_registry').value,
                self.get_parameter('world_file').value)
        except (OSError,ValueError,json.JSONDecodeError) as exc:
            self.get_logger().error(f'Equipment registry unavailable: {exc}')
            return
        self.equipment_registry['stamp']=self.get_clock().now().nanoseconds/1e9
        message=String()
        message.data=json.dumps(self.equipment_registry,allow_nan=False)
        self.equipment_pub.publish(message)


def main():run(IndustrialManager)
