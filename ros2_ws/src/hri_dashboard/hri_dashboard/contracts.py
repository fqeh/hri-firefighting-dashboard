"""Dashboard protocol v1. Public telemetry only; no private belief subscription."""
import math
from typing import Any, Literal, TypedDict

Source = Literal['observation', 'inference', 'simulation_truth', 'control', 'configuration']

class TopicEnvelope(TypedDict):
    data: dict[str, Any]
    received_at: float
    age_seconds: float
    stale: bool
    source: Source
    sim_stamp: float | None

def provenance(topic: str) -> Source:
    if topic == '/hri/equipment_registry':
        return 'configuration'
    if topic in ('/forest/fire_state', '/human/odom', '/uav/odom'):
        return 'simulation_truth'
    if topic in ('/hri/belief', '/hri/decision', '/hri/routes', '/hri/rescue_route', '/hri/robot_route', '/hri/warning', '/hri/escalation'):
        return 'inference'
    if topic.startswith('/dashboard/') or topic in ('/hri/status','/hri/events','/hri/navigation','/hri/operator_receipt','/hri/acknowledgement'):
        return 'control'
    return 'observation'

def validate_payload(topic, value):
    def check(item, depth=0):
        if depth > 20: raise ValueError('Payload nesting limit')
        if isinstance(item, float) and not math.isfinite(item): raise ValueError('Nonfinite number')
        if isinstance(item, dict):
            if len(item) > 2000: raise ValueError('Object size limit')
            for v in item.values(): check(v, depth+1)
        elif isinstance(item, list):
            if len(item) > 10000: raise ValueError('Array size limit')
            for v in item: check(v, depth+1)
    if not isinstance(value, dict): raise ValueError('Expected object')
    check(value)
    if 'stamp' in value and (type(value['stamp']) not in (int,float) or value['stamp'] < 0):
        raise ValueError('Invalid simulation timestamp')
    if topic == '/hri/observations':
        if not isinstance(value.get('hazards'),list): raise ValueError('Expected hazard list')
        for h in value['hazards']:
            if not isinstance(h,dict) or not isinstance(h.get('id'),str): raise ValueError('Invalid hazard')
            p=h.get('position')
            if not isinstance(p,list) or len(p)<2 or any(type(x) not in (int,float) for x in p): raise ValueError('Invalid hazard position')
    if topic == '/hri/equipment_registry':
        entries=value.get('equipment')
        if value.get('schema_version')!=1 or not isinstance(entries,list) or not entries or len(entries)>128:
            raise ValueError('Invalid equipment registry')
        ids=set();models=set()
        for entry in entries:
            if not isinstance(entry,dict):raise ValueError('Invalid equipment entry')
            equipment_id,model=entry.get('equipment_id'),entry.get('gazebo_model')
            position,contents=entry.get('position_m'),entry.get('contents')
            if not isinstance(equipment_id,str) or not equipment_id or equipment_id in ids:
                raise ValueError('Invalid or duplicate equipment_id')
            if not isinstance(model,str) or not model or model in models:
                raise ValueError('Invalid or duplicate Gazebo model reference')
            if (not isinstance(position,list) or len(position)!=3
                    or any(type(coordinate) not in (int,float) or not math.isfinite(coordinate) for coordinate in position)):
                raise ValueError('Invalid equipment position')
            if not isinstance(contents,dict) or any(
                not isinstance(contents.get(field),str) for field in ('label','material_class','source')
            ):
                raise ValueError('Invalid equipment contents metadata')
            ids.add(equipment_id);models.add(model)
    if topic == '/hri/belief':
        if not isinstance(value.get('beliefs',{}),dict):raise ValueError('Invalid belief map')
        for belief in [value, *value.get('beliefs',{}).values()]:
            if not isinstance(belief,dict):raise ValueError('Invalid belief entry')
            p=belief.get('posterior')
            if p is not None and (not isinstance(p,dict) or set(p)!= {'open_safe','blocked','open_hazardous','unknown_stale'}
                    or any(type(v) not in (int,float) or not 0<=v<=1 for v in p.values())
                    or abs(sum(p.values())-1)>1e-5): raise ValueError('Invalid posterior distribution')
    return value
