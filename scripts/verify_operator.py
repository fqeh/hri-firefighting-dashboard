#!/usr/bin/env python3
"""Verify live operator advisories from actual Nav2 plans and sensor detections."""
import json
import math
from pathlib import Path
import time
from urllib.request import Request, urlopen

BASE='http://127.0.0.1:8080'
output=Path('results/operator-integration.json')
evidence={'status':'failed','source':'actual ROS observations, Nav2, Gazebo and HTTP controls'}

def state():
    with urlopen(BASE+'/api/state',timeout=5) as response:return json.load(response)

def command(data):
    request=Request(BASE+'/api/control',data=json.dumps(data).encode(),headers={'Content-Type':'application/json'})
    with urlopen(request,timeout=5) as response:
        assert response.status==202
        return json.load(response)

def topic(snapshot,key):
    return snapshot.get('topics',{}).get(key,{}).get('data',{})

def until(predicate,timeout=90):
    deadline=time.monotonic()+timeout
    last_error=None
    while time.monotonic()<deadline:
        try:
            snapshot=state()
        except OSError as error:
            last_error=str(error)
            time.sleep(.25)
            continue
        if predicate(snapshot):return snapshot
        time.sleep(.25)
    raise AssertionError(f'Timed out waiting for expected live state; last transport error: {last_error}')

try:
    initial=until(lambda s:s['connected'] and any(h['id']=='hazard_fire' for h in topic(s,'/hri/observations').get('hazards',[])))
    mission=topic(initial,'/hri/uav_status')
    assert mission.get('state')=='observing' and mission.get('pose_fresh') and mission.get('sensor_fresh'),mission
    evidence['uav_mission']=mission
    command({'command':'waypoint','x':12.0,'y':.7})
    request_stamp=topic(initial,'/clock')['seconds']
    def fire_warning(snapshot):
        return next((e['data'] for e in reversed(snapshot.get('timeline',[]))
            if e.get('topic')=='/hri/operator_warning' and e.get('data',{}).get('hazard_id')=='hazard_fire'
            and e['data'].get('stamp',0)>=request_stamp),None)
    warned=until(lambda s:fire_warning(s) is not None)
    evidence['warning']=fire_warning(warned)
    belief_data=topic(warned,'/hri/operator_belief')
    evidence['belief_before']=belief_data.get('beliefs',{}).get('hazard_fire',belief_data)
    command({'command':'cancel_waypoint'})
    canceled=until(lambda s:topic(s,'/hri/navigation').get('state')=='canceled',timeout=30)
    evidence['cancellation']=topic(canceled,'/hri/navigation')
    command({'command':'acknowledge_warning','hazard_id':'hazard_fire'})
    acknowledged=until(lambda s:any(e.get('data',{}).get('event')=='hazard_acknowledged'
                      and e.get('data',{}).get('hazard_id')=='hazard_fire' for e in s.get('timeline',[])),timeout=15)
    evidence['acknowledgment_event']=next(e for e in reversed(acknowledged['timeline'])
        if e.get('data',{}).get('event')=='hazard_acknowledged')
    # Capture direct ROS assertion through observable public belief state when
    # it is the latest incident; later incidents can occupy this topic instead.
    acknowledged=until(lambda s:topic(s,'/hri/operator_belief').get('beliefs',{}).get('hazard_fire',{}).get('acknowledged'),timeout=15)
    belief=topic(acknowledged,'/hri/operator_belief')['beliefs']['hazard_fire']
    assert belief['posterior']['aware']==1.0
    evidence['belief_after']=belief
    command({'command':'waypoint','x':2.0,'y':0.0})
    finished=until(lambda s:topic(s,'/hri/navigation').get('state')=='succeeded'
                   and not s['topics']['/hri/navigation']['stale'],timeout=60)
    position=topic(finished,'/robot/odom')['position']
    assert math.dist(position[:2],[2,0])<.4,position
    evidence.update(status='passed',revised_goal=[2.,0.],final_position=position)
except Exception as error:
    evidence['error']=str(error)
    try:command({'command':'cancel_waypoint'})
    except Exception:pass
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(evidence,indent=2)+'\n')
print(json.dumps(evidence,indent=2))
raise SystemExit(0 if evidence['status']=='passed' else 1)
