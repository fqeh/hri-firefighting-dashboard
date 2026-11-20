"""Evaluation-only access to private belief and physical truth; not a ROS input."""
import math
from hri_reasoning.core import RouteGraph
from hri_reasoning.tom import canonical,route
from hri_reasoning.forest_model import inside_zone


def evaluate(records):
    metadata=next(r['data'] for r in records if r['topic']=='run_metadata')
    config=metadata['config'];graph=RouteGraph(config);goal=config['goal']
    responses={r['data']['warning_id']:r['data'] for r in records
        if r['topic']=='/evaluation/human_belief' and 'warning_id' in r['data']}
    knowledge=None;position=graph.nodes[config['start']];active={};episodes=[];open_episodes={}
    warnings=[];previous_pose=None;exposure=0.;stalled_since=None;entrapped=False;completed=False
    plans=[];detected={};replan_latencies=[];route_revision_seen=set()
    normal_observations=set();acks=set();last_pose_stamp=0.;completed_at=None;fire_zones=[]

    def danger(k,pos,h):
        if k is None:return False
        incompatible=('open_safe','open_hazardous') if h.get('kind') in ('tree','debris') else ('open_safe',)
        if canonical(k[h['edge']]) not in incompatible:return False
        path,_=route(graph,graph.nearest(pos),goal,k)
        edges={graph.edge_between(x,y).id for x,y in zip(path,path[1:])}
        return h['edge'] in edges

    ordered=sorted(enumerate(records),key=lambda item:(item[1]['data'].get('stamp',item[1].get('stamp',0)),item[0]))
    for _,row in ordered:
        topic,data=row['topic'],row['data'];stamp=data.get('stamp',row.get('stamp',0))
        if topic=='/evaluation/truth' and data.get('event')=='hazard_created':
            h=data['hazard'];active[h['id']]=h
        elif topic=='/evaluation/human_belief' and 'knowledge' in data:knowledge=data['knowledge']
        elif topic=='/forest/fire_state':fire_zones=data.get('zones',[])
        elif topic=='/human/odom':
            position=data['position'][:2];last_pose_stamp=stamp
            near=(any(inside_zone(position,z) for z in fire_zones) or
                  any(math.dist(position,h['position'][:2])<=h.get('radius',2)
                      for h in active.values() if h['kind']!='fire' or not fire_zones))
            if previous_pose:
                dt=max(0.,min(.5,stamp-previous_pose[0]));motion=math.dist(position,previous_pose[1])
                if near:exposure+=dt
                if near and motion<.002:
                    if stalled_since is None:stalled_since=stamp
                    if stamp-stalled_since>=10:entrapped=True
                else:stalled_since=None
            previous_pose=(stamp,position)
            if math.dist(position,graph.nodes[goal])<.65:
                completed=True
                if completed_at is None:completed_at=stamp
        elif topic=='/hri/observations':
            for h in data.get('hazards',[]):
                if h['kind'] in ('tree','debris') or h.get('state')=='blocked':
                    detected.setdefault(h['edge'],stamp)
                normal_observations.add((h['id'],h['kind'],h['edge'],tuple(round(x,2) for x in h['position']),h.get('radius')))
        elif topic=='/hri/rescue_route':
            plans.append(data)
            if data['revision'] not in route_revision_seen:
                route_revision_seen.add(data['revision'])
                for edge in data.get('blocked_edges',[]):
                    if edge in detected and edge not in data.get('edges',[]):replan_latencies.append(max(0,stamp-detected[edge]))
        elif topic=='/hri/acknowledgement':acks.add(data['warning_id'])
        elif topic=='/hri/warning':
            h=active.get(data.get('hazard_id'))
            before=responses.get(data['id'],{}).get('knowledge_before',knowledge)
            needed=bool(h and danger(before,position,h))
            warnings.append({**data,'needed':needed})
        for identifier,h in active.items():
            dangerous=danger(knowledge,position,h) and not completed
            if dangerous and identifier not in open_episodes:
                episode={'hazard_id':identifier,'start':stamp,'end':None};episodes.append(episode);open_episodes[identifier]=episode
            elif not dangerous and identifier in open_episodes:open_episodes.pop(identifier)['end']=stamp
    for e in episodes:
        if e['end'] is None:e['end']=last_pose_stamp
        e['warnings']=[w['id'] for w in warnings if w['needed'] and w.get('hazard_id')==e['hazard_id']
                       and e['start']-.15<=w['stamp']<=e['end']+.15]
    true=sum(w['needed'] for w in warnings);missed=sum(not e['warnings'] for e in episodes)
    delays=[min(w['stamp'] for w in warnings if w['id'] in e['warnings'])-e['start'] for e in episodes if e['warnings']]
    alert_responses=[responses[w['id']] for w in warnings if w['id'] in responses]
    replans=[p['replan_duration_ms'] for p in plans]
    return {'warning_count':len(warnings),'true_alerts':true,'false_alarms':len(warnings)-true,
        'warning_precision':true/len(warnings) if warnings else None,
        'warning_recall':(len(episodes)-missed)/len(episodes) if episodes else None,
        'dangerous_episodes':len(episodes),'missed_warnings':missed,'episodes':episodes,
        'mean_time_to_warning_s':sum(delays)/len(delays) if delays else None,
        'exposure_seconds':exposure,'exposure_rate':exposure/last_pose_stamp if last_pose_stamp else None,
        'entrapment':entrapped,'evacuation_completed':completed,'completion_time_s':completed_at,
        'completion_censored':not completed,'interruptions':len(warnings),
        'acknowledgements':len(acks),'modeled_compliance':sum(r['complied'] for r in alert_responses)/len(alert_responses) if alert_responses else None,
        'modeled_route_change':sum(r['route_changed'] for r in alert_responses)/len(alert_responses) if alert_responses else None,
        'route_success':completed,'route_revisions':len(route_revision_seen),
        'mean_replanning_ms':sum(replans)/len(replans) if replans else None,
        'max_replanning_ms':max(replans) if replans else None,
        'first_blockage_replan_latency_s':min(replan_latencies) if replan_latencies else None,
        'unsafe_blocked_recommendations':sum(bool(set(p.get('edges',[]))&set(p.get('blocked_edges',[]))) for p in plans),
        'semantic_robot_observations':sorted(normal_observations),
        'notes':'Episode recall uses true belief-optimal routes; precision uses knowledge before each response. Trust/compliance are simulated. Exposure uses time-varying forest fire truth when available and incident envelopes otherwise. Ten seconds stalled within these envelopes defines entrapment. No operational dose model.'}
