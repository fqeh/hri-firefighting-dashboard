"""Common forest/oil-field route contract from public map and measured hazards."""
import math
from .forest_model import plan_route,edge_exposure
from .core import RouteGraph


def recommend(map_config,position,hazards,clear_edges,config,entity='crew'):
    graph=RouteGraph(map_config)
    observed={edge:'clear' for edge in clear_edges if edge in graph.edges}
    zones=[]
    for h in hazards:
        edge=h['edge']
        if edge not in graph.edges:continue
        if h.get('state')=='blocked' or h['kind'] in ('tree','debris'):
            observed[edge]='blocked'
        else:
            if observed.get(edge)!='blocked':observed[edge]='hazardous'
            zones.append({'center':h['position'][:2],'radii':[h.get('radius',2.)+.4]*12,
                          'exposure_cost':config['fire_exposure_cost']})
    planner_config={**map_config,**config}
    plan=plan_route(planner_config,position,observed,zones)
    costs={}
    for edge in graph.edges.values():
        risk=edge_exposure(graph.nodes[edge.source],graph.nodes[edge.target],zones)
        state=plan['edge_states'][edge.id]
        base=edge.length+risk*config['exposure_weight']
        penalty=base*(config['unknown_penalty']-1) if state=='unknown' else 0.
        costs[edge.id]={'distance_m':edge.length,'exposure_integral':risk,
            'weighted_exposure':risk*config['exposure_weight'],'unknown_penalty':penalty,
            'total':None if state=='blocked' else base+penalty,'state':state}
    if plan['recommended']:
        # The off-graph connector is conditional; never allow it through a known
        # obstruction. Conservative semantic radius encloses the colliding log.
        target=graph.nodes[plan['recommended'][0]]
        from .forest_model import inside_zone
        length=math.dist(position[:2],target);count=max(1,math.ceil(length/.1))
        for h in hazards:
            if h['kind'] not in ('tree','debris'):continue
            radius=min(1.9,h.get('radius',2.))
            # Starting inside the conservative envelope is permitted only while
            # moving outward; actual collision remains Gazebo's responsibility.
            start_distance=math.dist(position[:2],h['position'][:2])
            blocked=any(math.dist([position[j]+(target[j]-position[j])*i/count for j in (0,1)],h['position'][:2])
                        <min(radius,start_distance)-.02 for i in range(1,count+1))
            if blocked:
                plan.update(recommended=[],edges=[],cost=None,verified=False,safety='unavailable',
                            message='No verified route: connector would enter an observed obstruction')
                break
    path=plan['recommended'];known=plan['verified']
    description=(' → '.join(path) if path else 'no connected route')
    explanation=f'Risk-weighted route: {description}. Blocked edges excluded; exposure weight {config["exposure_weight"]:g}, unknown multiplier {config["unknown_penalty"]:g}.'
    if not known:explanation+=' No verified safe route is currently available; any listed alternative is conditional.'
    return {**plan,'schema_version':1,'entity':entity,'frame':'world','nodes':graph.nodes,
        'waypoints':[{'node':n,'x':graph.nodes[n][0],'y':graph.nodes[n][1]} for n in path],
        'edge_costs':costs,'unknown_edges':[e for e in plan['edges'] if plan['edge_states'][e]=='unknown'],
        'blocked_edges':[e for e,s in plan['edge_states'].items() if s=='blocked'],
        'selected_exposure':sum(costs[e]['exposure_integral'] for e in plan['edges']),
        'explanation':explanation,'crew_position':list(position[:2])}
