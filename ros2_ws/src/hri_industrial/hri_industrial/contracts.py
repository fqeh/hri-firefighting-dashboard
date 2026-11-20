"""Environment-specific geometry and input validation; HRI algorithms are imported."""
import math
from hri_reasoning.core import RouteGraph
from hri_reasoning.tom import route
from hri_reasoning.route_risk import recommend
from hri_bringup.operator_model import segment_distance


def requested_path(public_map,position,destination,via=None):
    graph=RouteGraph(public_map);start=graph.nearest(position)
    if destination not in graph.nodes:raise ValueError('Select a named, surveyed facility waypoint')
    if via:
        if via not in graph.edges:raise ValueError('Unknown preferred corridor')
        edge=graph.edges[via];options=[]
        for a,b in ((edge.source,edge.target),(edge.target,edge.source)):
            prefix,cost1=route(graph,start,a,{via:'blocked'})
            suffix,cost2=route(graph,b,destination,{via:'blocked'},excluded=prefix[:-1])
            if prefix and suffix:options.append((cost1+edge.length+cost2,prefix+suffix))
        if not options:raise ValueError('Preferred corridor cannot connect this destination')
        path=min(options)[1]
    else:path,_=route(graph,start,destination,{})
    # Explicit future branching choices are operator actions, even before the
    # robot reaches the junction. Autonomous Nav2 detours never enter this path.
    branch=next(((a,b) for a,b in zip(path,path[1:]) if len(graph.adjacency[a])>=3),None)
    return path,branch or (None,None)


def on_road(point,layout,edges=None,margin=0):
    allowed=set(edges) if edges is not None else {e['id'] for e in layout['edges']}
    return any(e['id'] in allowed and segment_distance(point,layout['nodes'][e['source']],layout['nodes'][e['target']])
               <=layout['road_widths'][e['id']]/2-margin for e in layout['edges'])


def safe_recommendation(public_map,layout,position,destination,hazards,clear,route_costs):
    mapping={**public_map,'goal':destination}
    full=recommend(mapping,position,hazards,clear,route_costs,'robot')
    safe_edges={e for e,s in full['edge_states'].items() if s=='clear'
                and full['edge_costs'][e]['exposure_integral']<=layout['max_route_exposure']}
    safe_map={**mapping,'edges':[e for e in mapping['edges'] if e['id'] in safe_edges]}
    # Route from a surveyed road segment, not the geometrically nearest graph
    # node: a robot halfway along a long road can be closer to a parallel,
    # disconnected corridor than to either endpoint of its actual road.
    candidates=[]
    for edge in layout['edges']:
        if edge['id'] not in safe_edges:continue
        a=mapping['nodes'][edge['source']];b=mapping['nodes'][edge['target']]
        for node in (edge['source'],edge['target']):
            target=mapping['nodes'][node];distance=math.dist(position[:2],target)
            samples=max(1,math.ceil(distance/.1))
            points=[[position[j]+(target[j]-position[j])*i/samples for j in (0,1)]
                    for i in range(samples+1)]
            on_this_road=all(segment_distance(point,a,b)<=layout['road_widths'][edge['id']]/2-.35
                             for point in points)
            clear_of_hazards=all(math.dist(point,h['position'][:2])>h.get('radius',2)+.35
                                 for point in points for h in hazards)
            if not on_this_road or not clear_of_hazards:continue
            route=recommend(safe_map,target,hazards,clear,route_costs,'robot')
            if not route['recommended']:continue
            route['cost']+=distance
            route['connector_distance']=distance
            route['connector_verified']=True
            route['connector_edges']=[edge['id']] if distance>.01 else []
            candidates.append(route)
    candidate=min(candidates,key=lambda plan:(plan['cost'],plan['recommended'][0])) if candidates else {
        **full,'recommended':[],'edges':[],'cost':None,'connector_distance':None,
        'connector_verified':False,'message':'No verified route currently available; conditional alternative requires checking'}
    candidate['edges']=list(dict.fromkeys(candidate.get('connector_edges',[])+candidate['edges']))
    allowed=bool(candidate['recommended']) and candidate['connector_verified']
    keys=('recommended','edges','cost','connector_distance','connector_verified','message')
    plan={**full,**{k:candidate.get(k) for k in keys}}
    plan.update(destination=destination,navigation_allowed=allowed,verified=allowed,
        safety='surveyed_and_observed_clear' if allowed else 'unavailable',
        connector_verified=candidate['connector_verified'],
        survey_source=layout['survey_assumption'],conditional_candidate=full['recommended'])
    if not allowed:plan.update(recommended=[],edges=[],cost=None)
    plan['waypoints']=[{'node':n,'x':mapping['nodes'][n][0],'y':mapping['nodes'][n][1]} for n in plan['recommended']]
    plan['selected_exposure']=sum(full['edge_costs'][e]['exposure_integral'] for e in plan['edges'])
    plan['explanation']=(f'Approved-area risk route to {destination}: '+(' → '.join(plan['recommended']) if allowed else 'none')+
        '. Physical blockages, known hazard exposure and unverified segments are excluded. '+
        ('Operator approval is required before motion.' if allowed else 'Stop and request assistance; no sufficiently safe known route is available.'))
    return plan


def navigation_mask(layout,hazards):
    xmin,ymin,xmax,ymax=layout['bounds'];res=layout['mask_resolution']
    width=math.ceil((xmax-xmin)/res);height=math.ceil((ymax-ymin)/res)
    values=[]
    for row in range(height):
        for col in range(width):
            point=[xmin+(col+.5)*res,ymin+(row+.5)*res]
            allowed=on_road(point,layout,layout['initial_survey_edges']) and all(math.dist(point,h['position'][:2])>h.get('radius',2)+.35 for h in hazards)
            values.append(0 if allowed else 100)
    return width,height,values
