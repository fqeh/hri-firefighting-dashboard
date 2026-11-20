"""World-coordinate research hazards and route costs; Gazebo remains the simulator.

A deterministic, one-second radial-front approximation. No CFD, combustion,
spotting or calibrated wildfire prediction is implied.
"""
import heapq
import math
import random


def fuel_at(position, config):
    value = float(config.get('default', 1.0))
    for patch in config.get('patches', []):
        if math.dist(position[:2], patch['center']) <= patch['radius']:
            value = float(patch['load'])
    return max(0.0, value)


class FireModel:
    def __init__(self, config, seed):
        self.config = config
        self.seed = seed
        self.spokes = 12
        self.cache = {}
        self.rates = {}
        rng = random.Random(seed)
        for origin in config.get('origins', []):
            self.rates[origin['id']] = [rng.uniform(.95, 1.05) for _ in range(self.spokes)]

    def state(self, now):
        zones = []
        wind = self.config.get('wind', [0, 0])
        speed = math.hypot(*wind)
        wind_angle = math.atan2(wind[1], wind[0])
        for origin in self.config.get('origins', []):
            elapsed = now - origin['at']
            if elapsed < 0:
                continue
            step = int(elapsed)
            key = (origin['id'], step)
            if key not in self.cache:
                previous = self.cache.get((origin['id'], step-1))
                radii = list(previous) if previous else [origin.get('radius', .6)] * self.spokes
                first = step if previous else 1
                for _ in range(first, step+1):
                    for i in range(self.spokes):
                        angle = i*2*math.pi/self.spokes
                        point = [origin['position'][0]+radii[i]*math.cos(angle),
                                 origin['position'][1]+radii[i]*math.sin(angle)]
                        wind_factor = max(.1, 1+self.config.get('wind_gain', .4)*speed*math.cos(angle-wind_angle))
                        radii[i] = min(self.config.get('max_radius', 5), radii[i] + self.config['base_rate']*
                                       fuel_at(point, self.config['fuel'])*wind_factor*self.rates[origin['id']][i])
                self.cache[key] = radii
            radii = self.cache[key]
            zones.append({'id': origin['id'], 'center': origin['position'][:2], 'radii': radii,
                          'polygon': [[origin['position'][0]+r*math.cos(i*2*math.pi/self.spokes),
                                       origin['position'][1]+r*math.sin(i*2*math.pi/self.spokes)] for i,r in enumerate(radii)],
                          'exposure_cost': origin.get('exposure_cost', 8), 'age': elapsed})
        return zones


def inside_zone(point, zone):
    dx, dy = point[0]-zone['center'][0], point[1]-zone['center'][1]
    angle = math.atan2(dy, dx) % (2*math.pi)
    count = len(zone['radii'])
    coordinate = (angle/(2*math.pi)*count) % count
    a = int(coordinate)
    fraction = coordinate-a
    radius = zone['radii'][a]*(1-fraction)+zone['radii'][(a+1)%count]*fraction
    return math.hypot(dx,dy) <= radius


def edge_exposure(a, b, zones):
    # Midpoint integration in meters, not a navigable grid or a physics model.
    length = math.dist(a,b)
    count = max(1, math.ceil(length/.2))
    return length/count*sum(sum(z['exposure_cost'] for z in zones if inside_zone(
        [a[0]+(b[0]-a[0])*(i+.5)/count, a[1]+(b[1]-a[1])*(i+.5)/count], z)) for i in range(count))


def plan_route(config, position, observed, zones):
    nodes = config['nodes']
    start = min(nodes, key=lambda n: math.dist(position[:2], nodes[n]))
    connector = math.dist(position[:2], nodes[start])
    states, exposure, adjacency = {}, {}, {n: [] for n in nodes}
    for edge in config['edges']:
        a,b = edge['source'], edge['target']
        risk = edge_exposure(nodes[a],nodes[b],zones)
        state = observed.get(edge['id'], 'unknown')
        if state != 'blocked' and risk > 0: state = 'hazardous'
        states[edge['id']] = state
        exposure[edge['id']] = risk
        if state == 'blocked': continue
        cost = math.dist(nodes[a],nodes[b]) + risk*config.get('exposure_weight', 2)
        if state == 'unknown': cost *= config.get('unknown_penalty',1.5)
        adjacency[a].append((b,edge['id'],cost)); adjacency[b].append((a,edge['id'],cost))
    queue = [(connector,start,[start],[])]
    visited = set()
    while queue:
        cost,node,path,edges = heapq.heappop(queue)
        if node in visited: continue
        visited.add(node)
        if node == config['goal']:
            verified = connector<.5 and all(states[e]=='clear' for e in edges)
            return {'recommended':path,'edges':edges,'cost':cost,'edge_states':states,'exposure_costs':exposure,
                    'verified':verified,'safety':'observed_clear' if verified else 'conditional',
                    'connector_distance':connector,'connector_verified':connector<.5,
                    'message':'Observed route available' if verified else 'No verified route currently available; conditional alternative requires checking'}
        for nxt,e,cost_part in adjacency[node]:
            if nxt not in visited: heapq.heappush(queue,(cost+cost_part,nxt,path+[nxt],edges+[e]))
    return {'recommended':[],'edges':[],'cost':None,'edge_states':states,'exposure_costs':exposure,
            'verified':False,'safety':'unavailable','message':'No verified route currently available; no connected alternative'}


def incident_sequence(config):
    rng = random.Random(config['seed'])
    return sorted([{**event, 'at': event['at']+rng.uniform(0,event.get('jitter',0))}
                   for event in config.get('hazards',[])], key=lambda event:(event['at'],event['id']))
