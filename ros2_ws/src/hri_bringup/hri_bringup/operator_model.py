"""Belief-aware operator advisories from real goals and robot observations.

No hidden operator-belief or incident-truth input exists. Acknowledgment is an
observable operator report; Nav2 remains responsible for motion and avoidance.
"""
import math
from hri_reasoning.core import RouteGraph, BeliefFilter, WarningPolicy


def segment_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx + dy*dy
    t = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2)) if length2 else 0.
    return math.dist(point[:2], (a[0]+t*dx, a[1]+t*dy))


class OperatorModel:
    def __init__(self, config):
        self.graph = RouteGraph(config)
        self.hazards = {}
        self.filters = {}
        self.policy = WarningPolicy('B4')
        self.goal = None
        self.position = None
        self.plan = None
        self.active = False
        self.request_number = 0
        self.applied_evidence = {}
        self.acknowledged = set()

    def observe(self, hazards):
        for hazard in hazards:
            if hazard['edge'] not in self.graph.edges:
                continue
            self.hazards[hazard['id']] = dict(hazard)
            self.filters.setdefault(hazard['id'], BeliefFilter())

    def request(self, x, y):
        self.goal = (x, y)
        self.plan = None
        self.active = True
        self.request_number += 1

    def acknowledge(self, hazard_id):
        if hazard_id not in self.filters:
            return False
        self.filters[hazard_id].unaware = 0.0
        self.acknowledged.add(hazard_id)
        return True

    def evaluate(self, stamp):
        if self.goal is None or self.position is None:
            return [], [], None
        start, goal = self.graph.nearest(self.position), self.graph.nearest(self.goal)
        # This is the prior route implied by the declared waypoint, not a claim
        # that the operator secretly knows any particular map state.
        intended = self.graph.shortest(start, goal, {e: 'clear' for e in self.graph.edges})['path']
        known = {h['edge']: 'blocked' for h in self.hazards.values()}
        recommended = self.graph.shortest(start, goal, known)
        route = {'nodes': self.graph.nodes, 'intended': intended,
                 'recommended': recommended['path'], 'unknown_edges': recommended['unknown_edges'],
                 'safety': recommended['safety'], 'blocked_edges': list(known),
                 'explanation': 'Graph alternative excludes observed hazard corridors; unobserved sections need confirmation. '
                                'The operator must cancel or revise the waypoint; this adviser does not control motion.'}
        beliefs, warnings = [], []
        for hid, hazard in self.hazards.items():
            belief = self.filters[hid]
            implied_segment = next(((a,b) for a,b in zip(intended,intended[1:])
                                    if self.graph.edge_between(a,b).id == hazard['edge']), None)
            if self.applied_evidence.get(hid) != self.request_number and hid not in self.acknowledged:
                if implied_segment:
                    belief.update_action(self.graph, *implied_segment, goal, hazard['edge'])
                self.applied_evidence[hid] = self.request_number
            if self.plan is None:
                intersects = implied_segment is not None
                basis = 'requested waypoint route'
            else:
                intersects = any(segment_distance(hazard['position'], a,b) <= hazard.get('radius',2.)+.5
                                 for a,b in zip(self.plan,self.plan[1:]))
                basis = 'current Nav2 plan'
            relevant = self.active and intersects
            beliefs.append({'hazard_id': hid, 'edge': hazard['edge'],
                            'posterior': {'unaware':belief.unaware,'aware':1-belief.unaware},
                            'uncertainty': belief.entropy, 'relevant': relevant,
                            'acknowledged': hid in self.acknowledged, 'evidence': basis})
            action = self.policy.choose(hid,stamp,belief.unaware,relevant,
                                        math.dist(self.position,hazard['position'][:2]))
            if action != 'silent':
                warnings.append({'hazard_id':hid,'edge':hazard['edge'],'action':action,
                                 'severity':'critical' if action=='explicit' else 'advisory',
                                 'explanation': f'Observed {hazard.get("kind","hazard")} intersects the {basis}. '
                                     f'Estimated outdated operator belief: {belief.unaware:.0%}. '
                                     'Review the plan, acknowledge the hazard, or cancel and choose another waypoint.'})
        return beliefs, warnings, route
