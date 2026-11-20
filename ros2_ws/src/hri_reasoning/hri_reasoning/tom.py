"""Factorized Bayesian Theory of Mind. No ROS, scenario files, or private state.

Inputs are a public route graph, observed actions and information opportunities.
One four-state categorical HMM per edge costs O(E), not 4**E. Other edges use
the previous MAP hypotheses when computing a conditional action likelihood.
This is an explicit mean-field approximation, not an exact joint posterior.
"""
from __future__ import annotations
import heapq
import math
from .core import RouteGraph

STATES = ('open_safe', 'blocked', 'open_hazardous', 'unknown_stale')
ALIASES = {'clear': 'open_safe', 'hazardous': 'open_hazardous', 'unknown': 'unknown_stale'}
WAIT = '__wait__'
DEFAULT_PRIOR = (.55, .15, .15, .15)
DEFAULT_TRANSITION = (
    (.998, 0., 0., .002),
    (0., .998, 0., .002),
    (0., 0., .998, .002),
    (.002, 0., 0., .998),
)


class MotionStall:
    """Fresh odometry with attempted travel, <8cm progress for two sim seconds."""
    def __init__(self):
        self.anchor = None

    def update(self, now, position, attempting):
        if not attempting:
            self.anchor = None
            return False
        if self.anchor is None or math.dist(position,self.anchor[1]) >= .08:
            self.anchor = (now,tuple(position))
            return False
        return now-self.anchor[0] >= 2.


def canonical(state):
    value = ALIASES.get(state, state)
    if value not in STATES:
        raise ValueError(f'Unknown belief hypothesis: {state}')
    return value


def semantic_hazard_kind(name):
    return next((kind for kind in ('tree', 'fire', 'gas', 'debris')
                 if name == f'hazard_{kind}' or name.startswith(f'hazard_{kind}_')), None)


def probabilities(values, size=4):
    values = tuple(float(x) for x in values)
    if len(values) != size or any(not math.isfinite(x) or x < 0 for x in values) or sum(values) <= 0:
        raise ValueError('Expected a finite nonnegative distribution with positive mass')
    total = sum(values)
    return tuple(x / total for x in values)


def public_map(config):
    """Deliberate allowlist: excludes hazards, schedules, seeds and human beliefs."""
    return {'nodes': config['nodes'], 'edges': [
        {k: edge[k] for k in ('id', 'source', 'target')} for edge in config['edges']],
        'start': config.get('start', 'start'), 'goal': config.get('goal', 'safety')}


def validate_public_map(config):
    if set(config) != {'nodes', 'edges', 'start', 'goal'}:
        raise ValueError('Inference accepts only the public route-map schema')
    if any(set(edge) != {'id', 'source', 'target'} for edge in config['edges']):
        raise ValueError('Private edge annotations are forbidden')
    return RouteGraph(config)


def edge_cost(edge, knowledge, hazard_multiplier=4., unknown_multiplier=1.6):
    state = canonical(knowledge.get(edge.id, 'open_safe'))
    return edge.length * {'open_safe': 1., 'blocked': math.inf,
                          'open_hazardous': hazard_multiplier, 'unknown_stale': unknown_multiplier}[state]


def route(graph, start, goal, knowledge, excluded=(), **costs):
    queue = [(0., start, [start])]
    visited = set(excluded)
    while queue:
        cost, node, path = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        if node == goal:
            return path, cost
        for nxt, edge in graph.adjacency[node]:
            value = edge_cost(edge, knowledge, **costs)
            if nxt not in visited and math.isfinite(value):
                heapq.heappush(queue, (cost + value, nxt, path + [nxt]))
    return [], math.inf


def action_distribution(graph, origin, goal, knowledge, beta=1.2, lapse=.03,
                        wait_cost=2., **costs):
    """Q=-cost-to-go, simple suffix avoiding the current node; wait costs best+2m.

    Subtract the minimum before exp to avoid overflow. Uniform lapse over all
    adjacent actions plus waiting gives unexpected actions nonzero likelihood.
    Blocked edges remain excluded from the deliberate (1-lapse) policy.
    """
    if not math.isfinite(beta) or beta < 0 or not math.isfinite(lapse) or not 0 <= lapse <= 1:
        raise ValueError('Invalid rationality or lapse probability')
    if not math.isfinite(wait_cost) or wait_cost < 0:
        raise ValueError('Invalid waiting cost')
    for value in costs.values():
        if not math.isfinite(value) or value < 1:
            raise ValueError('Cost multipliers must be finite and at least one')
    available = {}
    actions = [n for n, _ in graph.adjacency[origin]] + [WAIT]
    if origin == goal:
        return {a: float(a == WAIT) for a in actions}
    for nxt, edge in graph.adjacency[origin]:
        _, remaining = route(graph, nxt, goal, knowledge, excluded=(origin,), **costs)
        total = edge_cost(edge, knowledge, **costs) + remaining
        if math.isfinite(total):
            available[nxt] = total
    best = min(available.values(), default=0.)
    available[WAIT] = best + wait_cost if available else 0.
    weights = {a: math.exp(-beta * (c - best)) for a, c in available.items()}
    norm = sum(weights.values())
    return {a: (1-lapse)*weights.get(a, 0.)/norm + lapse/len(actions) for a in actions}


class CategoricalHMM:
    def __init__(self, prior=DEFAULT_PRIOR, transition=DEFAULT_TRANSITION):
        self.p = probabilities(prior)
        if len(transition) != 4:
            raise ValueError('Transition matrix must be 4 by 4')
        self.transition = tuple(probabilities(row) for row in transition)
        if any(abs(sum(row)-1) > 1e-9 for row in transition):
            raise ValueError('Transition matrix rows must sum to one')

    def predict(self, steps=1):
        if not isinstance(steps, int) or steps < 0:
            raise ValueError('Prediction uses nonnegative integer simulation seconds')
        for _ in range(steps):
            self.p = probabilities(sum(self.p[i]*self.transition[i][j] for i in range(4)) for j in range(4))

    def information(self, state, probability):
        """Event-conditioned stochastic transition: probability of acquiring info.

        This does not assert compliance or read whether acquisition really occurred.
        """
        index = STATES.index(canonical(state))
        if not math.isfinite(probability) or not 0 <= probability <= 1:
            raise ValueError('Acquisition probability must be in [0,1]')
        self.p = tuple((1-probability)*p + (probability if i == index else 0.) for i,p in enumerate(self.p))

    def evidence(self, likelihood):
        if len(likelihood) != 4 or any(not math.isfinite(x) or x < 0 for x in likelihood):
            raise ValueError('Likelihoods must be four finite nonnegative values')
        logs = [math.log(p)+math.log(l) if p > 0 and l > 0 else -math.inf for p,l in zip(self.p,likelihood)]
        maximum = max(logs)
        if maximum == -math.inf:
            return False  # Impossible observation: retain prior, report ambiguity.
        weights = [math.exp(value-maximum) for value in logs]
        self.p = probabilities(weights)
        return True

    @property
    def posterior(self):
        return dict(zip(STATES, self.p))

    @property
    def entropy(self):
        return -sum(p*math.log(p) for p in self.p if p > 0)


class RouteToM:
    def __init__(self, map_config, beta=1.2, lapse=.03, prior=DEFAULT_PRIOR,
                 transition=DEFAULT_TRANSITION, **costs):
        self.graph = validate_public_map(map_config)
        self.goal = map_config['goal']
        self.beta, self.lapse, self.costs = beta, lapse, costs
        # Validate parameters even before an action arrives.
        action_distribution(self.graph, map_config['start'], self.goal, {}, beta, lapse, **costs)
        self.filters = {e: CategoricalHMM(prior, transition) for e in self.graph.edges}
        self.observed = {}
        self.last_relevant = {e: None for e in self.filters}
        self.last_action = None
        self.last_prediction = None
        self.action_count = 0
        self.informative_actions = {e:0 for e in self.filters}

    def advance(self, now):
        if not math.isfinite(now) or now < 0:
            raise ValueError('Invalid simulation time')
        second = int(now)
        if self.last_prediction is not None:
            if second < self.last_prediction:
                raise ValueError('Restart inference with Gazebo when resetting time')
            for f in self.filters.values():
                f.predict(second-self.last_prediction)
        self.last_prediction = second

    def observe(self, edge, state, stamp, position, identifier):
        """Robot seeing a hazard changes the public world estimate, NOT human belief."""
        if edge not in self.filters or canonical(state) not in ('blocked', 'open_hazardous'):
            raise ValueError('Expected an observed hazardous/blocked map edge')
        self.observed[edge] = {'state': canonical(state), 'stamp': stamp,
                               'position': list(position), 'id': identifier}
        self.last_relevant[edge] = stamp

    def information(self, edge, state, probability, stamp):
        self.filters[edge].information(state, probability)
        self.last_relevant[edge] = stamp

    def action(self, origin, target, stamp):
        if origin not in self.graph.nodes or target not in [n for n,_ in self.graph.adjacency[origin]]+[WAIT]:
            return False
        context = {e: STATES[max(range(4), key=lambda i: f.p[i])] for e,f in self.filters.items()}
        likelihoods = {}
        for edge in self.filters:
            likelihoods[edge] = [action_distribution(self.graph, origin, self.goal,
                {**context, edge: h}, self.beta, self.lapse, **self.costs).get(target, 0.) for h in STATES]
        for edge, values in likelihoods.items():
            self.filters[edge].evidence(values)
            if max(values)-min(values) > 1e-9:
                self.last_relevant[edge] = stamp
                self.informative_actions[edge] += 1
        self.action_count += 1
        self.last_action = {'origin': origin, 'target': target, 'stamp': stamp}
        return True

    def summary(self, intended_edges=()):
        result = {}
        for edge, f in self.filters.items():
            observation = self.observed.get(edge)
            relevant = edge in intended_edges
            # Unknown/stale is distinct from positively believing the route safe.
            safe = f.p[0]
            # A blocked trail is also incompatible with believing it open but
            # hazardous. For an observed fire, only a safe belief contradicts it.
            outdated = safe + (f.p[2] if observation and observation['state']=='blocked' else 0.)
            result[edge] = {'edge': edge, 'posterior': f.posterior,
                'informative_actions': self.informative_actions[edge],
                'uncertainty': f.entropy, 'confidence': 1-f.entropy/math.log(4),
                'p_outdated': outdated if observation else None,
                'p_mismatch': (outdated if relevant else 0.) if observation else None,
                'mismatch_upper_bound': (outdated+f.p[3] if relevant else 0.) if observation else None,
                'relevant': relevant, 'hazard_observed': bool(observation),
                'last_relevant_observation': self.last_relevant[edge],
                'last_hazard_observation': observation['stamp'] if observation else None,
                'hazard_id': observation['id'] if observation else None}
        return result
