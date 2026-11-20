"""Research models, never an environment simulator.

Positions and sensor detections are supplied by Gazebo. This module only models
knowledge, route preferences and communication decisions. A graph is a planning
abstraction over world coordinates, not a grid-world or a physics replacement.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import random
from typing import Mapping


def validate_behavior_parameters(beta=1.2,assumed_beta=1.2,trust=.8,observation_radius=3.0,passive_map_rate=.05):
    values=(beta,assumed_beta,trust,observation_radius,passive_map_rate)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Behavior parameters must be finite')
    if beta<0 or assumed_beta<0 or observation_radius<=0 or passive_map_rate<0 or not 0<=trust<=1:
        raise ValueError('Rationalities and attention rate must be nonnegative, radius positive, and trust in [0,1]')


def behavior_rng(seed,purpose):
    """Stable separate streams prevent a warning from consuming route/glance draws."""
    return random.Random(f'hri-human-v1:{seed}:{purpose}')


def glance_probability(rate,seconds):
    """Probability of at least one event of a homogeneous Poisson process."""
    if not math.isfinite(rate) or not math.isfinite(seconds) or rate<0 or seconds<0:
        raise ValueError('Rate and interval must be finite and nonnegative')
    return -math.expm1(-rate*seconds)


class PassiveMapAttention:
    """An independently seeded Poisson clock for voluntary dashboard glances."""
    def __init__(self,rate,seed):
        glance_probability(rate,0)
        self.rate = rate
        self.rng = behavior_rng(seed,'glance')
        self.next_glance = None

    def poll(self,now):
        if not math.isfinite(now):
            raise ValueError('Simulation time must be finite')
        if self.rate == 0:
            return []
        if self.next_glance is None:
            self.next_glance = now+self.rng.expovariate(self.rate)
        occurred = []
        while self.next_glance <= now:
            occurred.append(self.next_glance)
            self.next_glance += self.rng.expovariate(self.rate)
        return occurred


def pose_is_fresh(now,source_stamp,timeout=.5):
    """Allow a small clock/pose bridge delivery skew, never indefinite pose reuse."""
    return (source_stamp is not None and math.isfinite(now) and math.isfinite(source_stamp)
            and -.1<=now-source_stamp<=timeout)


@dataclass(frozen=True)
class Edge:
    id: str
    source: str
    target: str
    length: float


class RouteGraph:
    def __init__(self, config: dict):
        self.nodes = {name: tuple(point[:2]) for name, point in config['nodes'].items()}
        self.edges = {}
        self.adjacency = {name: [] for name in self.nodes}
        for item in config['edges']:
            a, b = item['source'], item['target']
            edge = Edge(item['id'], a, b, math.dist(self.nodes[a], self.nodes[b]))
            if edge.length <= 0 or edge.id in self.edges:
                raise ValueError('Edges need unique IDs and positive geometric lengths')
            self.edges[edge.id] = edge
            self.adjacency[a].append((b, edge))
            self.adjacency[b].append((a, edge))

    def edge_between(self, a: str, b: str) -> Edge:
        return next(edge for node, edge in self.adjacency[a] if node == b)

    def nearest(self, position) -> str:
        return min(self.nodes, key=lambda n: math.dist(position[:2], self.nodes[n]))

    def shortest(self, start: str, goal: str, knowledge: Mapping[str, str], unknown_penalty=1.5):
        """Unknown edges are explicitly returned and never declared safe."""
        frontier = [(0.0, start, [start], [])]
        seen = set()
        while frontier:
            cost, node, path, unknown = heapq.heappop(frontier)
            if node in seen:
                continue
            seen.add(node)
            if node == goal:
                return {'path': path, 'cost': cost, 'unknown_edges': unknown,
                        'safety': 'conditional' if unknown else 'observed_clear'}
            for nxt, edge in self.adjacency[node]:
                state = knowledge.get(edge.id, 'unknown')
                if state in ('blocked', 'hazardous') or nxt in seen:
                    continue
                unobserved = state == 'unknown'
                heapq.heappush(frontier, (cost + edge.length * (unknown_penalty if unobserved else 1),
                                         nxt, path + [nxt], unknown + ([edge.id] if unobserved else [])))
        return {'path': [], 'cost': None, 'unknown_edges': [], 'safety': 'unavailable'}

    def action_distribution(self, start: str, goal: str, knowledge: Mapping[str, str], beta: float):
        if beta < 0 or not math.isfinite(beta):
            raise ValueError('Rationality beta must be finite and nonnegative')
        costs = {}
        # Exclude backtracking through the present node when evaluating a next edge.
        for nxt, edge in self.adjacency[start]:
            if knowledge.get(edge.id) in ('blocked', 'hazardous'):
                continue
            remainder = dict(knowledge)
            for _, other in self.adjacency[start]:
                remainder[other.id] = 'blocked'
            route = self.shortest(nxt, goal, remainder, unknown_penalty=1)
            if route['cost'] is not None:
                costs[nxt] = edge.length + route['cost']
        if not costs:
            return {}
        best = min(costs.values())
        weights = {n: math.exp(-beta * (c - best)) for n, c in costs.items()}
        normalizer = sum(weights.values())
        return {n: w / normalizer for n, w in weights.items()}


def sample_action(probabilities: Mapping[str, float], rng: random.Random):
    if not probabilities:
        return None
    return rng.choices(list(probabilities), weights=list(probabilities.values()), k=1)[0]


class BeliefFilter:
    """Two-state HMM: aware of a specific change versus outdated safe belief.

    The prior represents uncertainty, not a third hidden ground-truth value.
    update_action accepts observable choices and a robot-observed hazard only.
    There is deliberately no true-human-belief argument.
    """
    def __init__(self, prior_unaware=0.5, spontaneous_awareness=0.02, forgetting=0.005,
                 assumed_beta=1.2, lapse_probability=0.05):
        for p in (prior_unaware, spontaneous_awareness, forgetting, lapse_probability):
            if not 0 <= p <= 1:
                raise ValueError('Probabilities must lie in [0,1]')
        self.unaware = prior_unaware
        self.spontaneous_awareness = spontaneous_awareness
        self.forgetting = forgetting
        self.beta = assumed_beta
        self.lapse = lapse_probability

    def predict(self, steps=1, warning_compliance=0.0):
        if steps < 0 or not 0 <= warning_compliance <= 1:
            raise ValueError('Invalid transition')
        for _ in range(steps):
            self.unaware = (self.unaware * (1 - self.spontaneous_awareness)
                            + (1 - self.unaware) * self.forgetting)
        self.unaware *= 1 - warning_compliance

    def update_action(self, graph: RouteGraph, origin: str, choice: str, goal: str, observed_edge: str):
        base = {eid: 'clear' for eid in graph.edges}
        aware = {**base, observed_edge: 'blocked'}
        unaware_dist = graph.action_distribution(origin, goal, base, self.beta)
        aware_dist = graph.action_distribution(origin, goal, aware, self.beta)
        choices = len(graph.adjacency[origin])
        floor = self.lapse / max(1, choices)
        p_u = floor + (1 - self.lapse) * unaware_dist.get(choice, 0)
        p_a = floor + (1 - self.lapse) * aware_dist.get(choice, 0)
        evidence = self.unaware * p_u + (1 - self.unaware) * p_a
        if evidence > 0:
            self.unaware = self.unaware * p_u / evidence
        return self.unaware

    @property
    def entropy(self):
        return -sum(p * math.log2(p) for p in (self.unaware, 1-self.unaware) if p > 0)


@dataclass(frozen=True)
class WarningCosts:
    missed: float = 30.0
    false_alarm: float = 4.0
    soft_interruption: float = 0.4
    explicit_interruption: float = 1.5
    soft_effectiveness: float = 0.35
    explicit_effectiveness: float = 0.85


def action_costs(p_mismatch: float, costs=WarningCosts()):
    if not 0 <= p_mismatch <= 1:
        raise ValueError('Mismatch must be a probability')
    return {
        'silent': p_mismatch * costs.missed,
        'soft': costs.soft_interruption + (1-p_mismatch) * costs.false_alarm * 0.5
                + p_mismatch * costs.missed * (1-costs.soft_effectiveness),
        'explicit': costs.explicit_interruption + (1-p_mismatch) * costs.false_alarm
                + p_mismatch * costs.missed * (1-costs.explicit_effectiveness),
    }


class WarningPolicy:
    def __init__(self, baseline='B4', costs=WarningCosts(), proximity=4.0, cooldown=10.0):
        if baseline not in ('B1', 'B2', 'B3', 'B4', 'oracle'):
            raise ValueError('Unknown baseline')
        self.baseline, self.costs, self.proximity, self.cooldown = baseline, costs, proximity, cooldown
        self.last_warning = {}
        self.seen = set()

    def choose(self, hazard_id, stamp, p_unaware, relevant, distance, oracle_unaware=None):
        if self.baseline == 'oracle' and oracle_unaware is None:
            raise ValueError('Oracle explicitly requires evaluator-only human belief')
        if not relevant or self.baseline == 'B1':
            return 'silent'
        if stamp - self.last_warning.get(hazard_id, -math.inf) < self.cooldown:
            return 'silent'
        if self.baseline == 'B2':
            action = 'explicit' if distance <= self.proximity else 'silent'
        elif self.baseline == 'B3':
            action = 'explicit' if hazard_id not in self.seen else 'silent'
        else:
            risk = float(oracle_unaware) if self.baseline == 'oracle' else p_unaware
            action = min(action_costs(risk, self.costs), key=action_costs(risk, self.costs).get)
        if action != 'silent':
            self.last_warning[hazard_id] = stamp
            self.seen.add(hazard_id)
        return action


class BeliefMDP:
    """Small communication MDP, NOT a movement or environment simulator.

    State is discretized mismatch probability. Outcomes approximate information
    gain and compliance; these assumptions must be calibrated before conclusions.
    """
    actions = ('silent', 'soft', 'explicit', 'observe')

    def __init__(self, bins=11, discount=0.9, costs=WarningCosts()):
        if bins < 2 or not 0 <= discount < 1:
            raise ValueError('MDP needs >=2 bins and discount in [0,1)')
        self.bins, self.discount, self.costs = bins, discount, costs

    def transitions(self, state, action):
        p = state / (self.bins - 1)
        if action == 'observe':
            return {0: 1-p, self.bins-1: p}
        effective = {'silent': 0, 'soft': self.costs.soft_effectiveness,
                     'explicit': self.costs.explicit_effectiveness}[action]
        successor = round(p * (1-effective) * (self.bins-1))
        return {successor: 1.0}

    def cost(self, state, action):
        return 1.0 if action == 'observe' else action_costs(state / (self.bins-1), self.costs)[action]

    def value_iteration(self, tolerance=1e-8, max_iterations=10000):
        values = [0.0] * self.bins
        policy = {}
        for iteration in range(max_iterations):
            new = []
            for state in range(self.bins):
                q = {action: self.cost(state, action) + self.discount * sum(
                     p*values[nxt] for nxt, p in self.transitions(state, action).items()) for action in self.actions}
                policy[state] = min(q, key=q.get)
                new.append(q[policy[state]])
            delta = max(abs(a-b) for a,b in zip(new, values))
            values = new
            if delta < tolerance:
                return {'values': values, 'policy': policy, 'iterations': iteration+1, 'converged': True}
        return {'values': values, 'policy': policy, 'iterations': max_iterations, 'converged': False}

    def q_learning(self, seed=0, episodes=4000, steps=30, alpha=0.15, epsilon=0.2):
        rng = random.Random(seed)
        q = {(s,a): 0.0 for s in range(self.bins) for a in self.actions}
        for _ in range(episodes):
            state = rng.randrange(self.bins)
            for _ in range(steps):
                action = rng.choice(self.actions) if rng.random() < epsilon else min(self.actions, key=lambda a:q[state,a])
                transitions = self.transitions(state, action)
                nxt = rng.choices(list(transitions), weights=list(transitions.values()))[0]
                target = self.cost(state, action) + self.discount*min(q[nxt,a] for a in self.actions)
                q[state,action] += alpha*(target-q[state,action])
                state = nxt
        return {'policy': {s:min(self.actions,key=lambda a:q[s,a]) for s in range(self.bins)},
                'q': {str(s):{a:q[s,a] for a in self.actions} for s in range(self.bins)}, 'seed': seed}
