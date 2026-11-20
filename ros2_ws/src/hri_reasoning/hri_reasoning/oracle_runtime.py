"""Explicitly privileged evaluation baseline. Never launched for B1–B4."""
from .rescue_runtime import WarningDecision,run
from .tom import canonical


class OracleWarning(WarningDecision):
    def __init__(self):
        super().__init__()
        if self.baseline!='oracle':raise ValueError('Oracle executable requires policy:=oracle')
        self.private=None
        self.subscribe_json('/evaluation/human_belief',self.truth)

    def truth(self,data):
        if 'knowledge' in data:self.private=data['knowledge']

    def estimate(self,hazard,belief):
        if self.private is None:return None
        incompatible=('open_safe','open_hazardous') if hazard.get('state')=='blocked' else ('open_safe',)
        return float(canonical(self.private[hazard['edge']]) in incompatible)


def main():run(OracleWarning)
