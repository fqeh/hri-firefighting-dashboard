"""Explicit privileged baseline only; never loaded by the normal operator policy."""
from hri_reasoning.oracle_runtime import OracleWarning
from hri_reasoning.lifecycle import run
from .operator_inference import OperatorWarningDecision


class OperatorOracle(OracleWarning,OperatorWarningDecision):
    pass


def main():run(OperatorOracle)
