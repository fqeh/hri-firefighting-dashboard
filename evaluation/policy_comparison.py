#!/usr/bin/env python3
"""Policy-model comparison only; this is NOT a Gazebo outcome experiment."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/hri_reasoning'))
from hri_reasoning.core import BeliefMDP,action_costs

parser=argparse.ArgumentParser()
parser.add_argument('--seed',type=int,default=42)
parser.add_argument('--output',type=Path,default=Path('results/policy_model.json'))
args=parser.parse_args()
model=BeliefMDP()
vi=model.value_iteration()
q=model.q_learning(seed=args.seed)
result={'kind':'communication_model_analysis_not_simulation_results','seed':args.seed,
        'value_iteration':vi,'q_learning':q,
        'cost_sensitive':{s:min(action_costs(s/(model.bins-1)),key=action_costs(s/(model.bins-1)).get) for s in range(model.bins)}}
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(result,indent=2)+'\n')
print(args.output)
