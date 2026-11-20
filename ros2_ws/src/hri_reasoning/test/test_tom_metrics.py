import sys
from pathlib import Path
import pytest

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'evaluation/tom_metrics.py').is_file())
sys.path.insert(0,str(ROOT/'evaluation'))
from tom_metrics import score_samples, evaluate


def test_analytic_multiclass_scores_and_undefined_denominators():
    assert score_samples([])['mismatch_precision'] is None
    result=score_samples([{'posterior':{'open_safe':.7,'blocked':.1,'open_hazardous':.1,'unknown_stale':.1},
        'true_hypothesis':'open_safe','p_mismatch':.7,'dangerous_mismatch':True}])
    assert result['classification_accuracy']==1
    assert result['mean_true_hypothesis_probability']==.7
    assert result['multiclass_brier']==pytest.approx(.12)
    assert result['expected_calibration_error']==pytest.approx(.3)
    assert result['mismatch_precision']==result['mismatch_recall']==1


def test_missing_observation_does_not_erase_false_negative():
    result=score_samples([{'posterior':{'open_safe':1.,'blocked':0.,'open_hazardous':0.,'unknown_stale':0.},
        'true_hypothesis':'open_safe','p_mismatch':None,'dangerous_mismatch':True}])
    assert result['fn']==result['unassessed_hazard_samples']==1
    assert result['mismatch_recall']==0


def test_evaluation_does_not_use_future_true_belief():
    def row(topic,t,**data): return {'topic':topic,'stamp':t,'data':{'stamp':t,**data}}
    records=[row('/evaluation/truth',1,event='hazard_created',hazard={'edge':'trail_a','position':[7,-2,0]}),
        row('/evaluation/human_belief',1,knowledge={'trail_a':'clear'}),
        row('/hri/human_intent',1,path=['fork','trail_a','safety']),
        row('/hri/belief',2,beliefs={'trail_a':{'posterior':{'open_safe':1.,'blocked':0.,'open_hazardous':0.,'unknown_stale':0.},'p_mismatch':1.}}),
        row('/evaluation/human_belief',3,knowledge={'trail_a':'blocked'})]
    result=evaluate(records)
    assert result['classification_accuracy']==1
    assert result['tp']==1
