#!/usr/bin/env python3
"""Combine retained trial evidence with an explicit corrected-condition retest.

Never rewrite a failed run. Every selected result keeps its source path.
"""
import argparse,json
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--suite',type=Path,required=True)
p.add_argument('--warning-retest',type=Path,required=True)
p.add_argument('--gui',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
suite=json.loads((a.suite/'summary.json').read_text())
retry=json.loads((a.warning_retest/'summary.json').read_text())
gui=json.loads((a.gui/'result.json').read_text())
conditions=dict(suite['conditions']);conditions['6_warning']=retry['conditions']['6_warning']
sources={name:str((a.warning_retest if name=='6_warning' else a.suite)/name) for name in conditions}
passed=(all(r['status']=='passed' for r in conditions.values()) and
        suite['matched_comparison']['passed'] and gui['status']=='passed')
report={'status':'passed' if passed else 'failed','condition_sources':sources,
    'conditions':conditions,'matched_comparison':suite['matched_comparison'],'gui':gui,
    'provenance':'The first suite warning delivery failed and is retained unchanged. Condition 6 is explicitly replaced by its reliable-delivery retest; other conditions were not rerun after the test-publisher-only fix.'}
a.output.mkdir(parents=True,exist_ok=False)
(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
rows=['| Condition | Accuracy | Mean P(true) | Brier | ECE | Mismatch precision | Recall |',
      '|---|---:|---:|---:|---:|---:|---:|']
def cell(value):return 'undefined' if value is None else f'{value:.3f}'
for name,result in conditions.items():
    m=result['metrics']
    rows.append('| '+name+' | '+' | '.join(cell(m[k]) for k in ('classification_accuracy',
        'mean_true_hypothesis_probability','multiclass_brier','expected_calibration_error',
        'mismatch_precision','mismatch_recall'))+' |')
(a.output/'metrics.md').write_text('\n'.join(rows)+'\n\nDescriptive time-sample scores, not independent subjects or a performance claim.\n')
print(report['status']);print('\n'.join(rows))
raise SystemExit(0 if passed else 1)
