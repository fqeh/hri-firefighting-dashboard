#!/usr/bin/env python3
"""Generate a small honest engineering report and standard scientific figure."""
import argparse
import json
from pathlib import Path
import statistics

parser = argparse.ArgumentParser()
parser.add_argument('comparison', type=Path)
parser.add_argument('--output', type=Path, default=Path('results/report'))
parser.add_argument('--incident-audit', type=Path)
parser.add_argument('--text-only', action='store_true', help='Write conditioned tables without plotting pooled settings')
args = parser.parse_args()
data = json.loads(args.comparison.read_text())
runs = data['runs']
valid = [r for r in runs if r['status'] == 'complete']
args.output.mkdir(parents=True, exist_ok=True)

def mean(rows, key):
    values = [float(r[key]) for r in rows if r.get(key) is not None]
    return statistics.mean(values) if values else None

def formatted(value):
    return '—' if value is None else f'{value:.2f}'

conditions = {}
for run in valid:
    key = tuple(run.get(k) for k in ('scenario', 'policy', 'human_beta', 'assumed_beta',
               'trust', 'observation_radius', 'passive_map_rate', 'requested_duration_s', 'config_sha256'))
    conditions.setdefault(key, []).append(run)

lines = ['# Measured Gazebo engineering trials', '',
    f'{len(valid)} valid trials; {len(runs)-len(valid)} incomplete/invalid trials retained in the source comparison.', '',
    'Each row is an independently configured condition. Values are means over seeds; missing values are undefined, not zero. '
    'This small engineering matrix validates the pipeline and is not a powered human-factors study.', '',
    '| Scenario | Policy | Human β / assumed β | Trust / radius / map rate | n | Completion | Time to safety (s) | Exposure (s) | Alerts | False alarms | Belief accuracy |',
    '| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
for key, rows in sorted(conditions.items()):
    scenario, policy, beta, assumed, trust, radius, attention, duration, digest = key
    lines.append(f'| {scenario} | {policy} | {beta} / {assumed} | {trust} / {radius} / {attention} | {len(rows)} | '
                 f'{formatted(mean(rows,"mission_completed"))} | {formatted(mean(rows,"time_to_safety_s"))} | '
                 f'{formatted(mean(rows,"hazard_exposure_s"))} | {formatted(mean(rows,"interruptions"))} | '
                 f'{formatted(mean(rows,"false_alarms"))} | {formatted(mean(rows,"belief_accuracy"))} |')
lines += ['', 'Time-to-safety means include successful runs only; inspect completion and censored flags alongside them. '
          'Detection lead times are undefined for hazards never entered. The JSON/CSV retain per-run metrics, sample counts and conditions.', '',
          'Neither a lower alert count nor a high posterior accuracy alone establishes improved safety. '
          'The default human local sensing and passive map viewing can make no-alert runs safe. '
          'Report null or unfavorable results without selecting only favorable seeds.', '']
if valid:
    seeds=sorted({r['seed'] for r in valid})
    durations=sorted({r['requested_duration_s'] for r in valid})
    lines += ['## What these runs show', '',f'Seeds: {seeds}. Requested simulation durations: {durations} s.', '']
    if all(r.get('mission_completed') is True for r in valid) and all(r.get('hazard_exposure_s')==0 for r in valid):
        lines += ['Every recorded mission completed with zero measured hazard exposure. '
                  'This matrix therefore provides no evidence of a safety improvement for B4.', '']
    for key,rows in sorted(conditions.items()):
        if key[1]!='B4': continue
        for baseline in ('B1','B3'):
            reference=conditions.get((key[0],baseline,*key[2:]))
            if reference:
                warning_delta=mean(rows,'interruptions')-mean(reference,'interruptions')
                current_time,reference_time=mean(rows,'time_to_safety_s'),mean(reference,'time_to_safety_s')
                time_delta=None if current_time is None or reference_time is None else current_time-reference_time
                lines.append(f'- {key[0]}, B4 minus {baseline}: {warning_delta:+.2f} warnings/trial; '
                             f'{"undefined" if time_delta is None else f"{time_delta:+.2f}"} s mean time to safety '
                             f'({len(rows)} B4 runs and {len(reference)} {baseline} runs).')
    lines += ['', 'These descriptive differences are not significance tests. This sample size and the stated model assumptions '
              'limit effectiveness claims. Earlier warnings can induce extra replanning or travel in this controller. '
              'The experiment records modeled response behavior, not human-subject trust or compliance.', '']
    if any(r['policy']=='B1' for r in valid):
        lines += ['Local sensing and passive map attention can let B1 avoid the route obstruction without a proactive warning.', '']
    if all(r.get('eligible_mismatch_incidents')==1 for r in valid):
        lines += ['Every trial contained one recorded dangerous-mismatch opportunity. This narrow condition '
                  'does not thoroughly test suppression of unnecessary alerts to already-informed humans; '
                  'future preregistered scenarios should vary human awareness and route relevance explicitly.', '']
    if all(r.get('human_beta')==r.get('assumed_beta') for r in valid):
        lines += ['The recorded matrix uses matched actual and assumed rationality. Incorrect-assumption robustness '
                  'sweeps are implemented but have not been established by this dataset.', '']
    lines += ['The trial commands used `nav2:=false` and `dashboard:=false`; these trials evaluate the '
              'scripted physical human-response warning loop. Separate integration checks cover the Nav2 operator '
              'and dashboard paths. Interactive features added later are not retroactively included in these results.', '']
if args.incident_audit:
    audit=json.loads(args.incident_audit.read_text())
    created=sum(r['created_count'] for r in audit['runs'])
    scheduled=sum(r['scheduled_count'] for r in audit['runs'])
    detected=sum(r['detected_count'] for r in audit['runs'])
    lines += ['## Incident and provenance audit', '',
              f'The stronger post-run audit passed {audit["passed"]} logs and failed {audit["failed"]}. '
              f'It found {created}/{scheduled} scheduled physical incidents created and {detected} detected. '
              'The audit records each event timestamp and SHA-256 of each original log. '
              'The original run labels and source manifests are preserved.', '',
              'Future validation requires every due physical incident to be created; first-incident detection '
              'remains required as an integration check. Later unobserved incidents are permitted because '
              'frustum-limited perception can legitimately miss them. Existing manifests predate the added '
              'terrain-mesh hashing; future manifests also include OBJ/MTL/DAE/STL and image assets.', '']
(args.output / 'RESULTS.md').write_text('\n'.join(lines))
if args.text_only:
    print(f'Wrote {args.output / "RESULTS.md"}; plots disabled explicitly.')
    raise SystemExit(0)

# Matplotlib is a dependency of this verified ROS image. Import only when making
# the figure so host-side text aggregation remains available without it.
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    print(f'Wrote {args.output / "RESULTS.md"}; run inside the ROS image to create the figure.')
    raise SystemExit(0)

scenarios = sorted({r['scenario'] for r in valid})
if scenarios:
    fig, axes = plt.subplots(len(scenarios), 4, figsize=(16, 3.5*len(scenarios)), squeeze=False)
    policies = ['B1', 'B2', 'B3', 'B4', 'oracle']
    palette = ['#8b98a8', '#d6aa55', '#df795f', '#36a78b', '#627fd3']
    metrics = [('interruptions', 'Warnings per trial'), ('time_to_safety_s', 'Time to safety (s)'), ('hazard_exposure_s', 'Exposure (s)'),
               ('mission_completed', 'Mission completion fraction')]
    for row, scenario in enumerate(scenarios):
        subset = [r for r in valid if r['scenario'] == scenario]
        settings = {tuple(r.get(k) for k in ('human_beta','assumed_beta','trust','observation_radius','passive_map_rate','requested_duration_s','config_sha256')) for r in subset}
        if len(settings) != 1:
            raise SystemExit('Refusing to pool robustness settings in a plot. Create separate comparison files per condition.')
        for col, (metric, title) in enumerate(metrics):
            axis = axes[row, col]
            for index, policy in enumerate(policies):
                values = [float(r[metric]) for r in subset if r['policy'] == policy and r.get(metric) is not None]
                if not values:
                    continue
                axis.bar(index, statistics.mean(values), color=palette[index], width=.65, alpha=.8)
                offsets = [0] if len(values)==1 else [-.12+.24*i/(len(values)-1) for i in range(len(values))]
                axis.scatter([index+o for o in offsets], values, color='#152536', s=22, zorder=3,clip_on=False)
            axis.set_xticks(range(len(policies)), policies)
            axis.set_title(f'{scenario.capitalize()} · {title}')
            axis.spines[['top','right']].set_visible(False)
            axis.grid(axis='y', alpha=.2)
            if metric == 'mission_completed':
                axis.set_ylim(0, 1.1)
            else:
                plotted_values=[float(r[metric]) for r in subset if r.get(metric) is not None]
                axis.set_ylim(0,max(1.0,max(plotted_values,default=0)*1.15))
    fig.suptitle('Actual Gazebo trials · dots are individual seeds; bars are means', fontsize=13)
    fig.tight_layout()
    fig.savefig(args.output/'comparison.png', dpi=180)
    fig.savefig(args.output/'comparison.pdf')
    print(f'Wrote {args.output / "RESULTS.md"}, comparison.png and comparison.pdf')
