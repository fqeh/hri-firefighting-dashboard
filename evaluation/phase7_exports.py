#!/usr/bin/env python3
"""Export reproducible Phase 7 analyses from archived real trial logs.

The script does not launch simulations or synthesize trial measurements.
The cost-threshold plot is explicitly analytical and uses the configured cost
equation; unavailable metrics remain null and are called out in the report.
"""
import csv
import html
import json
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'results'
OUT = RESULTS / 'phase7-final'
POLICIES = ('B1', 'B2', 'B3', 'B4', 'oracle')
COLORS = {'B1': '#667085', 'B2': '#d49b28', 'B3': '#c34a36',
          'B4': '#16836b', 'oracle': '#536fc0'}


def read_json(path):
    return json.loads(path.read_text())


def average(rows, field):
    values = [float(row[field]) for row in rows
              if isinstance(row.get(field), (int, float, bool))]
    return statistics.mean(values) if values else None


def paired_differences(runs):
    rows = []
    summary = {}
    metrics = ('interruptions', 'time_to_safety_s', 'hazard_exposure_s', 'mission_completed')
    for scenario in ('forest', 'industrial'):
        for baseline in ('B1', 'B2', 'B3', 'oracle'):
            deltas = {metric: [] for metric in metrics}
            for seed in sorted({r['seed'] for r in runs if r['scenario'] == scenario}):
                by_policy = {r['policy']: r for r in runs
                             if r['scenario'] == scenario and r['seed'] == seed
                             and r['policy'] in ('B4', baseline)}
                if 'B4' not in by_policy or baseline not in by_policy:
                    continue
                for metric in metrics:
                    b4 = by_policy['B4'].get(metric)
                    reference = by_policy[baseline].get(metric)
                    delta = (float(b4) - float(reference)
                             if isinstance(b4, (int, float, bool))
                             and isinstance(reference, (int, float, bool)) else None)
                    rows.append({'scenario': scenario, 'seed': seed, 'contrast': f'B4-{baseline}',
                                 'metric': metric, 'b4': b4, 'baseline': reference, 'delta': delta})
                    if delta is not None:
                        deltas[metric].append(delta)
            summary[f'{scenario}/B4-{baseline}'] = {
                metric: {'n_pairs': len(values),
                         'mean_delta': statistics.mean(values) if values else None,
                         'sample_sd': statistics.stdev(values) if len(values) > 1 else None,
                         'range': [min(values), max(values)] if values else None}
                for metric, values in deltas.items()}
    return rows, summary


def save_svg(filename, title, subtitle, body, height=610):
    content = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1040" height="{height}" viewBox="0 0 1040 {height}">
<rect width="100%" height="100%" fill="#fbfcfd"/>
<text x="42" y="46" font-family="Arial,sans-serif" font-size="24" font-weight="700" fill="#172b3a">{html.escape(title)}</text>
<text x="42" y="73" font-family="Arial,sans-serif" font-size="13" fill="#506273">{html.escape(subtitle)}</text>
{body}
</svg>'''
    (OUT / filename).write_text(content)


def label(x, y, text, size=12, color='#334155', anchor='start', weight='400'):
    return (f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" '
            f'font-family="Arial,sans-serif" font-size="{size}" font-weight="{weight}" '
            f'fill="{color}">{html.escape(str(text))}</text>')


def rect(x, y, w, h, color, rx=2):
    return f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" fill="{color}"/>'


def draw_policy_rates(runs):
    body = []
    means = {}
    for policy in POLICIES:
        rows = [r for r in runs if r['policy'] == policy]
        means[policy] = {k: (average(rows, k), sum(r.get(k) is not None for r in rows))
                         for k in ('warning_precision', 'warning_recall')}
    chart_x, chart_y, chart_w, chart_h = 110, 125, 850, 370
    for tick in range(0, 6):
        y = chart_y + chart_h * tick / 5
        body.append(f'<path d="M{chart_x} {y:.1f}H{chart_x + chart_w}" stroke="#d8e0e7"/>')
        body.append(label(chart_x - 14, y + 4, f'{1-tick/5:.1f}', 11, anchor='end'))
    body.append(label(48, 304, 'Rate', 12))
    for i, policy in enumerate(POLICIES):
        center = chart_x + chart_w * (i + .5) / len(POLICIES)
        for j, key in enumerate(('warning_precision', 'warning_recall')):
            value, n = means[policy][key]
            x = center - 25 + j * 27
            if value is not None:
                body.append(rect(x, chart_y + chart_h * (1 - value), 21, chart_h * value,
                                 '#243f5a' if j == 0 else '#38a58b'))
                body.append(label(x + 10, chart_y + chart_h * (1-value)-8, f'{value:.2f}', 10, anchor='middle'))
            else:
                body.append(label(x + 10, chart_y + chart_h + 18, 'n/a', 9, '#7b8794', anchor='middle'))
        body.append(label(center, chart_y + chart_h + 42, policy, 13, COLORS[policy], 'middle', '700'))
        body.append(label(center, chart_y + chart_h + 60,
                          f'P n={means[policy]["warning_precision"][1]} / R n={means[policy]["warning_recall"][1]}',
                          9, '#64748b', 'middle'))
    body.append(rect(380, 520, 14, 14, '#243f5a'))
    body.append(label(400, 532, 'Precision', 12))
    body.append(rect(500, 520, 14, 14, '#38a58b'))
    body.append(label(520, 532, 'Recall', 12))
    body.append(label(42, 574, 'Oracle uses privileged simulated belief; it is a reference, not a realizable robot policy.', 11))
    save_svg('01-warning-precision-recall.svg', 'Warning precision and recall',
             'Descriptive means across the archived 20-run Gazebo matrix; undefined denominators are not plotted.', ''.join(body))


def draw_error_bars(runs):
    body = []
    metrics = ('false_alarms', 'missed_warnings')
    names = ('False warnings / trial', 'Missed warnings / trial')
    chart_x, chart_y, chart_w, chart_h = 120, 130, 840, 330
    for tick in range(0, 6):
        value = tick / 5
        y = chart_y + chart_h * (1-value)
        body.append(f'<path d="M{chart_x} {y:.1f}H{chart_x+chart_w}" stroke="#d8e0e7"/>')
        body.append(label(chart_x-15, y+4, f'{value:.1f}', 11, anchor='end'))
    for i, policy in enumerate(POLICIES):
        rows = [r for r in runs if r['policy'] == policy]
        center = chart_x + chart_w * (i + .5) / len(POLICIES)
        for j, metric in enumerate(metrics):
            value = average(rows, metric)
            x = center - 23 + j * 25
            body.append(rect(x, chart_y + chart_h*(1-value), 20, chart_h*value, '#df8d3c' if j == 0 else '#7957a8'))
            body.append(label(x+10, chart_y+chart_h*(1-value)-8, f'{value:.2f}', 10, anchor='middle'))
        body.append(label(center, chart_y+chart_h+27, policy, 13, COLORS[policy], 'middle', '700'))
    body.append(rect(325, 510, 14, 14, '#df8d3c'));body.append(label(345, 522, names[0], 11))
    body.append(rect(520, 510, 14, 14, '#7957a8'));body.append(label(540, 522, names[1], 11))
    body.append(label(42, 570, 'Each policy mean pools both scenarios and two seeds (n=4). No significance test is implied.', 11))
    save_svg('02-false-alarms-vs-missed.svg', 'False alarms and missed warnings',
             'Per-trial means, pooled descriptively over the two scenarios; maximum possible rate here is 1 per trial.', ''.join(body))


def draw_completion(runs):
    body = []
    scenarios = ('forest', 'industrial')
    chart_x, chart_y, chart_w, chart_h = 115, 135, 850, 330
    for tick in range(6):
        val=tick/5;y=chart_y+chart_h*(1-val)
        body.append(f'<path d="M{chart_x} {y:.1f}H{chart_x+chart_w}" stroke="#d8e0e7"/>')
        body.append(label(chart_x-14,y+4,f'{val:.1f}',11,anchor='end'))
    for i, policy in enumerate(POLICIES):
        center=chart_x+chart_w*(i+.5)/len(POLICIES)
        for j, scenario in enumerate(scenarios):
            rows=[r for r in runs if r['policy']==policy and r['scenario']==scenario]
            value=average(rows,'mission_completed')
            width=24;x=center-26+j*28
            if value is not None:
                body.append(rect(x,chart_y+chart_h*(1-value),width,chart_h*value,
                                 '#3b82a0' if j==0 else '#84a86a'))
                body.append(label(x+width/2,chart_y+chart_h*(1-value)-8,f'{value:.2f}',9,anchor='middle'))
        body.append(label(center,chart_y+chart_h+27,policy,13,COLORS[policy],'middle','700'))
    body.append(rect(350,510,14,14,'#3b82a0'));body.append(label(370,522,'Forest',11))
    body.append(rect(450,510,14,14,'#84a86a'));body.append(label(470,522,'Industrial',11))
    body.append(label(42,570,'n=2 per policy/scenario; completion is a simulated outcome, not real-world safety evidence.',11))
    save_svg('03-evacuation-completion.svg','Evacuation completion comparison',
             'Fraction of complete valid trials reaching the configured safety goal by the requested duration.', ''.join(body))


def draw_beta(baseline, robustness):
    selected=[r for r in baseline if r['policy']=='B4']
    selected += [r for r in robustness if r['policy']=='B4']
    values={}
    for row in selected: values.setdefault(row['human_beta'],[]).append(row['belief_accuracy'])
    body=[];left,top,w,h=140,150,760,310
    for tick in range(6):
        v=tick/5;y=top+h*(1-v)
        body.append(f'<path d="M{left} {y:.1f}H{left+w}" stroke="#d8e0e7"/>')
        body.append(label(left-15,y+4,f'{v:.1f}',11,anchor='end'))
    betas=sorted(values)
    colors={.2:'#d49b28',1.2:'#16836b',3.0:'#536fc0'}
    xlo,xhi=min(betas),max(betas)
    for beta in betas:
        avg=statistics.mean(values[beta]);x=left+w*((beta-xlo)/(xhi-xlo) if xhi>xlo else .5);y=top+h*(1-avg)
        body.append(f'<circle cx="{x}" cy="{y}" r="8" fill="{colors.get(beta,"#334155")}"/>')
        body.append(label(x,y-15,f'{avg:.3f}',12,anchor='middle',weight='700'))
        body.append(label(x,top+h+25,f'β={beta:g}',12,anchor='middle'))
        body.append(label(x,top+h+45,f'n={len(values[beta])}',10,'#64748b','middle'))
    body.append(label(42,565,'β≠1.2 conditions each contain one B4 seed. β=1.2 combines four baseline B4 runs.',11))
    save_svg('04-belief-accuracy-by-beta.svg','Belief classification accuracy by actual human β',
             'B4 logged posterior classifications; behavior β and assumed inference β are distinct.', ''.join(body))


def draw_lead_time(runs):
    n=sum(int(r.get('lead_time_samples') or 0) for r in runs)
    body=['<rect x="115" y="130" width="850" height="340" fill="#f3f6f8" stroke="#cbd5e1"/>']
    body.append(label(540,230,f'No valid hazard-entry lead-time samples (n={n})',21,'#34495e','middle','700'))
    body.append(label(540,270,'All recorded hazards were detected but not physically entered in this matrix.',14,'#526579','middle'))
    body.append(label(540,303,'Detection-to-exposure lead time is undefined, not zero.',14,'#526579','middle'))
    body.append(label(540,350,'Separate accepted mechanism run: warning occurred 10.5 simulated seconds',13,'#526579','middle'))
    body.append(label(540,375,'after the dangerous mismatch episode began (one seed; not sensor lead time).',13,'#526579','middle'))
    body.append(label(42,565,'Source for the separate warning-delay example: phase4-visible-verified/visible-metrics.json.',11))
    save_svg('05-detection-lead-time.svg','Detection and warning lead-time evidence',
             'Null outcomes are shown explicitly; the separate warning delay has a different start event.', ''.join(body))


def draw_environment(runs):
    body=[];scenarios=('forest','industrial')
    metrics=(('interruptions','Mean warnings per trial',1.0),('mission_completed','Completion fraction',1.0),
             ('time_to_safety_s','Time to safety (s)',70.0),('hazard_exposure_s','Exposure (s)',1.0))
    top=125;panel_h=95
    for idx,(key,title,maxval) in enumerate(metrics):
        y=top+idx*panel_h
        body.append(label(42,y+23,title,13,'#263a4a',weight='700'))
        for j,scenario in enumerate(scenarios):
            rows=[r for r in runs if r['scenario']==scenario]
            value=average(rows,key)
            x=240+j*350
            body.append(label(x,y+21,scenario,10,COLORS['B4']))
            width=270*(value/maxval) if value is not None else 0
            body.append(rect(x,y+30,width,18,'#2b8a78' if scenario=='forest' else '#6383b4'))
            body.append(label(x+width+8,y+44,'—' if value is None else f'{value:.2f}',11))
    body.append(label(42,535,'20 trials total (10 per scenario); two seeds × five policies. Exposure was 0 in all trials.',11))
    body.append(label(42,558,'Bars pool distinct policies and are descriptive, not a policy-adjusted environment effect.',11))
    save_svg('06-forest-vs-industrial.svg','Forest versus industrial trial outcomes',
             'Environment-level descriptive averages from the archived baseline matrix.', ''.join(body), 600)


def draw_cost_sensitivity(config):
    costs=config['costs'];response=config['response']
    missed_default=costs['missed_danger'];false_default=costs['unnecessary_alert']
    eff=response['explicit_effectiveness'];interrupt=costs['explicit_interruption']
    delay=costs['evacuation_delay']*response['explicit_delay_s']
    hazard=costs['hazard_exposure']
    # Analyst-selected grid around the configured point; this is algebraic,
    # not a simulation parameter sweep or empirical outcome.
    missed_values=(5,10,20,30,45,60)
    false_values=(0,2,4,8,16)
    body=[];left,top,w,h=150,140,740,330
    for tick in range(6):
        v=tick/5;y=top+h*(1-v)
        body.append(f'<path d="M{left} {y:.1f}H{left+w}" stroke="#d8e0e7"/>')
        body.append(label(left-12,y+4,f'{v:.1f}',11,anchor='end'))
    for false_cost in false_values:
        points=[]
        for i,missed in enumerate(missed_values):
            # L(silent)=pH; L(explicit)=pH(1-e)+(1-p)F+C.
            threshold=(false_cost+interrupt+delay)/(eff*(missed+hazard)+false_cost)
            x=left+w*i/(len(missed_values)-1);y=top+h*(1-threshold)
            points.append(f'{x:.1f},{y:.1f}')
            if missed==missed_default and false_cost==false_default:
                body.append(f'<circle cx="{x}" cy="{y}" r="7" fill="#e5484d"/>')
        color=('#2563eb','#7c3aed','#dc2626','#059669','#d97706')[false_values.index(false_cost)]
        body.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="3"/>')
        body.append(label(925,top+24*false_values.index(false_cost)+12,
                          f'unnecessary-alert cost={false_cost}',11,color))
    for i,missed in enumerate(missed_values):
        x=left+w*i/(len(missed_values)-1)
        body.append(label(x,top+h+24,str(missed),11,anchor='middle'))
    body.append(label(440,top+h+48,'Missed-danger cost (analyst-selected grid)',12,anchor='middle'))
    body.append(label(45,305,'p*',12))
    body.append(label(42,565,f'Configured point: missed={missed_default}, false={false_default}; explicit effectiveness={eff:.2f}.',11))
    body.append(label(42,585,'Analytical threshold only; chosen grid is not a measured Gazebo trial or prespecified config range.',11))
    save_svg('07-analytic-cost-threshold.svg','Analytical warning cost-threshold sensitivity',
             'Break-even mismatch probability p* for explicit warning versus silence; assumptions fixed from warning_policy.json.', ''.join(body), 620)


def draw_timeline(visible_metrics, snapshots):
    body=[];metrics=visible_metrics
    # Accepted Phase 4 timeline facts are from retained simulation snapshots.
    blocked=read_json(RESULTS/'phase4-visible-verified/blocked.json')
    warning=read_json(RESULTS/'phase4-visible-verified/warning.json')
    acknowledged=read_json(RESULTS/'phase4-visible-verified/acknowledged.json')
    complete=read_json(RESULTS/'phase4-visible-verified/complete.json')
    def clock(state):
        return state.get('topics',{}).get('/clock',{}).get('data',{}).get('seconds')
    points=[
        (clock(blocked),'Hazard observed; public route replanned'),
        (clock(warning),'Confirmed crew action; warning issued'),
        (clock(acknowledged),'Supervisor acknowledgment recorded'),
        (clock(complete),'Crew reached safety; run completed'),
    ]
    points=[p for p in points if isinstance(p[0],(int,float))]
    latest=max([p[0] for p in points]+[float(metrics.get('completion_time_s',0))])
    left,right=130,930;y=280
    body.append(f'<path d="M{left} {y}H{right}" stroke="#8ba0b0" stroke-width="4"/>')
    for i,(stamp,text) in enumerate(points):
        x=left+(right-left)*stamp/latest if latest else left
        body.append(f'<circle cx="{x:.1f}" cy="{y}" r="9" fill="{("#d97706","#16836b","#536fc0","#334155")[i]}"/>')
        upper=i%2==0;label_y=190 if upper else 350
        body.append(f'<path d="M{x:.1f} {y}V{label_y+10 if upper else label_y-30}" stroke="#98a7b5" stroke-dasharray="3 4"/>')
        body.append(label(x,label_y,f'{stamp:.2f}s',12,'#243b53','middle','700'))
        wrapped=text.split('; ')
        for j,line in enumerate(wrapped):
            body.append(label(x,label_y+20+j*17,line,11,'#526579','middle'))
    body.append(label(42,500,f"Actual episode metrics: warning count={metrics['warning_count']}, precision={metrics['warning_precision']},",12))
    body.append(label(42,522,f"recall={metrics['warning_recall']}, exposure={metrics['exposure_seconds']}s, completion={metrics['evacuation_completed']}.",12))
    body.append(label(42,565,'Single accepted combined fire/tree demonstration; not a repeated-seed policy estimate.',11))
    save_svg('08-example-timeline.svg','Recorded belief-aware warning timeline',
             'Selected saved snapshots from the accepted forest fire/tree run; times are Gazebo simulation seconds.', ''.join(body), 610)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    comparison=read_json(RESULTS/'comparison.json')
    runs=[r for r in comparison['runs'] if r['status']=='complete']
    robustness=read_json(RESULTS/'robustness-comparison.json')
    robustness_runs=robustness.get('runs',[])
    if isinstance(robustness_runs,dict): robustness_runs=list(robustness_runs.values())
    robustness_runs=[r for r in robustness_runs if r.get('status')=='complete']
    # Per-episode outcomes: preserve the already validated normalized export.
    with (OUT/'episode-metrics.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(runs[0]) if runs else [])
        writer.writeheader();writer.writerows(runs)
    paired_rows, paired_summary = paired_differences(runs)
    with (OUT/'paired-differences.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=('scenario','seed','contrast','metric','b4','baseline','delta'))
        writer.writeheader();writer.writerows(paired_rows)
    # Per-event export from every archived base-matrix trial, retaining original data.
    event_path=OUT/'per-event.csv';event_count=0;configs=[]
    with event_path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=('source','event_index','topic','stamp','wall_time','data_json'))
        writer.writeheader()
        for path in sorted((RESULTS/'trials').rglob('*.jsonl')):
            metadata=None
            with path.open() as log:
                for idx,line in enumerate(log):
                    if not line.strip():continue
                    record=json.loads(line)
                    data=record.get('data',{})
                    stamp=data.get('stamp',record.get('stamp'))
                    writer.writerow({'source':str(path.relative_to(ROOT)),'event_index':idx,
                        'topic':record.get('topic'),'stamp':stamp,'wall_time':record.get('wall_time'),
                        'data_json':json.dumps(data,separators=(',',':'),sort_keys=True,allow_nan=False)})
                    event_count+=1
                    if record.get('topic')=='run_metadata': metadata=data
            if metadata is not None:
                configs.append({'source':str(path.relative_to(ROOT)),'scenario':metadata.get('scenario'),
                    'seed':metadata.get('seed'),'policy':metadata.get('policy'),
                    'config_sha256':metadata.get('config_sha256'),'metadata':metadata})
    config=read_json(ROOT/'ros2_ws/src/hri_reasoning/config/warning_policy.json')
    (OUT/'experiment-configurations.json').write_text(json.dumps({
        'description':'Actual metadata embedded in archived Gazebo run logs plus the checked-in warning-policy configuration.',
        'default_warning_policy':config,'runs':configs,
        'note':'The baseline matrix has two seeds per policy/scenario. This export does not add parameter conditions.'},
        indent=2,allow_nan=False)+'\n')
    grouped={}
    for scenario in ('forest','industrial'):
        for policy in POLICIES:
            rows=[r for r in runs if r['scenario']==scenario and r['policy']==policy]
            grouped[f'{scenario}/{policy}']={'n':len(rows),
                'means':{key:average(rows,key) for key in ('mission_completed','time_to_safety_s',
                    'hazard_exposure_s','interruptions','false_alarms','missed_warnings',
                    'warning_precision','warning_recall','belief_accuracy','belief_brier_score')}}
    visible=read_json(RESULTS/'phase4-report/visible-metrics.json')
    summary={'source':'Executed Gazebo/ROS trial logs and retained acceptance artifacts.',
        'base_matrix':{'runs':len(runs),'seeds':sorted({r['seed'] for r in runs}),
            'scenarios':sorted({r['scenario'] for r in runs}),'policies':list(POLICIES),
            'runs_per_scenario_policy':2,'grouped':grouped,
            'paired_differences_b4_minus_baselines':paired_summary,
            'uncertainty_note':'Only paired deltas and sample SD/range are descriptive; no confidence intervals or significance tests with n=2.'},
        'robustness_beta_b4':robustness_runs,'phase4_visible_b4_mechanism_demo':visible,
        'detection_lead_time':{'samples':sum(int(r.get('lead_time_samples') or 0) for r in runs),
            'mean_s':average(runs,'detection_lead_time_s'),'interpretation':'undefined: no hazard-entry observations'},
        'statistical_claims':'No significance testing: n=2 per baseline scenario/policy; robustness conditions n=1 each.'}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    draw_policy_rates(runs);draw_error_bars(runs);draw_completion(runs)
    draw_beta(runs,robustness_runs);draw_lead_time(runs);draw_environment(runs)
    draw_cost_sensitivity(config);draw_timeline(visible,None)
    print(f'Phase 7 exports: {len(runs)} episodes, {event_count} logged events, {len(configs)} configurations, 8 figures')


if __name__=='__main__':
    main()
