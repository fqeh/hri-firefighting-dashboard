#!/usr/bin/env python3
"""Live backend/control integration check. Requires a running industrial Nav2 demo."""
import argparse
import json
import math
from pathlib import Path
import time
from urllib.request import Request, urlopen

parser = argparse.ArgumentParser()
parser.add_argument('--base', default='http://127.0.0.1:8080')
parser.add_argument('--output', type=Path, default=Path('results/dashboard-integration.json'))
parser.add_argument('--x', type=float, default=10.0)
parser.add_argument('--y', type=float, default=-2.0)
args = parser.parse_args()

def state():
    with urlopen(args.base + '/api/state', timeout=5) as response:
        return json.load(response)

def command(data):
    request = Request(args.base + '/api/control', data=json.dumps(data).encode(),
                      headers={'Content-Type': 'application/json'})
    with urlopen(request, timeout=5) as response:
        assert response.status == 202
        return json.load(response)

def sim_time(snapshot):
    return snapshot['topics']['/clock']['data']['seconds']

evidence = {'status': 'failed', 'source': 'live ROS backend, Gazebo, Nav2'}
try:
    before = state()
    assert before['connected'], 'simulator is not connected'
    for topic in ('/robot/odom', '/human/odom', '/robot/scan', '/hri/routes', '/hri/belief'):
        assert topic in before['topics'] and not before['topics'][topic]['stale'], f'{topic} unavailable'
    with urlopen(args.base + '/api/camera.png', timeout=5) as response:
        png = response.read()
    assert png.startswith(b'\x89PNG\r\n\x1a\n'), 'camera is not a PNG'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix('.png').write_bytes(png)
    command({'command': 'pause'})
    time.sleep(1)
    paused_first = sim_time(state())
    time.sleep(1)
    paused_second = sim_time(state())
    assert abs(paused_second - paused_first) < .05, 'pause did not stop physics clock'
    command({'command': 'resume'})
    time.sleep(2)
    resumed = state()
    assert sim_time(resumed) > paused_second + .2, 'resume did not restart physics clock'
    evidence.update(camera_png_bytes=len(png), paused_sim_time=paused_first,
                    resumed_sim_time=sim_time(resumed))
    command({'command': 'waypoint', 'x': args.x, 'y': args.y})
    trace = []
    deadline = time.monotonic() + 120
    started = False
    while time.monotonic() < deadline:
        snapshot = state()
        nav = snapshot['topics'].get('/hri/navigation', {})
        data = nav.get('data', {})
        if data.get('state') in ('requested', 'navigating') and not nav.get('stale', True):
            started = True
        position = snapshot['topics']['/robot/odom']['data']['position']
        trace.append({'sim_time': sim_time(snapshot), 'position': position, 'navigation': data.get('state')})
        if started and data.get('state') == 'succeeded':
            assert math.dist(position[:2], [args.x, args.y]) < .4, 'goal result disagrees with odometry'
            evidence.update(status='passed', goal=[args.x, args.y], final_position=position, trace=trace)
            break
        if started and data.get('state') in ('failed', 'rejected', 'unavailable'):
            raise AssertionError(f'Nav2 goal failed: {data}')
        time.sleep(.25)
    else:
        raise AssertionError('Nav2 goal timed out')
except Exception as error:
    evidence['error'] = str(error)
    if 'trace' in globals():
        evidence['trace'] = trace
    # Ensure a failed pause check never leaves the user's simulation paused.
    try:
        command({'command': 'resume'})
    except Exception:
        pass
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(evidence, indent=2) + '\n')
print(json.dumps({k: v for k, v in evidence.items() if k != 'trace'}, indent=2))
raise SystemExit(0 if evidence['status'] == 'passed' else 1)
