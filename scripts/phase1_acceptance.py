#!/usr/bin/env python3
"""Real simulator acceptance: isolated fresh launches, bounded cleanup, saved evidence.

Run inside the built ROS runtime. This does not replace the separate GUI screenshot
check: it verifies physical dynamics, telemetry, TF, stop, collisions and mission.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def terminate(process):
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        if process.poll() is not None:
            return
        # ros2 launch forwards SIGINT to its own children. Signaling the entire
        # group here as well delivers two interrupts and corrupts clean teardown.
        if sig == signal.SIGINT:
            process.send_signal(sig)
        else:
            os.killpg(process.pid, sig)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue


def run_case(name, output, env):
    launch = ['ros2', 'launch', 'hri_bringup', 'demo.launch.py', 'scenario:=minimal',
              'dashboard:=false', 'reasoning:=false']
    if name == 'mission':
        launch += ['mission:=true', f'trajectory_path:={output / "mission_odom.csv"}']
    probe = ['ros2', 'run', 'hri_bringup', 'verify_telemetry', f'--{name}',
             '--timeout', '180', '--output', str(output / f'{name}.json'),
             '--trajectory', str(output / f'{name}_physical.csv')]
    record = {'launch_command': launch, 'probe_command': probe,
              'ROS_DOMAIN_ID': env['ROS_DOMAIN_ID'], 'GZ_PARTITION': env['GZ_PARTITION']}
    started = time.monotonic()
    with (output / f'{name}_simulator.log').open('w') as log:
        simulator = subprocess.Popen(launch, env=env, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        verifier = None
        try:
            with (output / f'{name}_probe.log').open('w') as probe_log:
                verifier = subprocess.Popen(probe, env=env, stdout=probe_log,
                                            stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    record['probe_exit_code'] = verifier.wait(timeout=300)
                except subprocess.TimeoutExpired:
                    record['probe_exit_code'] = 'timeout'
        finally:
            if verifier is not None:
                terminate(verifier)
            terminate(simulator)
    record['launch_exit_code'] = simulator.returncode
    record['wall_seconds'] = time.monotonic() - started
    log_text = (output / f'{name}_simulator.log').read_text()
    critical = re.compile(r'\[Err\]|\[ERROR\]|\bFATAL\b|Traceback|Segmentation fault|'
                          r'process has died|terminate called|Aborted \(core dumped\)')
    record['critical_log_lines'] = [line for line in log_text.splitlines() if critical.search(line)]
    record['warnings'] = [line for line in log_text.splitlines() if 'warn' in line.lower()]
    record['status'] = ('passed' if record['probe_exit_code'] == 0 and
                        not record['critical_log_lines'] and simulator.returncode == 0 else 'failed')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('results/phase1'))
    args = parser.parse_args()
    if (args.output / 'acceptance.json').exists():
        parser.error('output contains an existing acceptance result; use a fresh --output directory')
    args.output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('HRI_ACCEPTANCE_DOMAIN', '44'),
               GZ_PARTITION=f'hri_foundation_{os.getpid()}')
    results = {}
    for case in ('foundation', 'mission'):
        results[case] = run_case(case, args.output, env)
        print(f'{case}: {results[case]["status"]}', flush=True)
    passed = all(case['status'] == 'passed' for case in results.values())
    report = {'status': 'passed' if passed else 'failed',
              'scope': 'headless physical foundation; GUI visibility recorded separately', 'cases': results}
    (args.output / 'acceptance.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
