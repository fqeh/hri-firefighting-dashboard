#!/usr/bin/env python3
"""Isolated smoke-test process group with bounded, reliable cleanup."""
import os
import signal
import subprocess

env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('HRI_SMOKE_DOMAIN', '43'),
           GZ_PARTITION=f'hri_smoke_{os.getpid()}')
with open('results/smoke_simulator.log', 'w') as log:
    simulator = subprocess.Popen([
        'ros2', 'launch', 'hri_bringup', 'demo.launch.py', 'scenario:=minimal',
        'dashboard:=false', 'reasoning:=false'], env=env,
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        result = subprocess.run(['ros2', 'run', 'hri_bringup', 'verify_telemetry',
                                 '--output', 'results/telemetry.json', '--timeout', '120'],
                                env=env, timeout=180)
        code = result.returncode
    finally:
        for sig, timeout in [(signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)]:
            if simulator.poll() is not None:
                break
            if sig == signal.SIGINT:
                simulator.send_signal(sig)
            else:
                os.killpg(simulator.pid, sig)
            try:
                simulator.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
    raise SystemExit(code)
