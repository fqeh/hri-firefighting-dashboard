#!/usr/bin/env python3
"""Record the actual installed runtime alongside an experiment, inside ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
from datetime import datetime, timezone

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=root / 'results/runtime_manifest.json')
args = parser.parse_args()
def command(args):
    result = subprocess.run(args, capture_output=True, text=True, check=True)
    return result.stdout.strip()

sources = {}
for directory in ('ros2_ws/src', 'evaluation', 'scripts', 'docker', 'dashboard/src'):
    for path in (root / directory).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            sources[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
for filename in ('compose.yaml', 'compose.gui.yaml', 'dashboard/package.json',
                 'dashboard/package-lock.json', 'dashboard/vite.config.ts'):
    path = root / filename
    if path.is_file():
        sources[filename] = hashlib.sha256(path.read_bytes()).hexdigest()
manifest = {
    'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
    'platform': platform.platform(), 'architecture': platform.machine(),
    'os_release': Path('/etc/os-release').read_text(),
    'python': platform.python_version(),
    'gazebo_sim': command(['gz', 'sim', '--versions']),
    'packages': command(['dpkg-query', '-W', '-f=${Package}\t${Version}\n']),
    'source_sha256': sources,
}
bridge=root/'.runtime/bridge_ws/install/hri_bridge_revision.txt'
if bridge.exists():manifest['ros_gz_bridge_overlay_commit']=bridge.read_text().strip()
output = args.output
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(manifest, indent=2) + '\n')
print(output)
