#!/usr/bin/env bash
# Pinned upstream Jazzy backport fixes bridge ownership/shutdown races (#955/#991).
set -eo pipefail
cd "$(dirname "$0")/.."
bridge_revision=95e6fdea72fb57ef55a4136292dfa62370ab3fb9
bridge_archive=.runtime/downloads/ros_gz-95e6fde.tar.gz
mkdir -p .runtime/downloads .runtime/bridge_ws/src
if [[ ! -f "$bridge_archive" ]]; then
  curl -fL --retry 3 "https://codeload.github.com/gazebosim/ros_gz/tar.gz/$bridge_revision" -o "$bridge_archive"
fi
python3 - <<'PY'
import hashlib,tarfile
from pathlib import Path
archive=Path('.runtime/downloads/ros_gz-95e6fde.tar.gz')
expected='a8dedb29ee8abf147e8067f9ba4c7817139a871f00de16246dda44adeb2b14c2'
if hashlib.sha256(archive.read_bytes()).hexdigest()!=expected:raise SystemExit('Bridge archive SHA-256 mismatch')
destination=Path('.runtime/bridge_ws/src/ros_gz_bridge')
with tarfile.open(archive) as tar:
    for member in tar.getmembers():
        parts=Path(member.name).parts
        if len(parts)<3 or parts[1]!='ros_gz_bridge' or not member.isfile():continue
        relative=Path(*parts[2:])
        if '..' in relative.parts:raise SystemExit('Unexpected archive path')
        target=destination/relative;target.parent.mkdir(parents=True,exist_ok=True)
        content=tar.extractfile(member).read()
        if not target.exists() or target.read_bytes()!=content:target.write_bytes(content)
PY
source /opt/ros/jazzy/setup.bash
cd .runtime/bridge_ws
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-select ros_gz_bridge \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
printf '%s\n' "$bridge_revision" > install/hri_bridge_revision.txt
