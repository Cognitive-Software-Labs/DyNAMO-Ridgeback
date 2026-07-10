#!/usr/bin/env bash
# Create/refresh isaac_venv with Isaac Sim 6.0 (pip install, ~30-50 GB).
#
# Mirrors the perception_venv pattern (README "G1 perception venv"):
# system-site-packages venv + ros2.pth so the Isaac ROS 2 bridge links the
# system ROS Jazzy / CycloneDDS stack instead of its bundled libraries.
#
# Usage: tools/install_isaac_venv.sh [--warmup]
#   --warmup  after install, boot SimulationApp headless once to bake the
#             RTX shader cache (first boot takes minutes; subsequent
#             launches then start fast enough for the bringup gates).
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
VENV="$ROOT/isaac_venv"
MIN_DRIVER="580.95.05"
MIN_FREE_VRAM_MB=16000

# --- preflight: driver + free VRAM (shared box: warn, don't block) ---------
if command -v nvidia-smi >/dev/null; then
    driver="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
    if [ "$(printf '%s\n' "$MIN_DRIVER" "$driver" | sort -V | head -1)" != "$MIN_DRIVER" ]; then
        echo "WARNING: driver $driver < required $MIN_DRIVER for Isaac Sim 6.0" >&2
    fi
    free_vram="$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)"
    if [ "$free_vram" -lt "$MIN_FREE_VRAM_MB" ]; then
        echo "WARNING: only ${free_vram} MiB VRAM free (<${MIN_FREE_VRAM_MB}); check co-tenants (ISSUES.md)" >&2
    fi
else
    echo "WARNING: nvidia-smi not found; Isaac Sim needs an RTX-class GPU" >&2
fi

# --- venv (same recipe as perception_venv) ----------------------------------
if [ ! -d "$VENV" ]; then
    python3 -m venv --system-site-packages "$VENV"
fi
echo "/opt/ros/jazzy/lib/python3.12/site-packages" \
    > "$VENV/lib/python3.12/site-packages/ros2.pth"

"$VENV/bin/python3" -m pip install -U pip
"$VENV/bin/python3" -m pip install -r "$ROOT/requirements-isaac.txt"

# --- optional shader-cache warmup -------------------------------------------
if [ "${1:-}" = "--warmup" ]; then
    echo "== warmup: first headless SimulationApp boot (shader cache bake, several minutes)"
    OMNI_KIT_ACCEPT_EULA=YES "$VENV/bin/python3" - <<'EOF'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.timeline
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(60):
    app.update()
timeline.stop()
app.close()
print("warmup ok")
EOF
fi

echo "== isaac_venv ready: $VENV"
echo "   smoke test: source /opt/ros/jazzy/setup.bash && isaac_venv/bin/python3 tools/isaac/smoke_test.py"
