#!/usr/bin/env bash
# Create/refresh an Isaac Sim 6.1 Python environment (pip install, ~30-50 GB).
#
# Mirrors the perception_venv pattern (README "G1 perception venv"):
# system-site-packages venv + ros2.pth so the Isaac ROS 2 bridge links the
# system ROS Jazzy / CycloneDDS stack instead of its bundled libraries.
#
# Usage: tools/install_isaac_venv.sh [--venv <path>] [--warmup]
#   --venv   candidate environment path (default: <repo>/isaac_venv)
#   --warmup  after install, boot SimulationApp headless once to bake the
#             RTX shader cache (first boot takes minutes; subsequent
#             launches then start fast enough for the bringup gates).
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
VENV="$ROOT/isaac_venv"
MIN_DRIVER="595.58.03"
MIN_FREE_VRAM_MB=16000
WARMUP=false

usage() {
    sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --venv)
            [ "$#" -ge 2 ] || { echo "ERROR: --venv requires a path" >&2; exit 2; }
            if [[ "$2" = /* ]]; then
                VENV="$2"
            else
                VENV="$ROOT/$2"
            fi
            shift 2
            ;;
        --warmup)
            WARMUP=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# --- preflight: driver + free VRAM (shared box: warn, don't block) ---------
if command -v nvidia-smi >/dev/null \
        && driver="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"; then
    if [ "$(printf '%s\n' "$MIN_DRIVER" "$driver" | sort -V | head -1)" != "$MIN_DRIVER" ]; then
        echo "WARNING: driver $driver < required $MIN_DRIVER for Isaac Sim 6.1" >&2
    fi
    free_vram="$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)"
    if [ "$free_vram" -lt "$MIN_FREE_VRAM_MB" ]; then
        echo "WARNING: only ${free_vram} MiB VRAM free (<${MIN_FREE_VRAM_MB}); check co-tenants (docs/troubleshooting.md)" >&2
    fi
else
    echo "WARNING: nvidia-smi unavailable; Isaac Sim needs a working RTX-class GPU" >&2
fi

# --- venv (same recipe as perception_venv) ----------------------------------
if [ ! -d "$VENV" ]; then
    python3 -m venv --system-site-packages "$VENV"
fi
# A side-by-side candidate lives at the workspace root, where an unrestricted
# colcon invocation would otherwise crawl Isaac's bundled setup.py examples.
: > "$VENV/COLCON_IGNORE"
echo "/opt/ros/jazzy/lib/python3.12/site-packages" \
    > "$VENV/lib/python3.12/site-packages/ros2.pth"

"$VENV/bin/python3" -m pip install -U pip
"$VENV/bin/python3" -m pip install -r "$ROOT/requirements-isaac.txt"

# --- optional shader-cache warmup -------------------------------------------
if "$WARMUP"; then
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
echo "   smoke test: source /opt/ros/jazzy/setup.bash && $VENV/bin/python3 tools/isaac/smoke_test.py"
