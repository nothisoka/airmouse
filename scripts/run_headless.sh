#!/usr/bin/env bash
set -uo pipefail

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <map.json> <label> [extra run.py args...]" >&2
    exit 2
fi

MAP_PATH="$1"
LABEL="$2"
shift 2

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ -n "${VIRTUAL_ENV:-}" ]; then
    PYTHON="$VIRTUAL_ENV/bin/python3"
elif [ -f "$REPO_ROOT/.venv/bin/python3" ]; then
    PYTHON="$REPO_ROOT/.venv/bin/python3"
else
    PYTHON="python3"
fi

mkdir -p generated_maps

METRICS_JSON="generated_maps/metrics_${LABEL}.json"
LOG_FILE="generated_maps/run_${LABEL}.log"

# Support POSE_MODE=true|estimated
EXTRA_ARGS=()
if [ "${POSE_MODE:-}" = "true" ]; then
    EXTRA_ARGS+=(--debug-true-pose)
fi
if [ "${ENABLE_YOLO:-}" != "true" ]; then
    EXTRA_ARGS+=(--no-yolo)
fi

# Run the simulation headless under timeout 300
timeout 300 "$PYTHON" run.py \
    --map "$MAP_PATH" \
    --headless \
    --output-json "$METRICS_JSON" \
    "${EXTRA_ARGS[@]}" \
    "$@" > "$LOG_FILE" 2>&1
SIM_EXIT=$?

# Mirror log to root and generated_maps/<label>.log for convenience
cp -f "$LOG_FILE" "run_${LABEL}.log" 2>/dev/null || true
cp -f "$LOG_FILE" "generated_maps/${LABEL}.log" 2>/dev/null || true

# Extract protocol metrics and print ONE summary line
"$PYTHON" - "$METRICS_JSON" "$MAP_PATH" "$SIM_EXIT" << 'EOF'
import sys, json
from pathlib import Path

metrics_file = sys.argv[1]
map_path = sys.argv[2]
sim_exit = int(sys.argv[3])

data = {}
if Path(metrics_file).exists():
    try:
        with open(metrics_file, "r") as f:
            data = json.load(f)
    except Exception:
        pass

# layout
layout = Path(map_path).stem

# took_off
if "took_off" in data:
    took_off = "YES" if data["took_off"] else "NO"
else:
    took_off = "n/a"

# pose_valid (mean drift < 1.0 m)
if "pose_valid" in data:
    pose_valid = "YES" if data["pose_valid"] else "NO"
elif "mean_drift_m" in data:
    pose_valid = "YES" if data["mean_drift_m"] < 1.0 else "NO"
elif "pose_estimate_valid" in data:
    pose_valid = "YES" if data["pose_estimate_valid"] else "NO"
else:
    pose_valid = "n/a"

# coverage %
if "coverage_pct" in data:
    cov = data["coverage_pct"]
    cov_str = f"{cov:.1f}%" if isinstance(cov, (int, float)) else str(cov)
elif "coverage_percentage" in data:
    cov = data["coverage_percentage"]
    cov_str = f"{cov:.1f}%" if isinstance(cov, (int, float)) else str(cov)
else:
    cov_str = "n/a"

# survivors found/total
s_found = data.get("survivors_found")
s_total = data.get("survivors_total", data.get("total_survivors"))
if s_found is not None and s_total is not None:
    survivors = f"{s_found}/{s_total}"
elif s_found is not None:
    survivors = str(s_found)
else:
    survivors = "n/a"

# false positives
fp = data.get("false_positives")
fp_str = str(fp) if fp is not None else "n/a"

# collision_events
if "collision_events" in data:
    ce_str = str(data["collision_events"])
elif "collisions" in data:
    ce_str = str(data["collisions"])
else:
    ce_str = "n/a"

# home_error_m
he = data.get("home_error_m", data.get("return_home_error_m"))
he_str = f"{he:.2f}" if isinstance(he, (int, float)) else "n/a"

# flight_time_s
ft = data.get("flight_time_s", data.get("total_time_s"))
ft_str = f"{ft:.1f}" if isinstance(ft, (int, float)) else "n/a"

# path_length_m
pl = data.get("path_length_m", data.get("distance_traveled_m"))
pl_str = f"{pl:.2f}" if isinstance(pl, (int, float)) else "n/a"

# exit_reason
er = data.get("exit_reason")
if er is None:
    er = "TIMEOUT" if sim_exit == 124 else ("ERROR" if sim_exit != 0 else "COMPLETE")

print(f"| {layout} | {took_off} | {pose_valid} | {cov_str} | {survivors} | {fp_str} | {ce_str} | {he_str} | {ft_str} | {pl_str} | {er} |")
EOF

exit "$SIM_EXIT"
