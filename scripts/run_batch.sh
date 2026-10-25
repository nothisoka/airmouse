#!/usr/bin/env bash
set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ -n "${VIRTUAL_ENV:-}" ]; then
    PYTHON="$VIRTUAL_ENV/bin/python3"
elif [ -f "$REPO_ROOT/.venv/bin/python3" ]; then
    PYTHON="$REPO_ROOT/.venv/bin/python3"
else
    PYTHON="python3"
fi

# Check for --pose-mode flag in arguments
POSE_MODE="${POSE_MODE:-estimated}"
ARGS=()
for arg in "$@"; do
    case "$arg" in
        --pose-mode=*)
            POSE_MODE="${arg#*=}"
            ;;
        *)
            ARGS+=("$arg")
            ;;
    esac
done

START_SEED="${ARGS[0]:-100}"
COUNT="${ARGS[1]:-3}"
MODE="${ARGS[2]:-house}"

mkdir -p generated_maps

# Generate layouts with fixed seeds using generate_maps.py
"$PYTHON" generate_maps.py --mode "$MODE" --seed "$START_SEED" --count "$COUNT" --out generated_maps > /dev/null 2>&1

export POSE_MODE
export ENABLE_YOLO="${ENABLE_YOLO:-true}"

# Run headless sequentially on each layout and print markdown table with MEAN / WORST rows
"$PYTHON" - "$START_SEED" "$COUNT" "$MODE" "$POSE_MODE" << 'EOF'
import sys, subprocess, json
from pathlib import Path

start_seed = int(sys.argv[1])
count = int(sys.argv[2])
mode = sys.argv[3]
pose_mode = sys.argv[4]

results = []

for i in range(count):
    seed = start_seed + i
    map_name = f"{mode}_{seed:05d}"
    map_json = Path("generated_maps") / f"{map_name}.json"
    label = f"batch_{map_name}"
    
    # Run run_headless.sh in foreground; failure must not abort the batch
    cmd = ["scripts/run_headless.sh", str(map_json), label]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    # Read metrics JSON
    metrics_path = Path("generated_maps") / f"metrics_{label}.json"
    data = {}
    if metrics_path.exists():
        try:
            with open(metrics_path, "r") as f:
                data = json.load(f)
        except Exception:
            pass
            
    took_off = data.get("took_off", True if res.returncode == 0 else False)
    mean_drift = data.get("mean_drift_m", 0.0)
    pose_valid = data.get("pose_valid", mean_drift < 1.0)
    cov = data.get("coverage_pct", data.get("coverage_percentage", 0.0))
    s_found = data.get("survivors_found", 0)
    s_total = data.get("survivors_total", data.get("total_survivors", 0))
    fp = data.get("false_positives", 0)
    ce = data.get("collision_events", data.get("collisions", 0))
    he = data.get("home_error_m", data.get("return_home_error_m", 0.0))
    ft = data.get("flight_time_s", data.get("total_time_s", 0.0))
    pl = data.get("path_length_m", data.get("distance_traveled_m", 0.0))
    er = data.get("exit_reason", "TIMEOUT" if res.returncode != 0 else "COMPLETE")

    results.append({
        "layout": map_name,
        "took_off": took_off,
        "pose_valid": pose_valid,
        "coverage_pct": cov,
        "survivors_found": s_found,
        "survivors_total": s_total,
        "false_positives": fp,
        "collision_events": ce,
        "home_error_m": he,
        "flight_time_s": ft,
        "path_length_m": pl,
        "exit_reason": er,
    })

# Format markdown table
header = "| layout | took_off | pose_valid | coverage % | survivors | false_positives | collision_events | home_error_m | flight_time_s | path_length_m | exit_reason |"
sep    = "|---|---|---|---|---|---|---|---|---|---|---|"
lines = [header, sep]

for r in results:
    to_str = "YES" if r["took_off"] else "NO"
    pv_str = "YES" if r["pose_valid"] else "NO"
    cov_str = f"{r['coverage_pct']:.1f}%"
    surv_str = f"{r['survivors_found']}/{r['survivors_total']}"
    row = f"| {r['layout']} | {to_str} | {pv_str} | {cov_str} | {surv_str} | {r['false_positives']} | {r['collision_events']} | {r['home_error_m']:.2f} | {r['flight_time_s']:.1f} | {r['path_length_m']:.2f} | {r['exit_reason']} |"
    lines.append(row)

# Compute MEAN
n = len(results)
if n > 0:
    mean_to = f"{sum(1 for r in results if r['took_off'])}/{n}"
    mean_pv = f"{sum(1 for r in results if r['pose_valid'])}/{n}"
    mean_cov = f"{sum(r['coverage_pct'] for r in results) / n:.1f}%"
    tot_sf = sum(r['survivors_found'] for r in results)
    tot_st = sum(r['survivors_total'] for r in results)
    surv_mean = f"{tot_sf}/{tot_st}"
    mean_fp = f"{sum(r['false_positives'] for r in results) / n:.1f}"
    mean_ce = f"{sum(r['collision_events'] for r in results) / n:.1f}"
    mean_he = f"{sum(r['home_error_m'] for r in results) / n:.2f}"
    mean_ft = f"{sum(r['flight_time_s'] for r in results) / n:.1f}"
    mean_pl = f"{sum(r['path_length_m'] for r in results) / n:.2f}"
    complete_count = sum(1 for r in results if r['exit_reason'] == 'COMPLETE')
    mean_er = f"{complete_count}/{n} COMPLETE"

    mean_row = f"| **MEAN** | {mean_to} | {mean_pv} | {mean_cov} | {surv_mean} | {mean_fp} | {mean_ce} | {mean_he} | {mean_ft} | {mean_pl} | {mean_er} |"
    lines.append(mean_row)

    # Compute WORST
    worst_to = "NO" if any(not r['took_off'] for r in results) else "YES"
    worst_pv = "NO" if any(not r['pose_valid'] for r in results) else "YES"
    worst_cov = f"{min(r['coverage_pct'] for r in results):.1f}%"
    worst_surv = f"{min(r['survivors_found'] for r in results)}/{max(r['survivors_total'] for r in results)}"
    worst_fp = f"{max(r['false_positives'] for r in results)}"
    worst_ce = f"{max(r['collision_events'] for r in results)}"
    worst_he = f"{max(r['home_error_m'] for r in results):.2f}"
    worst_ft = f"{max(r['flight_time_s'] for r in results):.1f}"
    worst_pl = f"{min(r['path_length_m'] for r in results):.2f}"
    reasons = [r['exit_reason'] for r in results]
    worst_er = "CRASH" if "CRASH" in reasons else ("FAILSAFE" if "FAILSAFE" in reasons else ("TIMEOUT" if "TIMEOUT" in reasons else ("INCOMPLETE" if "INCOMPLETE" in reasons else "COMPLETE")))

    worst_row = f"| **WORST** | {worst_to} | {worst_pv} | {worst_cov} | {worst_surv} | {worst_fp} | {worst_ce} | {worst_he} | {worst_ft} | {worst_pl} | {worst_er} |"
    lines.append(worst_row)

print("\n".join(lines))
EOF
