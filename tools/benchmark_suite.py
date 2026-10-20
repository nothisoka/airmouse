#!/usr/bin/env python3
"""Automated Benchmark Suite for NIDAR AirMouse.

Executes autonomous search missions across >= 20 random procedural layouts,
evaluates performance metrics, and generates a formatted Markdown report.
"""
from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path
from typing import List
import numpy as np

try:
    from airmouse.tools import generate_maps
except ImportError:
    import generate_maps
from airmouse.run import run_pygame_sim


def ensure_benchmark_maps(maps_dir: Path, num_houses: int = 10, num_mazes: int = 10) -> List[Path]:
    """Generate reproducible benchmark layouts at runtime into maps_dir using fixed seeds."""
    maps_dir.mkdir(parents=True, exist_ok=True)
    generated = []

    # 1. Generate houses with fixed seeds 101..110
    for i in range(num_houses):
        seed = 101 + i
        name = f"house_{seed:05d}"
        json_path = maps_dir / f"{name}.json"
        if not json_path.exists():
            rng = random.Random(seed)
            grid = generate_maps.gen_house(28, 20, rng)
            start = generate_maps.pick_start(grid, (2, 2))
            generate_maps.add_obstacles(grid, rng, 5, avoid=start)
            people = generate_maps.place_people(grid, start, min(6, 1 + (seed % 6)), rng)
            generate_maps.export_map(grid, start, people, seed, "house", 1.0, maps_dir, name)
        generated.append(json_path)

    # 2. Generate mazes with fixed seeds 201..210
    for i in range(num_mazes):
        seed = 201 + i
        name = f"maze_{seed:05d}"
        json_path = maps_dir / f"{name}.json"
        if not json_path.exists():
            rng = random.Random(seed)
            grid = generate_maps.gen_maze(28, 20, rng)
            start = generate_maps.pick_start(grid, (1, 1))
            generate_maps.add_obstacles(grid, rng, 3, avoid=start)
            people = generate_maps.place_people(grid, start, min(6, 1 + (seed % 6)), rng)
            generate_maps.export_map(grid, start, people, seed, "maze", 1.0, maps_dir, name)
        generated.append(json_path)

    return generated


def run_benchmarks(maps_dir: str = "generated_maps", max_time_per_map: float = 60.0):
    p = Path(maps_dir)
    map_files = ensure_benchmark_maps(p, num_houses=10, num_mazes=10)

    print(f"\n==================================================================")
    print(f"  NIDAR AIRMOUSE: Acceptance Benchmark Suite (20 Random Layouts)")
    print(f"  Evaluating Coverage, Sensor Drift, Survivors Found, RTH Precision")
    print(f"==================================================================\n")

    results = []

    for idx, map_file in enumerate(map_files[:20], 1):
        print(f"\n[{idx}/20] Running benchmark on {map_file.name}...")
        t0 = time.time()
        metrics = run_pygame_sim(
            map_path=str(map_file),
            headless=True,
            max_flight_time_s=max_time_per_map,
            enable_yolo=True,
            seed=1000 + idx,
        )
        elapsed_wall = time.time() - t0

        results.append({
            "map": map_file.stem,
            "mode": "house" if "house" in map_file.name else "maze",
            "time_s": metrics.total_time_s,
            "distance_m": metrics.distance_traveled_m,
            "coverage_pct": metrics.coverage_percentage,
            "survivors_found": metrics.survivors_found,
            "survivors_total": metrics.total_survivors,
            "false_pos": metrics.false_positives,
            "rth_error_m": metrics.return_home_error_m,
            "mean_drift_m": metrics.mean_drift_m,
            "max_drift_m": metrics.max_drift_m,
            "collisions": metrics.collisions,
        })

    # Generate Markdown Table Report
    report_lines = [
        "# NIDAR AirMouse: 20-Layout Acceptance Benchmark Report\n",
        "| # | Layout | Mode | Flight Time (s) | Path (m) | Coverage (%) | Survivors Found | False Pos | RTH Error (m) | Mean Drift (m) | Collisions |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]

    total_cov = 0.0
    total_found = 0
    total_surv = 0
    total_rth_err = 0.0
    total_mean_drift = 0.0
    total_collisions = 0

    for i, r in enumerate(results, 1):
        report_lines.append(
            f"| {i} | `{r['map']}` | {r['mode']} | {r['time_s']:.1f} | {r['distance_m']:.2f} | {r['coverage_pct']:.1f}% | "
            f"{r['survivors_found']}/{r['survivors_total']} | {r['false_pos']} | {r['rth_error_m']:.2f} | {r['mean_drift_m']:.2f} | {r['collisions']} |"
        )
        total_cov += r['coverage_pct']
        total_found += r['survivors_found']
        total_surv += r['survivors_total']
        total_rth_err += r['rth_error_m']
        total_mean_drift += r['mean_drift_m']
        total_collisions += r['collisions']

    n = len(results)
    avg_cov = total_cov / n
    avg_rth = total_rth_err / n
    avg_drift = total_mean_drift / n

    report_lines.append("")
    report_lines.append("## Executive Benchmark Summary")
    report_lines.append(f"- **Total Evaluated Layouts:** {n}")
    report_lines.append(f"- **Average Room Coverage:** {avg_cov:.1f}%")
    report_lines.append(f"- **Total Survivors Located:** {total_found} / {total_surv} ({total_found/total_surv*100:.1f}%)")
    report_lines.append(f"- **Average Return-to-Home Error:** {avg_rth:.2f} m (Target: < 1.0 m)")
    report_lines.append(f"- **Average Localization Drift:** {avg_drift:.2f} m")
    report_lines.append(f"- **Total Obstacle Collisions:** {total_collisions} (Zero-collision guarantee: {'PASSED' if total_collisions==0 else 'FAILED'})")

    report_content = "\n".join(report_lines)
    Path("benchmark_report.md").write_text(report_content)
    print("\nBenchmark completed! Report written to benchmark_report.md")
    print(report_content)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps-dir", default="generated_maps")
    parser.add_argument("--time-per-map", type=float, default=30.0)
    args = parser.parse_args()
    run_benchmarks(args.maps_dir, args.time_per_map)

