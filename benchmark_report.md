# NIDAR AirMouse: 20-Layout Acceptance Benchmark Report

| # | Layout | Mode | Flight Time (s) | Path (m) | Coverage (%) | Survivors Found | False Pos | RTH Error (m) | Mean Drift (m) | Collisions |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | `house_00100` | house | 41.1 | 18.72 | 11.3% | 0/4 | 0 | 0.60 | 1.45 | 0 |
| 2 | `house_00101` | house | 54.7 | 25.02 | 18.5% | 0/2 | 0 | 2.75 | 3.87 | 0 |
| 3 | `house_00102` | house | 32.4 | 13.93 | 11.8% | 0/6 | 2 | 1.60 | 1.74 | 0 |
| 4 | `house_00103` | house | 60.0 | 28.97 | 15.7% | 0/2 | 0 | 2.33 | 1.89 | 0 |
| 5 | `house_00104` | house | 32.5 | 13.91 | 8.3% | 0/5 | 0 | 1.56 | 1.11 | 0 |
| 6 | `house_00105` | house | 60.0 | 30.82 | 14.5% | 0/2 | 0 | 13.75 | 5.30 | 8 |
| 7 | `house_00106` | house | 60.0 | 29.52 | 15.1% | 0/1 | 0 | 2.30 | 2.05 | 0 |
| 8 | `house_00107` | house | 36.9 | 15.71 | 9.4% | 0/4 | 0 | 2.62 | 1.72 | 0 |
| 9 | `house_00108` | house | 60.0 | 28.38 | 11.5% | 1/4 | 3 | 0.57 | 1.62 | 86 |
| 10 | `house_00109` | house | 46.4 | 20.56 | 12.8% | 0/3 | 0 | 2.78 | 2.64 | 0 |
| 11 | `maze_00200` | maze | 60.0 | 18.89 | 14.0% | 0/6 | 0 | 2.32 | 1.59 | 334 |
| 12 | `maze_00201` | maze | 50.7 | 19.31 | 9.8% | 0/1 | 0 | 11.99 | 4.32 | 391 |
| 13 | `maze_00202` | maze | 22.9 | 5.03 | 7.0% | 0/6 | 0 | 0.88 | 0.69 | 0 |
| 14 | `maze_00203` | maze | 28.2 | 9.23 | 8.7% | 0/5 | 0 | 2.99 | 1.56 | 48 |
| 15 | `maze_00204` | maze | 26.8 | 7.58 | 7.7% | 1/6 | 0 | 3.09 | 1.37 | 0 |
| 16 | `maze_00205` | maze | 32.0 | 10.94 | 8.5% | 0/4 | 0 | 0.68 | 1.40 | 24 |
| 17 | `maze_00206` | maze | 32.5 | 10.04 | 8.1% | 0/4 | 0 | 1.76 | 0.95 | 0 |
| 18 | `maze_00207` | maze | 23.0 | 6.11 | 5.5% | 0/6 | 0 | 2.93 | 1.32 | 0 |
| 19 | `maze_00208` | maze | 37.6 | 8.12 | 8.7% | 1/5 | 0 | 2.29 | 1.47 | 11 |
| 20 | `maze_00209` | maze | 25.7 | 5.38 | 6.2% | 0/6 | 0 | 0.58 | 0.66 | 0 |

## Executive Benchmark Summary
- **Total Evaluated Layouts:** 20
- **Average Room Coverage:** 10.7%
- **Total Survivors Located:** 3 / 82 (3.7%)
- **Average Return-to-Home Error:** 3.02 m (Target: < 1.0 m)
- **Average Localization Drift:** 1.93 m
- **Total Obstacle Collisions:** 902 (Zero-collision guarantee: FAILED)