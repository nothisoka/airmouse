# Diagnosis B: Root Cause Analysis of Low Coverage (True Pose)

**Date:** October 25, 2026  
**Context:** Phase B Autonomy Investigation with `POSE_MODE=true` (True Pose Baseline)  
**Evaluated Layouts:** `house_00104`, `house_00102`, `maze_00200`  

---

## 1. Executive Summary & Diagnostic Telemetry

Even with perfect ground-truth pose estimation (`POSE_MODE=true`), procedural building coverage across standard layouts remained well below the 90% target:
- `house_00104`: 10.4% in A9 baseline (ended at 170.7s); achieved 70.9% before timeout when allowed longer flight, but suffered multiple 50–60s stagnation plateaus.
- `house_00102`: 29.8% coverage (Target: 90%), flight time 323.6s, 48 collision events, 27 frontier clusters left unexplored.
- `maze_00200`: 31.8% coverage, flight time 584.9s, 422 collision events, 37 frontier clusters left unexplored.

### Quantitative Summary Table

| Metric | `house_00104` | `house_00102` | `maze_00200` |
|---|---|---|---|
| **Coverage (%)** | 70.9% (10.4% in A9) | 29.8% | 31.8% |
| **Flight Time (s)** | 374.0s (aborted by timeout) | 323.6s | 584.9s |
| **Exit Reason** | TIMEOUT | INCOMPLETE | INCOMPLETE |
| **End Cause** | Wall-clock Timeout | BUDGET_EXHAUSTED (rem=316.2s, req=329.4s) | BUDGET_EXHAUSTED (rem=274.8s, req=296.5s) |
| **Total Replans** | 188 | 152 | 252 |
| **Blacklisted Goals** | 10 | 17 (36 fail events) | 130 (266 fail events) |
| **Collision Events** | 0 | 48 | 422 |
| **Remaining Clusters** | 8 | 27 | 37 |
| **Time in EXPLORE** | ~350 s | 280.0 s | 321.4 s |
| **Time in RETURN_HOME** | 0.0 s | 37.1 s | 257.0 s |
| **Longest Stagnation** | 60 s (at 65.2%) | **200 s (at 27.1%)** | 70 s (at 31.5%) |

---

## 2. Telemetry Evidence Over Time

### Layout: `house_00102` (Severe 200-Second Stall)
```text
[DIAG t= 70.0s] State: EXPLORE     | Cov: 23.1% | Speed: 0.59m/s | Replans:  35 | Blacklisted:  2
[DIAG t= 80.0s] State: EXPLORE     | Cov: 26.8% | Speed: 0.68m/s | Replans:  41 | Blacklisted:  2
[DIAG t= 90.0s] State: EXPLORE     | Cov: 27.1% | Speed: 0.59m/s | Replans:  47 | Blacklisted:  2
... [STUCK IN PLACE: Speed collapsing to 0.00 m/s, Replans climbing from 47 to 150] ...
[DIAG t=200.0s] State: EXPLORE     | Cov: 26.3% | Speed: 0.08m/s | Replans: 104 | Blacklisted:  8
[DIAG t=250.0s] State: EXPLORE     | Cov: 27.1% | Speed: 0.00m/s | Replans: 133 | Blacklisted: 16
[DIAG t=270.0s] State: EXPLORE     | Cov: 28.2% | Speed: 0.44m/s | Replans: 145 | Blacklisted: 16
[DIAG t=280.0s] State: EXPLORE     | Cov: 27.6% | Speed: 0.47m/s | Replans: 150 | Blacklisted: 17
[DIAG t=290.0s] State: RETURN_HOME | Cov: 31.1% | Speed: 0.36m/s | Replans: 152 | Blacklisted: 17
```
**Finding:** Between $t=80\text{s}$ and $t=280\text{s}$ (200 continuous seconds), room coverage stayed locked between 26.8% and 28.2%. The drone executed over 100 replans while remaining trapped near a doorway opening, accumulating 17 blacklisted goals before its flight budget triggered return-to-home.

### Layout: `maze_00200` (Massive Blacklisting & Corridor Collisions)
```text
[DIAG t= 40.0s] State: EXPLORE     | Cov: 13.3% | Speed: 0.39m/s | Replans:  20 | Blacklisted:  0
[DIAG t= 70.0s] State: EXPLORE     | Cov: 15.7% | Speed: 0.58m/s | Replans:  50 | Blacklisted: 21
[DIAG t=100.0s] State: EXPLORE     | Cov: 16.1% | Speed: 0.45m/s | Replans:  81 | Blacklisted: 47
[DIAG t=160.0s] State: EXPLORE     | Cov: 19.9% | Speed: 0.46m/s | Replans: 119 | Blacklisted: 55
[DIAG t=220.0s] State: EXPLORE     | Cov: 30.1% | Speed: 0.00m/s | Replans: 166 | Blacklisted: 77
[DIAG t=310.0s] State: EXPLORE     | Cov: 31.8% | Speed: 0.55m/s | Replans: 239 | Blacklisted: 122
[DIAG t=340.0s] State: RETURN_HOME | Cov: 32.2% | Speed: 0.59m/s | Replans: 252 | Blacklisted: 130
```
**Finding:** In maze passages, 130 candidate frontier goals were permanently blacklisted across 266 path planning failures. The drone logged 422 collision events bouncing off walls, leaving 37 frontier clusters completely unvisited.

---

## 3. Ranked Top 3 Causes of Low Coverage

### Cause 1 (Rank 1): Doorway & Passage Inaccessibility from Excessive Obstacle Inflation vs Opening Width
- **Mechanism:** In `config/mapping.yaml`, obstacle inflation is configured as `inflation_radius_m: 0.40` (`robot_radius_m: 0.25` + `safety_margin_m: 0.15`). The grid map resolution is $0.25\text{ m}$. Standard doorways and maze corridors are 1.0 m wide (1 cell). When walls on both sides of a 1.0 m doorway or corridor are each inflated by $0.40\text{ m}$, the inflated obstacles encroach by $0.40 + 0.40 = 0.80\text{ m}$, leaving only $0.20\text{ m}$ of nominal free space. Since $0.20\text{ m} < 0.25\text{ m}$ (the cell resolution), the doorway becomes completely severed (lethal cost $\ge 200$) in the inflated costmap. A* planner declares the goal unreachable and `FrontierDetector.report_failure` blacklists the doorway target after 3 attempts.
- **Evidence:**
  - `maze_00200`: 130 blacklisted goals, 266 total planning failures. 37 frontier clusters were abandoned because A* could not find a path through 1-meter maze corridors.
  - `house_00102`: 17 blacklisted goals, 36 planning failures. 27 frontier clusters remained unreached behind room doorways.
  - `house_00104`: 10 blacklisted goals by $t=110\text{s}$.

### Cause 2 (Rank 2): Drone Stall / Goal Oscillation Near Openings Without a Watchdog
- **Mechanism:** When candidate frontiers lie immediately behind a constricted passage or doorway, the frontier selector picks a centroid near the doorway opening. The drone approaches the opening, but live ultrasonic readings trigger reactive avoidance (`slowdown_dist_m: 0.65` attenuating velocity and `lat_error` centering overriding lateral commands). The drone decelerates to near zero velocity ($0.00 - 0.08\text{ m/s}$) or bounces between two adjacent doorway cells. Because there is currently no stall or oscillation watchdog, the autonomy stack remains trapped in this state for minutes until flight budget runs out.
- **Evidence:**
  - `house_00102`: Coverage stagnated completely between 26.8% and 28.2% for **200 consecutive seconds** (t=80s to t=280s). Over 100 futile replans were executed in place while speed was repeatedly measured at $0.00\text{ m/s}$ (t=250s).
  - `house_00104`: 3 distinct multi-ten-second plateaus (t=60–120s at 24%, t=200–250s at 46%, t=280–340s at 65%).
  - `maze_00200`: Zero-velocity stalls at corner junctions ($t=220\text{s}$, speed $0.00\text{ m/s}$).

### Cause 3 (Rank 3): Premature Exploration Termination / False "Frontiers Exhausted" Heuristic
- **Mechanism:** In `core/mission/state_machine.py`, the exploration loop contains an aggressive fallback counter:
  ```python
  if not self.current_path:
      if self.no_frontier_count < 10:
          return VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.6)
      else:
          # Genuinely exhausted frontiers: exploration completed!
          self.state = MissionState.RETURN_HOME
  ```
  When the drone encounters a temporary blockage or turns away from open space, `_plan_next_frontier` fails to return a path. Since replanning occurs every 0.5–2.0 seconds, `no_frontier_count` reaches 10 within just 5–15 seconds of localized failure. The state machine concludes that all frontiers are exhausted, transitions to `RETURN_HOME`, lands, and ends the mission prematurely—despite dozens of unmapped rooms and clusters remaining.
- **Evidence:**
  - `house_00104` in A9 baseline: Ended exploration after only $170.7\text{s}$ at $10.4\%$ coverage and 0 collisions, falsely claiming exploration completion while $89.6\%$ of the house was unvisited.
  - `house_00102`: Returned home with 27 frontier clusters remaining.
  - `maze_00200`: Returned home with 37 frontier clusters remaining.

---

## 4. Prioritization for Phase B Fixes

1. **Fix Cause #1 (Task B4 / B7):** Adjust obstacle inflation and doorway traversal clearance so 1-cell (1.0m) doorways are passable by the planner while maintaining minimum physical drone hull clearance.
2. **Fix Cause #2 (Task B5 / B8):** Implement a stall and oscillation watchdog (detecting $<0.5\text{m}$ movement for 15s or 3 goal alternations) to blacklist unprogressable local goals and force dynamic replanning to alternate frontiers.
3. **Fix Cause #3 (Task B6 / B9):** Replace the brittle 10-count failure heuristic with cluster-based exploration-finished detection that only triggers return-home when no reachable frontier clusters exceeding the minimum size remain.
