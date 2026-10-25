# D1 Baseline: Maze Layouts (Estimated Pose, Baseline Profile)

Configuration: `PROFILE=baseline`, `POSE_MODE=estimated`
Command: `scripts/run_batch.sh 200 5 maze`

| layout | took_off | pose_valid | coverage % | survivors | false_positives | collision_events | home_error_m | flight_time_s | path_length_m | exit_reason |
|---|---|---|---|---|---|---|---|---|---|---|
| maze_00200 | YES | NO | 16.8% | 0/6 | 0 | 507 | 3.19 | 220.0 | 94.48 | TIMEOUT |
| maze_00201 | YES | NO | 2.8% | 0/1 | 0 | 1037 | 2.27 | 220.0 | 87.78 | TIMEOUT |
| maze_00202 | YES | NO | 20.2% | 0/6 | 0 | 378 | 1.73 | 220.0 | 105.59 | TIMEOUT |
| maze_00203 | YES | NO | 13.1% | 0/5 | 0 | 500 | 1.40 | 220.0 | 98.94 | TIMEOUT |
| maze_00204 | YES | NO | 2.1% | 0/6 | 5 | 825 | 6.26 | 220.0 | 79.99 | TIMEOUT |
| **MEAN** | 5/5 | 0/5 | 11.0% | 0/24 | 1.0 | 649.4 | 2.97 | 220.0 | 93.36 | 0/5 COMPLETE |
| **WORST** | YES | NO | 2.1% | 0/6 | 5 | 1037 | 6.26 | 220.0 | 79.99 | TIMEOUT |
