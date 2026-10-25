# D1 Baseline: House Layouts (Estimated Pose, Baseline Profile)

Configuration: `PROFILE=baseline`, `POSE_MODE=estimated`
Command: `scripts/run_batch.sh 100 5 house`

| layout | took_off | pose_valid | coverage % | survivors | false_positives | collision_events | home_error_m | flight_time_s | path_length_m | exit_reason |
|---|---|---|---|---|---|---|---|---|---|---|
| house_00100 | YES | NO | 7.3% | 0/4 | 3 | 647 | 10.29 | 220.0 | 84.45 | TIMEOUT |
| house_00101 | YES | NO | 13.9% | 0/2 | 2 | 275 | 18.20 | 220.0 | 110.23 | TIMEOUT |
| house_00102 | YES | NO | 5.6% | 0/6 | 2 | 838 | 9.21 | 220.0 | 95.61 | TIMEOUT |
| house_00103 | YES | NO | 8.0% | 0/2 | 0 | 805 | 17.01 | 220.0 | 96.97 | TIMEOUT |
| house_00104 | YES | NO | 19.3% | 0/5 | 6 | 174 | 8.01 | 220.0 | 111.34 | TIMEOUT |
| **MEAN** | 5/5 | 0/5 | 10.8% | 0/19 | 2.6 | 547.8 | 12.54 | 220.0 | 99.72 | 0/5 COMPLETE |
| **WORST** | YES | NO | 5.6% | 0/6 | 6 | 838 | 18.20 | 220.0 | 84.45 | TIMEOUT |

