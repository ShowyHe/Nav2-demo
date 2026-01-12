# Week 1 Day 7 — Recap (Nav2 TB3 Demo + Baseline)

## 1) 30s Interview Pitch (spoken)

I ran Nav2 successfully on TurtleBot3 in Gazebo and fixed a reproducible evaluation criterion:  
I extract the goal (x, y) from the `bt_navigator` log and the final pose from `/amcl_pose`, then compute FinalDist in the `map` frame. **FinalDist ≤ 0.25 m is counted as Success**.

Under the baseline setup (same start pose, same map, and four fixed goal points A/B/C/D), I ran 12 trials:  
**D: 3/3 success, A: 0/3, C: 0/3, B: 1/3**, overall **4/12 = 33.3%**.

This indicates the robot usually approaches the goal area, but there are cases where Nav2 reports **“Goal succeeded”** while my fixed FinalDist criterion still fails (often stopping around 0.3–0.4 m away).  
Next, I will run a **2×2 controlled experiment** around **costmap inflation** and **goal checker / controller goal-reaching behavior** (baseline / A / B / A+B), targeting **≥85% success** under the same evaluation metric.

## 2) Nav2 Pipeline (interview-friendly explanation)

### TF (Transforms)
- **Input:** `/tf`, `/tf_static` (must have a stable chain `map → odom → base_link`)
- **Output:** A consistent coordinate system so sensors, robot pose, and map align
- **If broken:** RViz errors, costmaps fail to update correctly, planner timeouts, AMCL cannot publish transforms

### Costmap (Global/Local)
- **Input:** map (static layer), sensor obstacles (obstacle layer), TF (for frame transforms)
- **Output:** `global_costmap` and `local_costmap` grids with inflated safety margins
- **If broken:** too conservative (stops far from goal / avoids corridors), too aggressive (clips obstacles), “ghost obstacles” causing oscillation

### Planner (Global planning)
- **Input:** `global_costmap`, start pose, goal pose
- **Output:** a global path (plan) to the goal
- **If broken:** “No valid plan”, frequent replans, low planning rate / timeouts

### Controller (Local control)
- **Input:** `local_costmap`, current robot pose, global plan
- **Output:** velocity commands `/cmd_vel`
- **If broken:** spinning in place, jittering, stopping early near goal, failing to follow the global plan

### BT Navigator (Behavior Tree + Recovery)
- **Input:** goal + feedback from planner/controller + recovery behaviors (spin/backup/etc.)
- **Output:** orchestrated navigation workflow: plan → control → recover → continue
- **Note:** Nav2 “Goal succeeded” is based on internal conditions; for experiments I use the fixed FinalDist metric to keep comparisons consistent.

## 3) Evidence (Week1)
- Day5 demo video: `videos/week1_day5_demo.mp4`
- Day6 baseline data: `results/day6_baseline.csv`
- Goal points screenshot: `picture/day6_ABCDpoint.png`
# Week 1 Day 7 — Recap (Nav2 TB3 Demo + Baseline)

## 1) 30s Interview Pitch (spoken)

I ran Nav2 successfully on TurtleBot3 in Gazebo and fixed a reproducible evaluation criterion:  
I extract the goal (x, y) from the `bt_navigator` log and the final pose from `/amcl_pose`, then compute FinalDist in the `map` frame. **FinalDist ≤ 0.25 m is counted as Success**.

Under the baseline setup (same start pose, same map, and four fixed goal points A/B/C/D), I ran 12 trials:  
**D: 3/3 success, A: 0/3, C: 0/3, B: 1/3**, overall **4/12 = 33.3%**.

This indicates the robot usually approaches the goal area, but there are cases where Nav2 reports **“Goal succeeded”** while my fixed FinalDist criterion still fails (often stopping around 0.3–0.4 m away).  
Next, I will run a **2×2 controlled experiment** around **costmap inflation** and **goal checker / controller goal-reaching behavior** (baseline / A / B / A+B), targeting **≥85% success** under the same evaluation metric.

## 2) Nav2 Pipeline (interview-friendly explanation)

### TF (Transforms)
- **Input:** `/tf`, `/tf_static` (must have a stable chain `map → odom → base_link`)
- **Output:** A consistent coordinate system so sensors, robot pose, and map align
- **If broken:** RViz errors, costmaps fail to update correctly, planner timeouts, AMCL cannot publish transforms

### Costmap (Global/Local)
- **Input:** map (static layer), sensor obstacles (obstacle layer), TF (for frame transforms)
- **Output:** `global_costmap` and `local_costmap` grids with inflated safety margins
- **If broken:** too conservative (stops far from goal / avoids corridors), too aggressive (clips obstacles), “ghost obstacles” causing oscillation

### Planner (Global planning)
- **Input:** `global_costmap`, start pose, goal pose
- **Output:** a global path (plan) to the goal
- **If broken:** “No valid plan”, frequent replans, low planning rate / timeouts

### Controller (Local control)
- **Input:** `local_costmap`, current robot pose, global plan
- **Output:** velocity commands `/cmd_vel`
- **If broken:** spinning in place, jittering, stopping early near goal, failing to follow the global plan

### BT Navigator (Behavior Tree + Recovery)
- **Input:** goal + feedback from planner/controller + recovery behaviors (spin/backup/etc.)
- **Output:** orchestrated navigation workflow: plan → control → recover → continue
- **Note:** Nav2 “Goal succeeded” is based on internal conditions; for experiments I use the fixed FinalDist metric to keep comparisons consistent.

## 3) Evidence (Week1)
- Day5 demo video: `videos/week1_day5_demo.mp4`
- Day6 baseline data: `results/day6_baseline.csv`
- Goal points screenshot: `picture/day6_ABCDpoint.png`
