# Week3 Day4 — 最小对照实验 A/B（只改 1 个参数）

## 目的
用 W3D2 的 action 自动计时（CSV）做一个最小 A/B 对照实验：
- baseline：不改参数，跑 N=10 次
- A：只改 1 个参数，跑 N=10 次
输出可复现的对照表，并能用 30 秒讲清“改动 → 行为 → 指标”。

---

## 实验设置（固定口径）
- 地图：沿用 Week1/Week2 的同一 Gazebo 场景
- 起点：沿用 Week1 固定起点（起点一致，参考week1_day6_ABCDpoint.png）
- 终点：固定同一个目标点（C点,，参考week1_day6_ABCDpoint.png）
- 每组次数：N=10
- 记录方式：脚本 `scripts/eval_v1_action_logger.py`，每次点击一次 Nav2 Goal 记录一行后退出
- 成功判据：以 Nav2 action 状态 `SUCCEEDED` 记为 `S`（本次两组均为 S）

---

## 参数改动（只改 1 个）
- baseline：保持默认/原参数（请在此处填入你当时的 baseline 数值）
- A：修改 local costmap 的 inflation 半径（更保守/更留边界）

把“证据”也记录下来：
    ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
    ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 1.2
    ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius

> 说明：本实验的关键是“只改 1 个参数”。你本次 A 组的 note= A，baseline note= baseline，满足对照设计。

---

## 数据文件
- baseline：`results/week3_day4_baseline_action.csv`
- A：`results/week3_day4_A_action.csv`

字段解释：
- run：第几次
- goal_id：本次 action 的 UUID
- result：S=SUCCEEDED，T=CANCELED，F=ABORTED/FAILED
- time_sec：本次导航耗时（秒）
- start_ns / end_ns：开始/结束时间戳（纳秒，ROS clock）
- notes：实验分组标记（baseline / A）

---

## 结果汇总（N=10 vs N=10）
本次两组成功率均为 100%（10/10）。

| group | runs | success_rate | avg_time(s) | median_time(s) | min_time(s) | max_time(s) |
|------:|-----:|-------------:|------------:|---------------:|------------:|------------:|
| baseline | 10 | 1.000 | 13.080 | 12.996 | 12.471 | 14.007 |
| A | 10 | 1.000 | 12.736 | 12.611 | 12.470 | 13.505 |

差值（A - baseline）：
- avg_time：-0.344 s
- median_time：-0.385 s

---

## 结论
1) 我做了一个最小 A/B 对照：两组只差 1 个参数（inflation_radius），其它条件（地图/起终点/次数/计时方式）保持一致。  
2) 在本次 N=10 的采样里，两组成功率都为 100%，但 A 组耗时更短（平均 -0.344s，中位数 -0.385s），说明“更保守的膨胀半径”在这个场景下没有拖慢任务，反而可能减少贴墙/微调导致的时间损耗。  
3) 这只是最小实验结论：下一步要在更多 episode 或更难地图上复现，并结合 W3D3 的 recovery 统计，判断耗时变化是否来自更少的 recovery / 更少的局部震荡。

---

## 复现步骤（最简版）
1) baseline 组（N=10）：
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v1_action_logger.py --note baseline --out results/week3_day4_baseline_action.csv
    （去 RViz 点击一次固定目标点，脚本写一行后退出；重复 10 次）

2) A 组：只改 1 个参数，然后同样跑 N=10：
    ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 1.2
    python3 scripts/eval_v1_action_logger.py --note A --out results/week3_day4_A_action.csv
    （去 RViz 点击同一固定目标点；重复 10 次）

3) 收尾：把 inflation_radius 改回 baseline，避免污染后续实验。

---

## 今日交付物清单（对照打勾）
-  results/week3_day4_baseline_action.csv（10 runs）
-  results/week3_day4_A_action.csv（10 runs）
-  docs/week3_day4_ab_test.md（本文件，含对照表与结论）
