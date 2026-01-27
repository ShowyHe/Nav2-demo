# Week4 Day2 — Baseline 评估结果（20 Runs）

## 1. 目的
本日工作用于建立 Week4 的 **baseline 基准数据**，后续 A / B / A+B 组的改参实验均以此为对照，确保对比结论具备可追溯性与可复现性。

## 2. 实验设置（固定口径）
- 地图与场景：沿用 Week1 Day6 记录的地图与标注点位（见 `picture/week1_day6_ABCDpoint.png`）。
- 起点：固定为 Week1 Day6 设定的起点。
- 终点：固定为 C 点。
- 单次评估方式：每次启动记录脚本后，在 RViz 下发一次 Nav2 Goal（起点→C），脚本捕获 action 终态后写入一条记录并退出；回到起点的过程不计入本次记录。
- 数据文件：`results/week4_baseline_runs.csv`

## 3. 数据来源说明（透明声明）
- Runs 1–10：沿用 Week3 的同口径数据（起点→C 点），用于保持历史对照一致性。
- Runs 11–20：Week4 Day2 重新补跑获取，用于将 baseline 样本量扩展至 20 次。

> 备注：由于 Gazebo 复位可能引入地图漂移与 local/global costmap 偏差，本项目评估采用“单次记录 + 人工回到起点再发目标”的方式，以降低复位导致的系统性误差。

## 4. Baseline 汇总结果（来自 `scripts/summarize_action_csv.py`）
- file: `results/week4_baseline_runs.csv`
- runs: 20
- success: 20
- fail: 0
- cancel: 0
- success_rate: 1.000
- avg_time: 13.139 s
- median_time: 12.996 s
- min_time: 12.471 s
- max_time: 14.007 s

## 5. 初步观察
- 本组 20 次均达到 `SUCCEEDED`，在当前固定起终点与参数配置下，系统表现稳定。
- 耗时分布集中在 12.5–14.0 s 区间，后续 Week4 A/B/A+B 组主要关注：在不引入额外不稳定性的前提下，是否能降低耗时或减少潜在恢复/卡滞迹象（若记录项包含 recovery 字段则同步比较）。
