# Week4 Day3 — A 组实验记录（Goal Tolerance）

## 1. 目的
在 Week4 的 2×2 对照框架中完成 **A 组实验**（仅修改 1 个参数），评估该参数对导航结果与耗时的影响，并与 baseline 组进行可比性对照。

## 2. 固定实验口径（与 baseline 保持一致）
- 地图：与 Week1/Week3 实验一致（同一仿真地图）
- 起点/终点：沿用 `picture/week1_day6_ABCDpoint.png` 的固定起点与 C 点目标
- 单次评估范围：仅统计“起点 → C 点”这一段导航；返回起点过程不计入
- 每组次数：20 runs
- 评估方式：使用 action 状态计时与结果判定，并结合 BT log 提取恢复行为字段

## 3. A 组参数变更（仅改 1 项）
- 变更对象：`controller_server` 的 `general_goal_checker.xy_goal_tolerance`
- baseline 原值：0.25
- A 组设定值：0.40
- 变更动机：
  - 到达判定半径增大后，机器人更容易满足“到达目标”的终止条件
  - 预期表现：成功率可能上升；在“靠近目标区域徘徊”的情况下，终止更早，耗时可能下降

参数核验建议（运行时确认以避免“改了但未生效”）：
    ros2 param get /controller_server general_goal_checker.xy_goal_tolerance

## 4. 数据采集与执行流程
### 4.1 BT log 采集（终端 C，持续运行）
    source /opt/ros/humble/setup.bash
    rm -f /tmp/week4_day3_bt.log
    ros2 topic echo /behavior_tree_log > /tmp/week4_day3_bt.log

说明：BT log 位于 `/tmp`，用于本地解析恢复行为与区间行号，不作为仓库长期存档文件。

### 4.2 单次记录（终端 D，每个 run 启动一次）
每次 run 执行一次脚本后，在 RViz 中发送一次 “Nav2 Goal（起点 → C 点）”，脚本写入 1 行记录并自动退出。
返回起点由人工操作完成，不计入评估记录。
    python3 scripts/eval_v2_action_recovery.py \
    --btlog /tmp/week4_day3_bt.log \
    --out results/week4_A_runs.csv \
    --note "W4D3 A run_01"

## 4.3 参数回滚（实验结束后恢复 baseline）
为保证后续 B 组与 A+B 组实验可比性，A 组完成后需将参数恢复为 baseline 设定，并进行运行时核验。

- 回滚对象：`controller_server.general_goal_checker.xy_goal_tolerance`
- A 组设定值：0.40
- baseline 设定值：0.25

回滚后建议重启 Nav2（确保运行态参数一致），并使用以下命令核验生效值：

    ros2 param get /controller_server general_goal_checker.xy_goal_tolerance

核验结果应为 baseline 值（0.25）。

## 5. 产出文件
- A 组结果 CSV：
  - `results/week4_A_runs.csv`
- BT log（本地临时）：
  - `/tmp/week4_day3_bt.log`

CSV 列字段与 Week4 baseline 文件保持一致（用于 Day6 总表直接对照）：
- run, goal_id, result, time_sec, notes, start_ns, end_ns, recovery, recovery_types, recovery_hits, bt_start_line, bt_end_line, source, orig_run

## 6. 结果摘要（A 组）
数据来源：`results/week4_A_runs.csv`

- runs: 20
- success: 20
- fail: 0
- cancel: 0
- success_rate: 1.000
- avg_time: 12.126 s
- median_time: 11.811 s
- min_time: 11.661 s
- max_time: 13.980 s
- recovery: 本组记录为 N（未观察到恢复动作命中）

## 7. 与 baseline 的对照解读（初步）
baseline（W4D2）摘要（来自 `results/week4_baseline_runs.csv`）：
- avg_time: 13.139 s
- median_time: 12.996 s
- success_rate: 1.000

A 组相对 baseline 的变化：
- 平均耗时：13.139 → 12.126（-1.013 s，约 -7.7%）
- 中位耗时：12.996 → 11.811（-1.185 s，约 -9.1%）
- 成功率：保持 100%（本口径下 baseline 已达满分，成功率提升空间为 0）

解释要点（面试可用）：
- 当 baseline 成功率已达到上限时，A 组更有价值的指标是“耗时分布”与“是否在目标附近触发恢复/徘徊”。
- `xy_goal_tolerance` 放宽后，更倾向于减少目标附近的无效微调与收敛时间，从而降低整体耗时。

## 8. 备注与边界
- 本实验未做自动复位，采用人工返回起点方式，原因是自动复位会引入 map 漂移与 global/local costmap 不一致风险，从而破坏可比性。
- A 组仅完成单参数验证；是否存在与 B 组（progress checker）叠加效应，将在 A+B 组实验中验证。
