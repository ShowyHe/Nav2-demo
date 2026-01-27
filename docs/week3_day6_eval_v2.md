# Week3 Day6 — Eval v2（Action + Recovery 证据链）

## 今天做了什么
把 W3D3 的 eval_v2（**action 终态 + BT recovery 统计**）用于一组新的 Day6 记录：每次导航只记录 1 行 CSV，回答——**“为什么这次慢/失败？有没有触发恢复？触发了什么？触发了几次？”**  
本日重点不是“调参变快”，而是把导航结果升级成**可解释的证据链**（可回放、可定位、可统计）。

---

## 复现步骤（单次记录模式）
> 目标：每跑一次只记录一条；流程是 **启动脚本 → RViz 点一次 Goal → 写一行 → 退出**。

### Step 0 — 前置条件（保持一致）
- Gazebo + Nav2 + RViz 已启动并正常工作  
- `use_sim_time` 全链一致（Gazebo / Nav2 / RViz）

### Step 1 — 终端 C：持续写 BT log（Day6 专用文件，避免混历史）
    source /opt/ros/humble/setup.bash
    rm -f /tmp/week3_day6_bt.log
    ros2 topic echo /behavior_tree_log > /tmp/week3_day6_bt.log

（自检：确认文件在增长）
    ls -lh /tmp/week3_day6_bt.log
    wc -l /tmp/week3_day6_bt.log
    tail -n 3 /tmp/week3_day6_bt.log

### Step 2 — 每次记录 1 条（终端 D / E 任意一个）
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week3_day6_bt.log \
      --out results/week3_day6_eval_v2.csv \
      --note clean

### Step 3 — RViz 点一次 Nav2 Goal
- **先启动脚本，再点 Goal**
- 小车完成一次导航后，脚本会：
  - 捕捉 action 终态（S/T/F）
  - 计算 `time_sec`
  - 对本次 run 的 BT log **增量切片**统计 recovery
  - 自动 append 一行 CSV 并退出

### Step 4 — 重复多次即可
- 每次都重复 Step 2 → Step 3
- 需要做 Cancel / 对抗点时，只要改 `--note`（例如 `cancel_test` / `corner` / `narrow`）

---

## CSV 字段解释（证据链口径）
- `run`：第几次记录（按文件现有行数递增）
- `goal_id`：本次 action goal 的 uuid（唯一定位一次导航）
- `result`：S=SUCCEEDED，T=CANCELED，F=ABORTED/FAIL（与 action 状态对应）
- `time_sec`：从捕捉到 active goal 开始计时，到终态的耗时
- `notes`：本次标签（例如 clean / cancel_test / recovery_test / corner / narrow）
- `recovery`：Y/N，本次 run 的 BT log 增量里是否命中恢复关键词
- `recovery_types`：命中的恢复类型去重列表（例如 `ClearLocalCostmap;BackUp`）
- `recovery_hits`：命中次数（反映恢复触发频率强弱）
- `bt_start_line` / `bt_end_line`：本次 run 切片在 BT log 里的行号范围（用于证据链定位与回查）

---

## 评估口径声明（非常关键：避免“不可比”）
本日数据**包含两类样本**，用途不同，**不混用做耗时对比**：

1) **基准样本（Comparable）**  
   - 终点：固定（例如 C 点）  
   - 用途：用于与 W3D2 / W3D4 / W3D5 的固定终点 action-time 做可比对照  
   - 指标：成功率、avg/median/min/max 等

2) **对抗样本（Adversarial / Stress test）**  
   - 终点：窄道 / 墙角 / 障碍密集区域等  
   - 用途：更容易触发 recovery，用于验证“恢复链路是否工作、触发了什么、触发几次、对耗时的影响方向”  
   - 指标：recovery=Y 的比例、types 分布、hits 强度、典型慢样本解释

> 说明：本文件的统计是“Day6 全样本汇总”。如果需要做严格耗时对比，应单独统计“固定终点子集”。

---

## 本次结果摘要（results/week3_day6_eval_v2.csv）
- 总 runs：11
- S：9，T：2，F：0
- 成功率：0.818

### 成功耗时（只统计 S）
- avg：23.807s
- median：13.956s
- min/max：11.661s / 75.658s

### Recovery 触发情况
- recovery=Y：3/11

明细：
- run5：`ClearLocalCostmap + ClearGlobalCostmap`（hits=6，34.040s）
- run6：`ClearLocalCostmap`（hits=2，19.942s）
- run10：`ClearLocalCostmap + BackUp`（hits=9，75.658s）

---

## 怎么解释这些现象
1) `T`（Cancel）不是算法失败，是人为中止，属于**对抗样本**，证明评估能记录非成功终态并落库（可用于演示“评估不是只会报喜”）。
2) `recovery=Y` 的样本耗时明显变长，符合直觉：恢复动作（清图/倒车等）会占用时间，但它的价值是**把任务从卡死边缘拉回成功**；因此“更慢但成功”是合理结果。
3) 单看 `avg_time` 会被极端慢样本拉高，所以同时报告 `median_time` 更稳健；Day6 的中位数仍然在 14s 左右，说明“多数样本并不慢”，慢的是“触发恢复的少数难例”。
4) `recovery_hits` 可以当作恢复强度的代理指标：hits 越高，通常意味着恢复动作重复触发、局部环境更难或更容易卡住，对应更高的 time_sec（如 run10）。

---

## 今日产出物
- `scripts/eval_v2_action_recovery.py`
- `results/week3_day6_eval_v2.csv`
- `docs/week3_day6_eval_v2.md`

---
