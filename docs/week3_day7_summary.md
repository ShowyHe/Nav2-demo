# Week3 Day7 — Weekly Summary (Day1–Day6)

## 这周做了什么
把 Week2 的“手工10次表格”升级成一条能复现、能量化、还能解释 failure/recovery 的评估证据链：Eval v0 → v1 → v2；并做了最小 A/B 对照实验，用数据说话。

---

## Day1–Day6 每天干了什么
- Day1（Eval v0 / BT log 切片）：基于 `/behavior_tree_log` 的增量切片统计 recovery，但 result/time 还是手填，先把“recovery证据链”跑通。
- Day2（Eval v1 / Action 自动计时）：订阅 `/navigate_to_pose/_action/status`，自动得到 result(S/T/F) + time_sec，彻底摆脱秒表。
- Day3（Eval v2 / Action + Recovery）：把 v1 的 action 结果 + v0 的 BT增量切片合并到一条 CSV：能回答“这次成功/失败是否触发清图/倒车/旋转、触发了几次”。
- Day4（最小 A/B）：只改 1 个参数，跑同口径 N 次，输出对照表（success_rate / avg/median/min/max）。
- Day5（汇总固化）：用 `summarize_action_csv.py` 固化统计口径，能一键汇总 baseline vs A。
- Day6（v2 日专用输出 + timeout口径）：不用 day3 的 CSV，重新记一份 day6 的 `week3_day6_eval_v2.csv`；补齐“捕捉到 goal 但没等到终态 → 记为失败(timeout)”的落库口径。

---

## 本周交付物（面试官点开就能复现）
### docs/
- `docs/week3_day1_eval_v0.md`
- `docs/week3_day2_eval_v1.md`
- `docs/week3_day3_eval_v2.md`
- `docs/week3_day4_AB_test.md`
- `docs/week3_day5_summary.md`
- `docs/week3_day6_eval_v2.md`
- `docs/week3_day7_summary.md`（本文件）

### results/
- `results/week3_day1_eval.csv`
- `results/week3_day2_eval_action_v1.csv`
- `results/week3_day3_eval_v2.csv`
- `results/week3_day4_baseline_action.csv`
- `results/week3_day4_A_action.csv`
- `results/week3_day5_baseline_action.csv`
- `results/week3_day5_A_action.csv`
- `results/week3_day6_eval_v2.csv`

---

## 固定评估口径（保证可比）
- 起点：沿用 Week1 Day6 固定起点（picture/week1_day6_ABCDpoint.png）
- 终点：从 Day2 起固定 C 点（Day1 早期是 A 点，后面统一）
- 单次记录模式：启动脚本 → RViz 点一次 Nav2 Goal → 写一行 CSV → 脚本退出（回起点那次默认不计入）

---

## 关键结果（这周最能打的数字）

### Day2（Eval v1 / action 自动计时）
- runs=10
- success=10/10 (100%)
- avg_time=12.682s, median=12.661s, min=12.470s, max=12.961s

### Day3（Eval v2 / action + recovery）
- 既有 cancel 样本：result=T
- 也有“成功但触发恢复”的样本：result=S 且 recovery=Y（例如 ClearLocalCostmap 命中）
=> 说明我能解释“慢/卡/失败是否走了恢复链路”，不是只报成功率。

### Day4（最小 A/B 对照实验：只改 1 个参数）
- 我做的是最小对照：baseline 跑 N 次，A 组只改 1 个参数再跑同样 N 次。
- 产出文件：
  - `docs/week3_day4_AB_test.md`
  - `results/week3_day4_baseline_action.csv`
  - `results/week3_day4_A_action.csv`
- 对照表口径：success_rate / avg_time / median_time / min/max（来自 action 自动计时 CSV）
- 结论：A 组和 baseline 的差异能用“只改1个参数”解释，不是乱调一堆参数堆出来的运气结果。

### Day5（baseline vs A 的对照结论，take=10 保证两组可比）
baseline（results/week3_day5_baseline_action.csv）
- success_rate: 0.900
- avg_time: 13.148
- median_time: 13.031
- min_time: 12.561
- max_time: 14.007

A（results/week3_day5_A_action.csv）
- success_rate: 1.000
- avg_time: 12.736
- median_time: 12.611
- min_time: 12.470
- max_time: 13.505

### Day6（Eval v2：action + recovery，能解释慢/卡/恢复）
- 产出文件：
  - `docs/week3_day6_eval_v2.md`
  - `results/week3_day6_eval_v2.csv`
- 这天的重点不是成功率，是“解释链路”：CSV 里每条 run 除了 result/time_sec，还会记录：
  - recovery(Y/N)
  - recovery_types（例如 ClearLocalCostmap / ClearGlobalCostmap / BackUp）
  - recovery_hits（触发次数）
- 我这份 day6 数据里明确出现了：
  - `ClearLocalCostmap;ClearGlobalCostmap`（清图触发）
  - `ClearLocalCostmap;BackUp`（清局部+倒车）
- 解释口径：
  - 某些 run 耗时明显变长，不是“随机慢”，而是 recovery=Y 且 recovery_types 命中清图/倒车，属于恢复链路介入后的可解释变慢。

结论（我能30秒复述）
1) A 组成功率更高，平均/中位耗时更低（同口径、同方法统计）。
2) 我这周不追求“调参一定变好”，追求“实验像样”：只改1个参数 + 同起终点 + 同次数 + 同计时口径。
3) 评估已经可解释：慢样本可以用 recovery 字段解释（是否触发清图/倒车/旋转、触发次数）。

---

## 一套最小复现步骤
1) 启动 Gazebo + Nav2 + RViz（沿用 Week1 标准 demo）
2) Eval v1（action 单次记录）
   - `python3 scripts/eval_v1_action_logger.py --note clean --out results/xxx.csv`
   - 去 RViz 点一次 Nav2 Goal（起点->C）
3) Eval v2（action + recovery 单次记录）
   - 终端持续写：`ros2 topic echo /behavior_tree_log > /tmp/week3_day6_bt.log`
   - 记录一次：`python3 scripts/eval_v2_action_recovery.py --btlog /tmp/week3_day6_bt.log --out results/week3_day6_eval_v2.csv --note clean`
   - 去 RViz 点一次 Nav2 Goal

---

## 本周收获
- 把“现象”变成“证据”：S/T/F、time_sec、recovery_types/hits 都能落库。
- 会做最小 A/B：只改 1 个参数 + 固定口径 + 输出对照数字。
- 能解释 failure/recovery：不仅说“失败了/慢了”，还能说“是否触发恢复、触发了哪些、触发了几次”。
