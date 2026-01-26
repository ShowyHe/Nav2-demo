# Week3 Day5 — Eval 工程化总结（Baseline vs A）

## 我做了什么（打包 + 结果页合一）
- 把 Week3 的 action 评估结果固化成“可复现 + 可对比 + 可解释”的一页证据链：
  - 原始证据：results/ 下的 CSV
  - 复现工具：scripts/summarize_action_csv.py
  - 展示结论：本文件（summary + packaging 合并）

---

## 产物清单
- results/week3_day5_baseline_action.csv
- results/week3_day5_A_action.csv
- scripts/summarize_action_csv.py
- docs/week3_day5_summary.md（本文件）

---

## 复现步骤（最短路径）
1) 确保有两份数据：
   - results/week3_day5_baseline_action.csv
   - results/week3_day5_A_action.csv
2) 执行汇总（只取前 10 条用于公平对照，避免多跑混入）：
   - python3 scripts/summarize_action_csv.py --in results/week3_day5_baseline_action.csv --take 10
   - python3 scripts/summarize_action_csv.py --in results/week3_day5_A_action.csv --take 10

---

## 指标口径
- result：S=SUCCEEDED，F=ABORTED，T=CANCELED
- time_sec：从 /navigate_to_pose action 终态自动计时
- 对照规则：统一取前 N=10 条样本

---

## 汇总结果（N=10）
### baseline
- runs: 10
- success: 9
- success_rate: 0.900
- avg_time: 13.148 s
- median_time: 13.031 s
- min_time: 12.561 s
- max_time: 14.007 s

### A
- runs: 10
- success: 10
- success_rate: 1.000
- avg_time: 12.736 s
- median_time: 12.611 s
- min_time: 12.470 s
- max_time: 13.505 s

---

## 结论（3 条）
1) 成功率：A 组 10/10（100%）> baseline 9/10（90%），A 组在本次固定起点/终点设置下更稳定。
2) 耗时：A 组 avg/median 更低（13.148→12.736，13.031→12.611），说明 A 组更快进入终态（到达判据更容易满足 / 或行为更顺滑）。
3) 这份对照的意义：用固定口径 + 原始 CSV + 汇总脚本，把一次改动的影响量化成可复现证据链，而不是“我感觉”。

---

## 备注
- baseline 只有 1 次未记录为成功（success=9）。如果要深挖，就拿那条对应 goal_id 回看当次 action 终态与行为日志。
