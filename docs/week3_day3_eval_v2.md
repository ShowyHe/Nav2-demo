# Week3 Day3 — Eval v2（Action 结果 + Recovery 证据）

## 目的
在 W3D2 的 action 计时基础上，增加“是否触发恢复/触发了什么/触发次数”的证据链，用于解释 failure/recovery。

## 数据来源
- action 状态：/navigate_to_pose/_action/status（捕捉 goal_id，记录 result 与耗时）
- BT 日志：/behavior_tree_log（持续写入 /tmp/w3d3_bt.log，本次 run 使用增量切片统计 recovery）

## 输出文件
- scripts/eval_v2_action_plus_recovery.py
- results/week3_day3_eval_v2.csv

## CSV 字段
run, goal_id, result(S/T/F), time_sec, notes, recovery(Y/N), recovery_types, recovery_hits, bt_start_line, bt_end_line

## 复现步骤
1) 终端 C：ros2 topic echo /behavior_tree_log > /tmp/w3d3_bt.log
2) 终端 D：python3 scripts/eval_v2_action_plus_recovery.py --note <tag>
3) RViz 点击 Nav2 Goal（脚本自动记录并写入 CSV）

## 今日样本
- cancel_test：result=T（手动 Cancel）
- recovery_test（点墙角）：result=S 但 recovery=Y，recovery_types=ClearLocalCostmap，hits=2
