# Week3 Day6 — eval_v2（action + recovery 证据链）

## 今天做了什么
把 W3D3 的 eval_v2（action 终态 + BT recovery 统计）用于一组新的 Day6 记录：每次导航只记录 1 行 CSV，用于回答“失败/恢复是否触发、触发了什么、触发了几次”。

## 复现步骤（单次记录模式）
1) 启动 Gazebo + Nav2 + RViz（保持 use_sim_time 一致）
2) 另开一个终端持续写 BT log（今天用 Day6 专用文件，避免混到历史）：
    source /opt/ros/humble/setup.bash
    rm -f /tmp/week3_day6_bt.log
    ros2 topic echo /behavior_tree_log > /tmp/week3_day6_bt.log
3) 每次记录 1 条：
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py --btlog /tmp/week3_day6_bt.log --out results/week3_day6_eval_v2.csv --note clean
4) 在 RViz 点一次 Nav2 Goal（脚本先启动，再点 Goal）
5) 脚本捕捉到终态后自动落一行 CSV 并退出，重复多次即可

## CSV 字段解释
- run：第几次记录（按文件现有行数递增）
- goal_id：本次 action goal 的 uuid（用于唯一定位一次导航）
- result：S=SUCCEEDED，T=CANCELED，F=ABORTED/FAIL（口径与 action 状态对应）
- time_sec：从捕捉到 active goal 开始计时，到终态的耗时
- notes：本次标签（例如 clean / cancel_test / recovery_test）
- recovery：Y/N，本次 run 的 BT log 增量里是否命中恢复关键词
- recovery_types：命中的恢复类型去重列表（例如 ClearLocalCostmap;BackUp）
- recovery_hits：命中次数（反映恢复触发频率强弱）
- bt_start_line / bt_end_line：本次 run 切片在 BT log 里的行号范围（证据链定位用）

## 本次结果摘要（results/week3_day6_eval_v2.csv）
- 总 runs：11
- S：9，T：2，F：0
- 成功率：0.818
- 成功耗时（只统计 S）：
  - avg：23.807s
  - median：13.956s
  - min/max：11.661s / 75.658s
- recovery=Y：3/11
  - run5：ClearLocalCostmap + ClearGlobalCostmap（hits=6，34.040s）
  - run6：ClearLocalCostmap（hits=2，19.942s）
  - run10：ClearLocalCostmap + BackUp（hits=9，75.658s）

## 我怎么解释这些现象（面试可直接说）
1) Cancel（T）不是算法失败，是人为中止，属于“对抗样本”，证明评估能记录非成功终态。
2) recovery=Y 的样本耗时明显变长，符合直觉：恢复动作（清图/倒车等）会占用时间，但能把任务从卡死拉回成功。
3) 单看平均耗时会被极端慢样本拉高，所以我同时报告 median_time，避免误导。
