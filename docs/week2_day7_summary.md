# Week2 Day7 — Week2 证据链一页纸（TF → Costmap → Planner/Controller → BT/Recovery → Baseline）

## 这一周我做成了什么
我把 Nav2 从“能跑 demo”变成“可解释、可复现、可量化”：每一层出问题我都能指出证据（日志/参数/截图/表格）。

---

## Nav2 链路
1) TF：先保证 map → odom → base_link 链路正常，否则 costmap / planner 会超时或失效  
2) Costmap：静态层/障碍层/膨胀层定义“哪里能走、靠近障碍代价多大”  
3) Planner：ComputePathToPose 请求 planner_server 计算全局路径  
4) Controller：FollowPath 请求 controller_server 跟随路径并持续输出 /cmd_vel  
5) BT（行为树）：bt_navigator 用 BT 编排“算路→走路→失败→恢复→重试”  
6) Baseline：固定同一起点/终点，跑 10 次，用统一口径记录结果与恢复发生情况

---

## Week2 证据目录

### Day1：导航链路一页纸（我能讲清每个模块干什么）
- docs/week2_day1_pipeline_onepager.md

### Day2：TF 参数证据（我能把 TF 的关键点落到参数/现象）
- docs/week2_day2_tf_values.md
说明：Week2 Day2 我目前只沉淀了 values（参数/输出记录）；后续如果补“排查剧本”，会新增 week2_day2_playbook_tf.md。

### Day3：Costmap 排查剧本（我能用证据解释 costmap 影响导航）
- docs/week2_day3_costmap_playbook.md

### Day4：Planner / Controller 参数记录（我能说清“哪个参数影响什么行为”）
- docs/week2_day4_planner_controller_values.md

### Day5：BT + Recovery 证据（我能证明“不是黑盒”：失败后进入恢复分支）
- docs/week2_day5_bt_recovery_values.md
我在日志里能指认的关键链路是：
- ComputePathToPose 周期性 SUCCESS：说明系统在持续 replanning（持续重规划）
- FollowPath FAILURE：触发恢复分支（NavigateRecovery）
- ClearLocalCostmap / backup / spin 等：恢复动作落地执行后回到 FollowPath 重试

### Day6：10 次 Baseline 量化记录（我能拿数据说话）
- docs/week2_day6_baseline.md
- picture/day6_ABCDpoint.png

---

## Week2 的结论（面试官最关心的 4 句话）
- 我能讲清 Nav2 的调度层（BT）和执行层（planner/controller/costmap/behavior）的关系。  
- 我能用 /behavior_tree_log 或 nav2 终端日志，把“失败发生在哪一层”说清楚并给出证据。  
- 我能复现恢复链路（FollowPath 失败 → clear costmap / backup / spin → 重试）。  
- 我能在固定条件下给 baseline 数据（成功率、时间、恢复次数），为 Week3 自动化评估做基线。

---

## 60 秒可背版本（照念）
我这周把 Nav2 的链路用证据跑通了：先保证 TF 链路稳定，然后 costmap 才能正常更新；bt_navigator 用行为树把“算路、走路、恢复”串起来。我用 /behavior_tree_log 抓到证据：ComputePathToPose 周期性 SUCCESS 说明系统在持续 replanning；当 FollowPath FAILURE，会进入恢复分支触发 clear local costmap 或 backup/spin 等动作，然后回到 FollowPath 重试。最后我用同一起点终点跑了 10 次 baseline，统一口径记录结果、耗时和恢复次数，证明我能把导航从“能跑”变成“可解释 + 可量化”的工程闭环。
