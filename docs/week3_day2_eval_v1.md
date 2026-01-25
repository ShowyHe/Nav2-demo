# Week3 Day2 — Eval v1 (action status auto timing)

## 做了什么
把 W3D1 的“手工 time_sec + 手工 result”升级为：从 Nav2 Action 状态自动得到 result(S/T/F) 与 time_sec。

## 原理
- Nav2 的导航目标是一个 action：/navigate_to_pose（server 在 /bt_navigator）
- action 的状态会在隐藏 topic 发布：/navigate_to_pose/_action/status
- 脚本订阅该 status：
  - 捕捉到新的 goal_id 后开始跟踪
  - 看到终态 SUCCEEDED / CANCELED / ABORTED 就判定 result=S/T/F
  - 用 ROS clock 的纳秒时间戳计算 time_sec
  - 写入 CSV 后自动退出（因此“回起点”的那次不计入）

## 输入
- topic: /navigate_to_pose/_action/status（隐藏 topic，需要 include-hidden-topics 才能在 list 中看到）

## 输出
- results/week3_day2_eval_action_v1.csv
  - run, goal_id, result, time_sec, start_ros_ns, end_ros_ns, notes

## 评估设置（为可比性固定）
- 起点：沿用 Week1 Day6 固定起点（见 picture/week1_day6_ABCDpoint.png）
- 终点：固定 C 点
- notes：本次统一 clean（clean：无明显抖动/卡墙/恢复动作，导航顺利完成（主观标签，仅用于快速筛选样本））

## 运行方式（单次记录）：
- 启动脚本 → RViz 点击一次 Nav2 Goal（起点→C）→ 等脚本自动写入并退出；回起点的导航不计入（因为脚本已退出）

## Summary
- runs=10
- success=10/10 (100.0%)
- avg_time=12.682s
- median_time=12.661s
- min_time=12.470s
- max_time=12.961s
