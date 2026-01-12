# Week2 Day1 — Nav2 链路一页纸（面试版）

## 0. 一句话总览（10 秒）
Nav2 = **定位（AMCL/TF） + 代价地图（costmap） + 全局规划（planner） + 局部控制（controller） + 行为树编排（BT）**，把目标点变成 `/cmd_vel`，并在失败时用恢复行为继续推进。

## 1. 数据流：从“点目标”到“机器人动起来”
**目标输入**：RViz `Nav2 Goal`（NavigateToPose action）  
↓  
**BT Navigator**：按行为树流程驱动（规划→控制→必要时恢复→继续）  
↓  
**Planner Server**：在 `global_costmap` 上算全局路径（global plan）  
↓  
**Controller Server**：在 `local_costmap` 上跟踪路径，输出 `/cmd_vel`     
↓  
**机器人运动**：Gazebo/底盘执行，产生里程计 `odom` 与 TF（`odom→base_link`）  
↓  
**AMCL**：融合激光/地图，发布 `map→odom` 与 `/amcl_pose`  
↓  
**TF 闭环**：`map→odom→base_link` 让所有模块对齐坐标系

## 2. 每个模块三句话（输入/输出/坏了会怎样）

### 2.1 TF（坐标变换）
- 输入：`/tf`、`/tf_static`（核心链路 `map→odom→base_link`）
- 输出：把地图/机器人/传感器统一到同一坐标系
- 坏了：RViz 红、costmap 错位/不更新、planner 超时、AMCL 警告无法发布 transform

### 2.2 定位（AMCL）
- 输入：激光扫描 + 地图 + TF + 初始位姿（2D Pose Estimate）
- 输出：`map→odom` + `/amcl_pose`
- 坏了：提示 “Please set the initial pose”，导航起不来或漂移严重

### 2.3 Costmap（全局/局部代价地图）
- 输入：静态地图（static layer）+ 传感器障碍（obstacle layer）+ inflation + TF
- 输出：`global_costmap` / `local_costmap`
- 坏了：膨胀过大→过度保守/到点停很远；膨胀过小→贴障碍风险；幽灵障碍→抖动/打转

### 2.4 Planner（全局规划）
- 输入：`global_costmap` + 起点 + 目标点
- 输出：全局路径 plan
- 坏了：No valid plan / 频率掉 / 超时（计算负载或 costmap/TF 问题）

### 2.5 Controller（局部控制）
- 输入：`local_costmap` + 当前姿态 + 全局路径
- 输出：`/cmd_vel` + 局部轨迹
- 坏了：原地打转/抖动/走走停停/到点犹豫（常与局部代价、到点策略、速度限制相关）

### 2.6 BT Navigator（行为树 + 恢复）
- 输入：目标点 + planner/controller 反馈 + 恢复行为（Spin/BackUp 等）
- 输出：导航执行流程（含恢复）
- 坏了：卡在恢复循环、频繁 spin、或系统报 succeeded 但实际停得偏远（需要统一评估口径）

## 3. Week1 Baseline 现象（用数据说话）
- 评估口径：goal (x,y) 来自 `bt_navigator` log；final 来自 `/amcl_pose`；FinalDist ≤ **0.25 m** 判 Success
- 结果：12 次 baseline：D 3/3 成功，A 0/3，C 0/3，B 1/3，总体 4/12
- 解释：多数能靠近目标，但存在 “Nav2 Goal succeeded 与 FinalDist 口径不一致/提前停下” 的现象，后续用 2×2 对照实验定位（costmap inflation vs goal checker/controller）

## 4. 我今天要能回答的 3 个问题（自测）
1) 机器人不动了，我第一眼看 TF 还是 /cmd_vel？为什么？【TODO：写你的答案一句话】
2) RViz 里 global/local costmap 看起来差很多，可能是哪两类原因？【TODO：写两条】
3) 为什么我不用 “Nav2 succeeded” 直接当 success？【TODO：写一句话】
