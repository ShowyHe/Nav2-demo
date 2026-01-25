# Week 1 Day 7 — 复盘（Nav2 跑通 + Baseline 数据）

## 1) 总览

我在 TurtleBot3 的 Gazebo 仿真里跑通了 Nav2，并把评估口径固定为可复现的标准：  
从 `bt_navigator` 日志中提取 goal 的 (x, y)（map 坐标系），从 `/amcl_pose` 读取机器人最终位置（map 坐标系），计算两者的欧氏距离 FinalDist。**FinalDist ≤ 0.25 m 判定为 Success**。

在 baseline 条件下（同一起点、同一张地图、固定四个目标点 A/B/C/D），我跑了 12 次：  
**D 点 3/3 成功，A 点 0/3 失败，C 点 0/3 失败，B 点 1/3 成功**，总体 **4/12 = 33.3%**。

这说明机器人多数情况下能走到目标附近，但存在“Nav2 输出 `Goal succeeded` 与我固定评估口径不一致”的情况，部分 run 会停在距离目标约 0.3–0.4m 的位置。  
下一步我会围绕 **costmap 膨胀** 与 **goal checker / controller 的到点行为**做 **2×2 对照实验（baseline / A / B / A+B）**，目标是在相同评估标准下把成功率提升到 **≥85%**，并用数据验证改动有效性。

## 2) Nav2 链路说明

### TF（坐标变换）
- **输入**：`/tf`、`/tf_static`（关键链路 `map → odom → base_link` 必须稳定存在）
- **输出**：让“地图、机器人姿态、传感器数据”落在同一坐标系中
- **坏了会怎样**：RViz 报错、costmap 不更新/错位、planner 超时、AMCL 无法发布 transform

### Costmap（代价地图：全局/局部）
- **输入**：静态地图（静态层）、传感器障碍（障碍层）、TF（把数据变换到 map/odom）
- **输出**：`global_costmap` 与 `local_costmap` 栅格（含 inflation 膨胀安全距离）
- **坏了会怎样**：过度保守（贴墙/窄道不敢走、到点停得远）、过度激进（贴障碍风险高）、幽灵障碍导致原地抖动/打转

### Planner（全局规划）
- **输入**：`global_costmap`、起点、目标点
- **输出**：全局路径（global plan）
- **坏了会怎样**：No valid plan、规划频率下降、超时

### Controller（局部控制）
- **输入**：`local_costmap`、当前姿态、全局路径
- **输出**：速度指令 `/cmd_vel`
- **坏了会怎样**：原地打转、抖动、走走停停、到点附近犹豫/提前停下

### BT Navigator（行为树与恢复）
- **输入**：目标点 + planner/controller 反馈 + 恢复行为（Spin/BackUp 等）
- **输出**：导航流程编排（规划 → 控制 → 必要时恢复 → 继续）
- **备注**：Nav2 的 `Goal succeeded` 是系统内部条件；为了可对比实验，我使用固定的 FinalDist 口径统一衡量 Success。

## 3) 证据与数据（Week1）
- Day5 Demo 视频：`videos/week1_day5_demo.mp4`
- Day6 Baseline 数据：`results/w1d6_baseline.csv`
- 目标点截图（A/B/C/D）：`picture/week1_day6_ABCDpoint.png`
