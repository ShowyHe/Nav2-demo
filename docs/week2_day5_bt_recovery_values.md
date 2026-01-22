# Week2 Day5 — BT Navigator / Recovery 证据链（TB3 + Nav2）

## 目的
- 用 `/behavior_tree_log` 做证据：导航流程由 **BT（行为树）调度**；`planner_server / controller_server / costmap / behavior_server` 是被 BT 调用的执行层。
- 留两份样本：**Success（正常导航）** / **Recovery（FollowPath 失败 → 清 costmap → 行为动作自救 → 回到 FollowPath 重试）**。
- 输出一句面试可背的“证据链总结”（见第 7 节）。

---

## 1) 证据入口：BT 日志话题 + 当前使用的 BT 树
命令：
    ros2 topic list | grep -E "behavior_tree_log" | sort
    ros2 param get /bt_navigator default_nav_to_pose_bt_xml

记录（我的真实输出）：
- `/behavior_tree_log`
- `default_nav_to_pose_bt_xml =`
  `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

---

## 2) 二层结构（调度层 / 执行层）

### 2.1 调度层（BT / bt_navigator）
- 决定：先做什么、后做什么、失败走哪条分支、多久重算一次、触发哪些恢复动作。
- `/behavior_tree_log` 记录的是：BT 节点状态变化（`IDLE / RUNNING / SUCCESS / FAILURE`）。

### 2.2 执行层（真正干活的 server）
- `planner_server`：算全局路径（Path）
- `controller_server`：跟随路径并持续输出 `/cmd_vel`
- `local_costmap / global_costmap`：维护代价地图 + 提供 `clear_*` 服务
- `behavior_server`：执行 `spin / backup / drive_on_heading / wait` 等行为动作（可作为恢复动作库）

---

## 3) BT 节点名 ↔ 执行层含义（对照表）

- `RateController`（BT 控制节点/节拍器）  
  作用：按固定频率 tick 子树，常用于周期性触发规划（replanning）。

- `ComputePathToPose`（BT 动作节点）  
  含义：BT 发起“计算路径”的请求（执行层由 `planner_server` 完成并返回 Path）。

- `FollowPath`（BT 动作节点）  
  含义：BT 发起“执行路径”的请求（执行层由 `controller_server` 持续输出 `/cmd_vel` 来跟随路径）。

- `NavigateRecovery`（恢复子树入口/容器）  
  含义：进入恢复分支的入口，本身不等于具体动作；具体动作是 Clear/Spin/BackUp 等。

- `ClearLocalCostmap-Context`（恢复动作节点）  
  含义：执行“清理 local costmap”恢复动作（落地到 `/local_costmap/clear_*` 等接口）。

---

## 4) 样本 A：Success（正常导航）证据

采集（20s 窗口）：
    timeout 20 ros2 topic echo /behavior_tree_log > /tmp/week2_day5_bt_success.log
    tail -n 120 /tmp/week2_day5_bt_success.log

Success 样本里要确认的模式：
- `RateController` 在 `RUNNING / SUCCESS` 间周期性出现（说明 BT 按节拍循环 tick）
- `ComputePathToPose` 周期性出现 `RUNNING → SUCCESS`（说明周期性规划成功 = replanning 正在发生）
- `bt_navigator` 终端日志出现：`Goal succeeded`

补充说明：
- `success.log` 命中 `NavigateRecovery` 不代表发生恢复动作；是否发生恢复，以是否出现 `ClearLocalCostmap / spin / backup / drive_on_heading` 等“具体动作节点”为准（见样本 B）。

（可选：关键字命中，方便面试逐行指认）
    grep -nE "NavigateRecovery|ComputePathToPose|RateController|FollowPath|ClearLocalCostmap|Spin|BackUp|backup|spin|FAILURE|SUCCESS|RUNNING" /tmp/week2_day5_bt_success.log | tail -n 120

---

## 5) 样本 B：Recovery（FollowPath 失败 → 清图/行为自救 → 回到 FollowPath）证据

B0 原始值（用于回滚）：
- `FollowPath.max_vel_theta = 1.0`
- `FollowPath.min_speed_theta = 0.0`

B1 触发器（只触发一次，不做对照实验）
目的：制造一次 `FollowPath` failure，让 BT 走进恢复分支采证据。  
方法：降低角速度上限（更易“转不动/卡住”）：
    ros2 param set /controller_server FollowPath.max_vel_theta 0.1
    ros2 param get /controller_server FollowPath.max_vel_theta

B2 采集 BT log（20s 窗口）：
    timeout 20 ros2 topic echo /behavior_tree_log > /tmp/week2_day5_bt_recovery.log
    tail -n 200 /tmp/week2_day5_bt_recovery.log

B3 Recovery 证据链（我在 log 里确认到的动作链）
- `FollowPath: RUNNING → FAILURE`
- `ClearLocalCostmap-Context: IDLE → SUCCESS → IDLE`
- `FollowPath: IDLE → RUNNING`
- 同时 `ComputePathToPose` 仍周期性 `RUNNING → SUCCESS`（replanning 未停止）

B4 额外系统级证据（来自 nav2 终端日志，证明“清图/行为动作”真的在执行层发生）
- `controller_server`：`Failed to make progress`（触发恢复的典型原因）
- `local_costmap.local_costmap`：`Received request to clear entirely the local_costmap`
- 有时还会出现：`global_costmap.global_costmap`：`Received request to clear entirely the global_costmap`
- `behavior_server`：`Running backup / Running spin`（行为自救动作确实被执行）

---

## 6) “清图接口”和“恢复动作库”证据（已按真实情况修正）

### 6.1 清图接口（local + global 都可能被调用）
命令（列出本系统提供的清图服务）：
    ros2 service list | grep -E "clear_.*costmap"

我的真实输出（节选）：
    /local_costmap/clear_around_local_costmap
    /local_costmap/clear_entirely_local_costmap
    /local_costmap/clear_except_local_costmap
    /global_costmap/clear_around_global_costmap
    /global_costmap/clear_entirely_global_costmap
    /global_costmap/clear_except_global_costmap

说明：
- BT log 中出现 `ClearLocalCostmap-Context`，证明 BT 在恢复分支触发了“清 local costmap”。
- nav2 终端日志中我也看到了实际执行：`clear entirely the local_costmap`，并且在更激进的恢复阶段也出现过 `clear entirely the global_costmap`。
- `around / entirely / except` 是清理范围/策略的不同：局部清、全清、清但保留某些部分；恢复策略会按失败程度升级清理强度。

### 6.2 恢复/行为动作库（behavior_server）
说明：当前系统存在 `/behavior_server`；查询 `/recoveries_server recovery_plugins` 时显示 `Node not found`，因此以 `behavior_server` 插件作为恢复动作库证据。

命令：
    ros2 param get /behavior_server behavior_plugins

我的真实输出：
    ['spin', 'backup', 'drive_on_heading', 'wait']

---

## 7) 面试可背总结（证据链）
`/behavior_tree_log` 记录 BT 节点状态变化。BT 通过 `ComputePathToPose` 调用 `planner_server` 周期性算路（由 `RateController` 控制 tick 频率），并通过 `FollowPath` 调用 `controller_server` 跟随路径持续输出 `/cmd_vel`。当 `FollowPath` 返回 `FAILURE`（例如 `controller_server` 报 `Failed to make progress`），BT 会进入恢复子树（`NavigateRecovery`），触发清图（如 `ClearLocalCostmap-Context`，对应 `/local_costmap/clear_*`，严重时也可能清 `/global_costmap/clear_*`），并可进一步调用 `behavior_server` 执行 `backup / spin` 等动作自救；随后回到 `FollowPath` 重试。同时 `ComputePathToPose` 仍周期性 `SUCCESS`，说明 replanning 持续进行。

---

## 8) 回滚（恢复参数）
    ros2 param set /controller_server FollowPath.max_vel_theta 1.0
    ros2 param get /controller_server FollowPath.max_vel_theta
