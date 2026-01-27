# Week 4 Day 1 — 实验设计与参数选择（Nav2）

## 1. 目的
建立可复现的 2×2 对照实验（baseline / A / B / A+B），量化两项参数对导航结果的影响，包括成功率、完成时间以及与恢复行为相关的表现。

---

## 2. 可复现条件（Week 4 全程保持一致）
为确保各组结果可直接对比，Week 4 全部实验统一采用以下条件：

- **地图/场景**：沿用 `week1_day6_ABCDpoint.png`（位于 `picture/`，你此前所有实验均基于该图）。
- **起点位姿**：固定为 `week1_day6_ABCDpoint.png` 中对应的起点位姿（与既往实验一致）。
- **目标点位**：固定为 `week1_day6_ABCDpoint.png` 标注的 **C 点**。
- **每组运行次数**：20 runs / group。
- **超时判据**：120 s / run。
- **碰撞判据（简化）**：沿用 Week 1 README 中已定义的简化碰撞判据（Week 4 不作调整）。
- **成功判据（执行口径）**：Nav2 动作返回状态为 **SUCCEEDED** 记为成功。
  - 说明：参数 A 会改变 goal checker 的判定阈值，因此各组将明确记录该阈值，并在结论中解释其对 success/time 的影响。

---

## 3. 基线配置来源（以运行实例为准）
本次 Nav2 启动命令未显式传入 `params_file:=...`，launch 系统会将参数合并写入临时文件，并由运行中的容器进程加载。

- **运行进程证据（节选）**：

      /opt/ros/humble/lib/rclcpp_components/component_container_isolated \
        ... \
        --params-file /tmp/launch_params_omo6ye1w \
        --params-file /tmp/launch_params_65b1n736 \
        ...

- **基线参数文件（运行时合并产物）**：
  - `/tmp/launch_params_omo6ye1w`
  - `/tmp/launch_params_65b1n736`

Week 4 的基线参数值均以以上运行实例文件为依据。

---

## 4. 自变量（Week 4 仅变更两项参数）
Week 4 仅选择两项参数作为自变量，保证解释清晰且避免引入无关耦合。

### 4.1 参数 A — 到达判定阈值（Goal Checker）
- **所属组件**：`controller_server`
- **插件**：`general_goal_checker`（`nav2_controller::SimpleGoalChecker`）
- **作用**：定义何时判定“到达目标”，从而触发动作状态进入 **SUCCEEDED**。

**基线值（运行实例证据）**  
来自 `/tmp/launch_params_65b1n736` 的 `general_goal_checker` 配置块：

- `general_goal_checker.xy_goal_tolerance = 0.25`
- `general_goal_checker.yaw_goal_tolerance = 0.25`（Week 4 保持不变）

**Week 4 设置（A 组）**
- `general_goal_checker.xy_goal_tolerance = 0.40`
- `general_goal_checker.yaw_goal_tolerance = 0.25`（不变）

**预期影响（工作假设）**
更大的位置容差可能提高动作达到 **SUCCEEDED** 的概率，并可能降低到达阶段的收敛时间；具体效果以评估数据为准。

**重要说明（避免误改）**
运行配置中同时存在 `dwb_core::DWBLocalPlanner` 下的 `xy_goal_tolerance = 0.05`，该值属于 DWB 控制器内部参数，不等价于“到达判定阈值”。Week 4 的参数 A 指的是 `general_goal_checker.xy_goal_tolerance`。

---

### 4.2 参数 B — 进度判定时间窗口（Progress Checker）
- **所属组件**：`controller_server`
- **插件**：`progress_checker`（`nav2_controller::SimpleProgressChecker`）
- **作用**：用于判定是否“缺乏进度/可能卡住”，并影响恢复行为触发。

**基线值（运行实例证据）**  
来自 `/tmp/launch_params_65b1n736`：

- `progress_checker.movement_time_allowance = 10.0`（该值在文件中可定位到第 157 行附近）

**Week 4 设置（B 组）**
- `progress_checker.movement_time_allowance = 15.0`

**预期影响（工作假设）**
增大时间窗口可能减少过早触发“卡住判定/恢复”的情况，但对真实卡住样本可能带来耗时增加；具体效果以评估数据为准。

---

## 5. 实验分组（2×2 对照设计）
Week 4 评估以下四组配置：

- **Baseline**：A=baseline，B=baseline
- **A only**：A=0.40，B=baseline
- **B only**：A=baseline，B=15.0
- **A+B**：A=0.40，B=15.0

---

## 6. Day 1 完成条件
Day 1 完成的判定标准如下：

- 已明确并记录 Week 4 的统一条件（地图、起点、目标 C 点、运行次数、timeout、success/collision 判据）。
- 已确认基线参数来源为运行实例的 `/tmp/launch_params_*` 合并文件。
- 已选定参数 A 与参数 B，并记录其基线值与 Week 4 目标值。
- 已明确 Week 4 采用 2×2 对照设计，并固定输出文件命名规范。
