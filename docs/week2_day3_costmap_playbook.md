# Week2 Day3 — Costmap 实验 + 排查剧本（TB3 + Nav2）

目标：用 3 个可复现实验证明 costmap 的关键层如何影响机器人行为，并形成可背诵的排查剧本（面试可直接复述）。

---

## 0. Costmap实验成果

### 截图（RViz 同屏 global + local costmap）
- picture/costmap/week2_day3/week2_day3_baseline.png
- picture/costmap/week2_day3/week2_day3_inflation_big.png
- picture/costmap/week2_day3/week2_day3_inflation_small.png
- picture/costmap/week2_day3/week2_day3_no_obstacle.png

### 参数与数据
- /scan 频率（证明 obstacle layer 有数据源）
- inflation_radius / cost_scaling_factor / obstacle_layer.enabled（Baseline + 实验后）

---

## 1) Baseline（正常）

### 1.1 RViz 证据
- 图：picture/costmap/week2_day3/week2_day3_baseline.png

### 1.2 /scan 频率（数据源证据）

命令：
```bash
ros2 topic hz /scan
```

输出（证据）：
```text
average rate: 4.993
        min: 0.200s max: 0.201s std dev: 0.00031s window: 6
average rate: 4.994
        min: 0.199s max: 0.201s std dev: 0.00037s window: 11
average rate: 4.994
        min: 0.199s max: 0.201s std dev: 0.00033s window: 16
```

解释：  
/scan 平均约 5Hz，说明 LiDAR 数据源正常，obstacle_layer 有输入。

### 1.3 Baseline 参数（local_costmap）

命令：
```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.cost_scaling_factor
ros2 param get /local_costmap/local_costmap obstacle_layer.enabled
```

输出（证据）：
```text
Double value is: 1.0
Double value is: 3.0
Boolean value is: True
```

现象一句话：  
baseline 下 local costmap 为 rolling window，障碍周围存在膨胀代价带，机器人保持合理安全距离。

---

## 2) 实验A：膨胀过大（“怂包”）

### 2.1 修改参数（只改 local costmap inflation）

命令：
```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 1.2
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 10.0
```

输出：
```text
Set parameter successful
Set parameter successful
```

### 2.2 RViz 截图证据
- 图：picture/costmap/week2_day3/week2_day3_inflation_big.png

### 2.3 参数确认（证据）

命令：
```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.cost_scaling_factor
```

输出（证据）：
```text
Double value is: 1.2
Double value is: 10.0
```

现象一句话（行为层面）：  
更保守、更容易绕路，窄通道通过困难。

解释（核心机制）：  
- inflation_radius 变大 → 膨胀影响范围变大（离障碍更远也会被赋予代价）  
- cost_scaling_factor 变大 → 代价衰减更快（红色高代价区可能变窄，但影响范围仍在）

---

## 3) 实验B：膨胀过小（“莽夫”）

### 3.1 修改参数

命令：
```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.10
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 1.0
```

输出：
```text
Set parameter successful
Set parameter successful
```

### 3.2 RViz 截图证据
- 图：picture/costmap/week2_day3/week2_day3_inflation_small.png

### 3.3 参数确认（证据）

命令：
```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.cost_scaling_factor
```

输出（证据）：
```text
Double value is: 0.1
Double value is: 1.0
```

现象一句话：  
膨胀范围非常小，除障碍附近外几乎无代价，机器人更敢贴墙走，碰撞风险上升（更激进）。

---

## 4)实验C：关障碍层（“瞎子模式”）

### 4.1 恢复 baseline inflation（保证只测试 obstacle_layer）

命令：
```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 1.0
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 3.0
```

输出：
```text
Set parameter successful
Set parameter successful
```

### 4.2 关闭 obstacle_layer

命令：
```bash
ros2 param set /local_costmap/local_costmap obstacle_layer.enabled False
```

输出：
```text
Set parameter successful
```

### 4.3 RViz 截图证据
- 图：picture/costmap/week2_day3/week2_day3_no_obstacle.png

现象一句话：  
关闭 obstacle_layer 后，/scan 产生的动态障碍不会再写入 costmap（机器人对动态障碍“变瞎”）。

解释：  
静态墙柱来自 static_layer（地图层），所以在纯静态地图场景里变化可能不明显；但遇到动态物体/临时障碍时会明显失效。

### 4.4 打开 obstacle_layer

命令：
```bash
ros2 param set /local_costmap/local_costmap obstacle_layer.enabled True
```

输出：
```text
Set parameter successful
```

---

## 5) 可复现闭环：恢复 baseline

命令：
```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 1.0
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 3.0
ros2 param set /local_costmap/local_costmap obstacle_layer.enabled True
```

输出：
```text
Set parameter successful
Set parameter successful
Set parameter successful
```

---

## 6) 参数含义总结

### inflation_radius（膨胀半径）
含义：离障碍多远仍会被膨胀层赋予代价（影响范围）。

### cost_scaling_factor（代价衰减速度）
含义：代价从障碍边缘往外下降得有多快（危险区厚度）。
- scaling 越大 → 衰减越快 → 高代价红区更窄（更快变安全）
- scaling 越小 → 衰减越慢 → 高代价区更宽（更保守）

一句话总括：  
inflation_radius 管“影响范围”，cost_scaling_factor 管“危险衰减速度”。

---

## 7) 排查剧本（工程闭环版）

### 7.1 先看有没有数据源

```bash
ros2 topic hz /scan
```

解释：  
没频率/没数据 → obstacle_layer 再对也没用。

### 7.2 再看 global/local costmap 是否正常（可视化证据）
RViz 同时打开：
- /global_costmap/costmap（全局地图，用于 planner）
- /local_costmap/costmap（局部 rolling window，用于 controller）

解释：  
local 跟着机器人移动是正常现象；global 覆盖全局是正常现象（两者不同是正常的）。

### 7.3 用最小改动定位“性格问题”
- 太怂：适当减小 inflation_radius 或降低 cost_scaling_factor
- 太莽：适当增大 inflation_radius 或提高 cost_scaling_factor
- 完全不避障：确认 obstacle_layer.enabled=True + /scan 有频率

一句话总结：  
“膨胀大机器人更保守，膨胀小机器人更激进，关障碍层机器人对动态障碍变瞎。参考四张截图和参数输出。”
