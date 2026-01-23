# Week2 Day6 — 10 Runs 量化记录（TB3 + Nav2）

## 固定口径
- 成功判定：Nav2 终端出现 `bt_navigator: Goal succeeded`
- 超时判定：120s 未成功 → Cancel（time_sec=120）
- 计时口径：RViz 发 Goal 起 → 终端 `Goal succeeded` 止（手机秒表）
- Recovery 判定（两条证据任选其一即可）：
  - RViz 面板 `Recoveries` > 0 ⇒ Recovery=Y；=0 ⇒ Recovery=N
  - BT log 新增内容命中任一关键词 ⇒ Recovery=Y：`ClearLocalCostmap|ClearGlobalCostmap|backup|spin|drive_on_heading|FAILURE`；否则 Recovery=N

## 起点 / 终点策略（固定）
- 起点：见 `day6_ABCDpoint.png`（本次使用图中标注的固定起点）
- 终点：使用图中 **C 点**（需要拐弯、经过狭窄区域、最终停在开阔位置）

## 10 次记录表（Baseline）
| run | result (S/T/F) | time_sec | recovery (Y/N) | notes |
|---:|:---:|---:|:---:|---|
| 1 | S | 14.6 | N | reached, recoveries=0 |
| 2 | S | 14.4 | N | reached, recoveries=0 |
| 3 | S | 14.6 | N | reached, recoveries=0 |
| 4 | S | 14.5 | N | reached, recoveries=0 |
| 5 | S | 14.7 | N | reached, recoveries=0 |
| 6 | S | 14.4 | N | reached, recoveries=0 |
| 7 | S | 14.6 | N | reached, recoveries=0 |
| 8 | S | 14.4 | N | reached, recoveries=0 |
| 9 | S | 14.7 | N | reached, recoveries=0 |
| 10 | S | 14.8 | N | reached, recoveries=0 |

## 快速结论（3 行）
- 成功率：10/10
- Recovery 发生次数：0/10（Recoveries=0，且 BT log 未命中 clear/backup/spin 等关键词）
- 耗时稳定性：平均 14.57 s；范围 14.4–14.8 s；波动很小（baseline 很稳）

## 采证据方式（本次执行记录）
- 终端 C（持续记录 BT log 到文件）：
```bash
    source /opt/ros/humble/setup.bash
    rm -f /tmp/w2d6_bt.log
    ros2 topic echo /behavior_tree_log > /tmp/w2d6_bt.log
```

- 终端 D（需要查看时，跟踪文件是否在持续写入）：
```bash
    tail -f /tmp/w2d6_bt.log
```
