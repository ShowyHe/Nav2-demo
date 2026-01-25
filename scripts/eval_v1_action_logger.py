#!/usr/bin/env python3
import argparse
import csv
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray

def uuid_to_hex(u) -> str:
    return "".join([f"{b:02x}" for b in u])

# action_msgs/GoalStatus status codes
ACCEPTED = 1
EXECUTING = 2
CANCELING = 3
SUCCEEDED = 4
CANCELED = 5
ABORTED = 6

class ActionStatusLogger(Node):
    """
    单次记录模式（修正版）：
    - 不依赖“baseline非空”，避免空status导致永远不初始化
    - 只要捕捉到第一个 active goal_id，就开始跟踪
    - 终态(S/T/F)自动写CSV并退出
    """
    def __init__(self, out_csv: str, note: str, wait_timeout: float):
        super().__init__("eval_v1_action_logger")
        self.out_csv = out_csv
        self.note = note.strip()
        self.wait_timeout = wait_timeout

        os.makedirs(os.path.dirname(self.out_csv), exist_ok=True)
        if not os.path.exists(self.out_csv):
            with open(self.out_csv, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(
                    f,
                    fieldnames=["run","goal_id","result","time_sec","start_ros_ns","end_ros_ns","notes"],
                )
                w.writeheader()

        self.target_goal_id: Optional[str] = None
        self.start_ns: Optional[int] = None
        self.started_at_ns = self.get_clock().now().nanoseconds

        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.on_status,
            10,
        )
        self.create_timer(0.2, self.on_timer)

        self.get_logger().info("W3D2 eval_v1: 已启动（单次记录模式）")
        self.get_logger().info("现在去 RViz 点一次 Nav2 Goal（起点->A），我会自动记录并退出。")
        self.get_logger().info(f"Output: {self.out_csv}")

    def on_timer(self):
        if self.target_goal_id is None:
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - self.started_at_ns) / 1e9 > self.wait_timeout:
                self.get_logger().error("超时：没捕捉到任何 goal status。")
                self.get_logger().error("请确认你点了 Nav2 Goal，并且 /navigate_to_pose/_action/status 在刷。")
                rclpy.shutdown()

    def on_status(self, msg: GoalStatusArray):
        now_ns = self.get_clock().now().nanoseconds
        if not msg.status_list:
            return

        # 选择一个“最像正在跑”的goal：优先 EXECUTING/ACCEPTED/CANCELING，避免拿到已结束残留
        pick = None
        for st in msg.status_list:
            if st.status in (EXECUTING, ACCEPTED, CANCELING):
                pick = st
                break
        if pick is None:
            pick = msg.status_list[0]

        gid = uuid_to_hex(pick.goal_info.goal_id.uuid)

        # 第一次见到 active goal：锁定为目标
        if self.target_goal_id is None:
            self.target_goal_id = gid
            self.start_ns = None
            self.get_logger().info(f"捕捉到 goal_id={self.target_goal_id}，开始跟踪…")

        # 只跟踪锁定的目标 goal
        for st in msg.status_list:
            cur = uuid_to_hex(st.goal_info.goal_id.uuid)
            if cur != self.target_goal_id:
                continue

            if self.start_ns is None and st.status in (ACCEPTED, EXECUTING, CANCELING):
                self.start_ns = now_ns

            if st.status in (SUCCEEDED, CANCELED, ABORTED):
                end_ns = now_ns
                if self.start_ns is None:
                    self.start_ns = end_ns
                dt = (end_ns - self.start_ns) / 1e9

                if st.status == SUCCEEDED:
                    res = "S"
                elif st.status == CANCELED:
                    res = "T"
                else:
                    res = "F"

                self.append_row(self.target_goal_id, res, dt, self.start_ns, end_ns, self.note)
                self.get_logger().info(f"[logged] result={res} time_sec={dt:.3f} notes={self.note or '-'}")
                rclpy.shutdown()
                return

    def append_row(self, goal_id: str, result: str, time_sec: float, start_ns: int, end_ns: int, note: str):
        run_id = 1
        try:
            with open(self.out_csv, "r", encoding="utf-8") as f:
                run_id = max(1, sum(1 for _ in f))  # header+rows
        except Exception:
            run_id = 1

        row = {
            "run": str(run_id),
            "goal_id": goal_id,
            "result": result,
            "time_sec": f"{time_sec:.3f}",
            "start_ros_ns": str(start_ns),
            "end_ros_ns": str(end_ns),
            "notes": note,
        }

        with open(self.out_csv, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(
                f,
                fieldnames=["run","goal_id","result","time_sec","start_ros_ns","end_ros_ns","notes"],
            )
            w.writerow(row)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results/week3_day2_eval_v1.csv")
    ap.add_argument("--note", default="clean")
    ap.add_argument("--wait_timeout", type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    node = ActionStatusLogger(args.out, args.note, args.wait_timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
