#!/usr/bin/env python3
import argparse
import csv
import os
import time
from typing import List, Tuple, Set

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray

ACTIVE = {1, 2, 3}      # ACCEPTED/EXECUTING/CANCELING
TERMINAL = {4, 5, 6}    # SUCCEEDED/CANCELED/ABORTED

def ns_now(node: Node) -> int:
    return int(node.get_clock().now().nanoseconds)

def goal_uuid_hex(goal_id) -> str:
    # Humble: goal_id.uuid is uint8[16] (often numpy.ndarray)
    try:
        return bytes(goal_id.uuid).hex()
    except Exception:
        return "".join(f"{int(b):02x}" for b in goal_id.uuid)

def load_keywords(path: str) -> List[str]:
    kws = []
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            kws.append(ln)
    return kws

def read_lines(path: str) -> List[str]:
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        return f.readlines()

def count_recovery_hits(lines: List[str], keywords: List[str]) -> Tuple[str, str, int]:
    hits = []
    total = 0
    for ln in lines:
        for kw in keywords:
            if kw in ln:
                total += 1
                hits.append(kw)

    uniq = []
    seen: Set[str] = set()
    for x in hits:
        if x not in seen:
            uniq.append(x)
            seen.add(x)

    recovery = "Y" if total > 0 else "N"
    return recovery, ";".join(uniq), total

def next_run_id(csv_path: str) -> int:
    if not os.path.exists(csv_path):
        return 1
    # 行数 = 表头 + 数据行，下一次 run 就用“当前总行数”
    with open(csv_path, "r", encoding="utf-8", errors="ignore") as f:
        n = sum(1 for _ in f)
    return max(1, n)

class EvalV2(Node):
    def __init__(self, args):
        super().__init__("eval_v2_action_plus_recovery")
        self.args = args

        self.btlog = args.btlog
        self.keywords = load_keywords(args.keywords)

        self.goal_id = None
        self.start_ns = None
        self.end_ns = None
        self.result = None

        self.bt_start_line = self._bt_line_count()

        self.sub = self.create_subscription(
            GoalStatusArray,
            args.status_topic,
            self.on_status,
            10
        )

        self.get_logger().info("W3D3 eval_v2: 启动成功")
        self.get_logger().info(f"Action status topic: {args.status_topic}")
        self.get_logger().info(f"BT log: {self.btlog}")
        self.get_logger().info(f"bt_start_line={self.bt_start_line}")
        self.get_logger().info("现在去 RViz 点一次 Nav2 Goal（先启动脚本，再点）")

    def _bt_line_count(self) -> int:
        if not os.path.exists(self.btlog):
            return 0
        with open(self.btlog, "r", encoding="utf-8", errors="ignore") as f:
            return sum(1 for _ in f)

    def on_status(self, msg: GoalStatusArray):
        # 第一次：锁定一个 active goal 并开始计时
        if self.goal_id is None:
            for st in msg.status_list:
                if st.status in ACTIVE:
                    self.goal_id = goal_uuid_hex(st.goal_info.goal_id)
                    self.start_ns = ns_now(self)
                    self.get_logger().info(f"捕捉到 goal_id={self.goal_id}，开始计时…")
                    return

        # 后续：只跟踪这个 goal，直到终态
        if self.goal_id is not None and self.result is None:
            for st in msg.status_list:
                gid = goal_uuid_hex(st.goal_info.goal_id)
                if gid != self.goal_id:
                    continue
                if st.status in TERMINAL:
                    self.end_ns = ns_now(self)
                    if st.status == 4:
                        self.result = "S"
                    elif st.status == 5:
                        self.result = "T"
                    else:
                        self.result = "F"
                    self.write_row_and_exit()
                    return

    def write_row_and_exit(self):
        all_lines = read_lines(self.btlog) if os.path.exists(self.btlog) else []
        bt_end_line = len(all_lines)
        chunk = all_lines[self.bt_start_line:bt_end_line]

        recovery, types, hits = count_recovery_hits(chunk, self.keywords)

        time_sec = ""
        if self.start_ns and self.end_ns and self.end_ns > self.start_ns:
            time_sec = f"{(self.end_ns - self.start_ns)/1e9:.3f}"

        out = self.args.out
        os.makedirs(os.path.dirname(out), exist_ok=True)
        file_exists = os.path.exists(out)

        run_id = next_run_id(out)

        fieldnames = [
            "run","goal_id","result","time_sec","notes",
            "recovery","recovery_types","recovery_hits",
            "bt_start_line","bt_end_line"
        ]

        with open(out, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                w.writeheader()
            w.writerow({
                "run": run_id,
                "goal_id": self.goal_id or "",
                "result": self.result or "",
                "time_sec": time_sec,
                "notes": self.args.note or "",
                "recovery": recovery,
                "recovery_types": types,
                "recovery_hits": str(hits),
                "bt_start_line": str(self.bt_start_line),
                "bt_end_line": str(bt_end_line),
            })

        self.get_logger().info(
            f"[logged] run={run_id} result={self.result} time_sec={time_sec} "
            f"recovery={recovery} types={types if types else '-'} hits={hits}"
        )
        self.get_logger().info(f"Output: {out}")
        self.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--btlog", default="/tmp/w3d3_bt.log")
    ap.add_argument("--keywords", default="configs/recovery_keywords.txt")
    ap.add_argument("--out", default="results/week3_day3_eval_v2.csv")
    ap.add_argument("--note", default="")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--status_topic", default="/navigate_to_pose/_action/status")
    args = ap.parse_args()

    rclpy.init()
    node = EvalV2(args)

    t0 = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.result is not None:
            break
        if time.time() - t0 > args.timeout:
            node.get_logger().error("超时：没捕捉到 goal 或没等到终态。确保：先启动脚本，再点 Nav2 Goal。")
            break
    rclpy.shutdown()

if __name__ == "__main__":
    main()
