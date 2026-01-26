#!/usr/bin/env python3
import csv, statistics, argparse, math

def to_float(x):
    try:
        v = float(x)
        if math.isfinite(v):
            return v
    except Exception:
        return None
    return None

def load(path):
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def summarize(rows, take=None):
    if take is not None:
        rows = rows[:take]

    results = [r.get("result", "") for r in rows]
    times = []
    for r in rows:
        v = to_float(r.get("time_sec", ""))
        if v is not None:
            times.append(v)

    n = len(results)
    succ = sum(1 for x in results if x == "S")
    fail = sum(1 for x in results if x == "F")
    cancel = sum(1 for x in results if x == "T")

    out = {
        "runs": n,
        "success": succ,
        "fail": fail,
        "cancel": cancel,
        "success_rate": (succ / n) if n else 0.0
    }

    if times:
        out.update({
            "avg_time": sum(times) / len(times),
            "median_time": statistics.median(times),
            "min_time": min(times),
            "max_time": max(times),
        })
    else:
        out.update({"avg_time": 0.0, "median_time": 0.0, "min_time": 0.0, "max_time": 0.0})

    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--take", type=int, default=None, help="只取前 N 条用于对照（防止多跑混入）")
    args = ap.parse_args()

    rows = load(args.inp)
    s = summarize(rows, take=args.take)

    print(f"file: {args.inp}")
    if args.take:
        print(f"take: {args.take}")
    for k in ["runs","success","fail","cancel","success_rate","avg_time","median_time","min_time","max_time"]:
        v = s[k]
        if isinstance(v, float):
            print(f"{k}: {v:.3f}")
        else:
            print(f"{k}: {v}")

if __name__ == "__main__":
    main()
