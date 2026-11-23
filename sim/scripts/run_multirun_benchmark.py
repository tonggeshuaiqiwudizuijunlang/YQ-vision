#!/usr/bin/env python3
"""Multi-run statistical benchmark across all 11 scenarios (mean +/- std)."""
import csv
import json
import math
import statistics
import sys
import time
import urllib.request
from pathlib import Path

BASE_URL = "http://127.0.0.1:9001"
BENCH_DIR = Path.home() / "sim_benchmarks"
REPEATS = int(sys.argv[1]) if len(sys.argv) > 1 else 3

SCENARIOS = [
    {"name": "1. 3m 静止", "distance_m": 3.0, "move_speed": 0.0, "move_span": 2.5, "spin_omega": 0.0, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "2. 5m 横移", "distance_m": 5.0, "move_speed": 1.8, "move_span": 3.5, "spin_omega": 0.0, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "3. 5m 小陀螺", "distance_m": 5.0, "move_speed": 0.0, "move_span": 3.5, "spin_omega": 7.5, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "4. 5m 横移+小陀螺", "distance_m": 5.0, "move_speed": 1.5, "move_span": 3.5, "spin_omega": 7.5, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "5. 7m 移动+陀螺", "distance_m": 7.0, "move_speed": 1.8, "move_span": 5.0, "spin_omega": 7.5, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "6. 5m 双动+陀螺", "distance_m": 5.0, "move_speed": 1.5, "move_span": 3.5, "spin_omega": 7.5, "self_spin": False, "self_move_speed": 1.5, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "H1. 5m 高速陀螺 (ω=12)", "distance_m": 5.0, "move_speed": 0.0, "move_span": 3.5, "spin_omega": 12.0, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "H2. 5m 极速陀螺 (ω=15)", "distance_m": 5.0, "move_speed": 0.0, "move_span": 3.5, "spin_omega": 15.0, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "H3. 7m 快移+快陀螺 (v=2.2,ω=12)", "distance_m": 7.0, "move_speed": 2.2, "move_span": 5.0, "spin_omega": 12.0, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "H4. 5m 己陀螺+双动+敌快陀螺", "distance_m": 5.0, "move_speed": 1.5, "move_span": 3.5, "spin_omega": 10.0, "self_spin": True, "self_move_speed": 1.2, "self_move_span": 3.0, "target_type": "infantry", "duration_s": 10.0},
    {"name": "H5. 7m 英雄快移+陀螺", "distance_m": 7.0, "move_speed": 1.5, "move_span": 5.0, "spin_omega": 10.0, "self_spin": False, "self_move_speed": 0.0, "self_move_span": 3.0, "target_type": "hero", "duration_s": 10.0},
]


def http_get(path):
    with urllib.request.urlopen(f"{BASE_URL}{path}", timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


def http_post(path, payload):
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{BASE_URL}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


def analyze_csv(csv_path: Path):
    total = valid = fire = 0
    yaw_errs, pitch_errs = [], []
    if not csv_path.exists():
        return {"valid_pct": 0.0, "fire_pct": 0.0, "yaw_mae": 0.0, "pitch_mae": 0.0}
    with csv_path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if float(row["t_s"]) < 0.5:
                continue
            total += 1
            if int(row["vis_valid"]) == 1:
                valid += 1
                yaw_errs.append(abs(float(row["err_yaw_deg"])))
                pitch_errs.append(abs(float(row["err_pitch_deg"])))
                if int(row["vis_fire"]) == 1:
                    fire += 1
    return {
        "valid_pct": (valid / total * 100.0) if total else 0.0,
        "fire_pct": (fire / total * 100.0) if total else 0.0,
        "yaw_mae": (sum(yaw_errs) / len(yaw_errs)) if yaw_errs else 0.0,
        "pitch_mae": (sum(pitch_errs) / len(pitch_errs)) if pitch_errs else 0.0,
    }


def run_once(sc):
    payload = {k: v for k, v in sc.items() if k != "name"}
    st0 = http_get("/api/state")
    prev_ids = {b["id"] for b in st0.get("bench_history", [])}
    http_post("/api/arena", payload)
    time.sleep(sc["duration_s"] + 1.2)
    for _ in range(15):
        st = http_get("/api/state")
        for b in st.get("bench_history", []):
            if b["id"] not in prev_ids:
                extra = analyze_csv(BENCH_DIR / b["csv_file"])
                b["extra"] = extra
                return b
        time.sleep(0.2)
    return None


def mean_std(vals):
    m = statistics.mean(vals)
    sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
    return m, sd


def main():
    report = []
    print(f"=== Multi-Run Benchmark ({len(SCENARIOS)} scenarios x {REPEATS} repeats) ===", flush=True)
    for sc in SCENARIOS:
        name = sc["name"]
        runs = []
        retries = 0
        while len(runs) < REPEATS:
            b = run_once(sc)
            if b is None:
                continue
            # Retry once if a scenario-switch transition glitch occurs (< 15 shots)
            if b["launch_count"] < 15 and retries < 2:
                retries += 1
                print(f"  [retry {retries}] {name} transition glitch ({b['hit_count']}/{b['launch_count']})", flush=True)
                time.sleep(0.5)
                continue
            runs.append(b)
        rates = [r["hit_rate_pct"] for r in runs]
        hits = [r["hit_count"] for r in runs]
        launches = [r["launch_count"] for r in runs]
        locks = [r["extra"]["valid_pct"] for r in runs]
        yaws = [r["extra"]["yaw_mae"] for r in runs]
        pitches = [r["extra"]["pitch_mae"] for r in runs]

        r_m, r_s = mean_std(rates)
        h_m, h_s = mean_std(hits)
        l_m, _ = mean_std(launches)
        lk_m, _ = mean_std(locks)
        y_m, _ = mean_std(yaws)
        p_m, _ = mean_std(pitches)

        item = {
            "name": name,
            "repeats": REPEATS,
            "hit_rate_mean": r_m,
            "hit_rate_std": r_s,
            "hits_mean": h_m,
            "hits_std": h_s,
            "launches_mean": l_m,
            "lock_mean": lk_m,
            "yaw_mae_mean": y_m,
            "pitch_mae_mean": p_m,
            "raw_rates": [round(x, 1) for x in rates],
            "raw_hits": hits,
            "raw_launches": launches,
        }
        report.append(item)
        raw_str = ", ".join(f"{r['hit_rate_pct']:.1f}%({r['hit_count']}/{r['launch_count']})" for r in runs)
        print(
            f"{name}: 命中率 {r_m:.1f}%±{r_s:.1f}% | 命中 {h_m:.1f}±{h_s:.1f}/{l_m:.0f} | 锁定 {lk_m:.1f}% | |ΔY|={y_m:.2f}° | [{raw_str}]",
            flush=True,
        )

    std_hits = sum(x["hits_mean"] for x in report[:6])
    hard_hits = sum(x["hits_mean"] for x in report[6:])
    std_rate = statistics.mean(x["hit_rate_mean"] for x in report[:6])
    hard_rate = statistics.mean(x["hit_rate_mean"] for x in report[6:])
    print(f"\nSUMMARY: 标准6场景 均分 {std_rate:.1f}% / 总命中 {std_hits:.1f}发 | 高难5场景 均分 {hard_rate:.1f}% / 总命中 {hard_hits:.1f}发 | 全11场景 总命中 {std_hits+hard_hits:.1f}发 ({(std_hits+hard_hits)/110.0:.2f}中/秒)", flush=True)

    out_path = BENCH_DIR / "latest_multirun_report.json"
    out_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Saved to {out_path}", flush=True)


if __name__ == "__main__":
    main()
