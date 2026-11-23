#!/usr/bin/env python3
import csv
import json
import math
import time
import urllib.request
from pathlib import Path

BASE_URL = "http://127.0.0.1:9001"
BENCH_DIR = Path.home() / "sim_benchmarks"

SCENARIOS = [
    {
        "name": "1. 3m 静止",
        "distance_m": 3.0,
        "move_speed": 0.0,
        "move_span": 2.5,
        "spin_omega": 0.0,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "2. 5m 横移",
        "distance_m": 5.0,
        "move_speed": 1.8,
        "move_span": 3.5,
        "spin_omega": 0.0,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "3. 5m 小陀螺",
        "distance_m": 5.0,
        "move_speed": 0.0,
        "move_span": 3.5,
        "spin_omega": 7.5,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "4. 5m 横移+小陀螺",
        "distance_m": 5.0,
        "move_speed": 1.5,
        "move_span": 3.5,
        "spin_omega": 7.5,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "5. 7m 移动+陀螺",
        "distance_m": 7.0,
        "move_speed": 1.8,
        "move_span": 5.0,
        "spin_omega": 7.5,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "6. 5m 双动+陀螺",
        "distance_m": 5.0,
        "move_speed": 1.5,
        "move_span": 3.5,
        "spin_omega": 7.5,
        "self_spin": False,
        "self_move_speed": 1.5,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
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
    total = 0
    valid = 0
    fire = 0
    yaw_errs = []
    pitch_errs = []
    fire_yaw_errs = []
    fire_pitch_errs = []
    if not csv_path.exists():
        return {}
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_s = float(row["t_s"])
            if t_s < 0.5:
                continue
            total += 1
            v = int(row["vis_valid"]) == 1
            fr = int(row["vis_fire"]) == 1
            ey = float(row["err_yaw_deg"])
            ep = float(row["err_pitch_deg"])
            if v:
                valid += 1
                yaw_errs.append(ey)
                pitch_errs.append(ep)
                if fr:
                    fire += 1
                    fire_yaw_errs.append(ey)
                    fire_pitch_errs.append(ep)

    def stats(arr):
        if not arr:
            return (0.0, 0.0, 0.0)
        mean = sum(arr) / len(arr)
        mae = sum(abs(x) for x in arr) / len(arr)
        mx = max(abs(x) for x in arr)
        return (mean, mae, mx)

    y_mean, y_mae, y_max = stats(yaw_errs)
    p_mean, p_mae, p_max = stats(pitch_errs)
    fy_mean, fy_mae, _ = stats(fire_yaw_errs)
    fp_mean, fp_mae, _ = stats(fire_pitch_errs)
    return {
        "valid_pct": (valid / total * 100.0) if total else 0.0,
        "fire_pct": (fire / total * 100.0) if total else 0.0,
        "yaw_bias": y_mean,
        "yaw_mae": y_mae,
        "yaw_max": y_max,
        "pitch_bias": p_mean,
        "pitch_mae": p_mae,
        "pitch_max": p_max,
        "fire_yaw_mae": fy_mae,
        "fire_pitch_mae": fp_mae,
    }


def main():
    results = []
    for sc in SCENARIOS:
        name = sc["name"]
        payload = {k: v for k, v in sc.items() if k != "name"}
        st0 = http_get("/api/state")
        prev_ids = {b["id"] for b in st0.get("bench_history", [])}

        print(f"Running {name} ({sc['duration_s']}s)...", flush=True)
        http_post("/api/arena", payload)
        time.sleep(sc["duration_s"] + 1.2)

        # Wait up to 3s for finalize
        summary = None
        for _ in range(15):
            st = http_get("/api/state")
            for b in st.get("bench_history", []):
                if b["id"] not in prev_ids:
                    summary = b
                    break
            if summary is not None:
                break
            time.sleep(0.2)

        if summary is None:
            print(f"  [WARN] no summary for {name}", flush=True)
            continue

        extra = analyze_csv(BENCH_DIR / summary["csv_file"])
        summary["name"] = name
        summary["extra"] = extra
        results.append(summary)
        print(
            f"  -> 命中率 {summary['hit_rate_pct']:.1f}% ({summary['hit_count']}/{summary['launch_count']}) "
            f"| 锁定 {extra.get('valid_pct',0):.1f}% | 开火 {extra.get('fire_pct',0):.1f}% "
            f"| |ΔY|={extra.get('yaw_mae',0):.2f}° (bias {extra.get('yaw_bias',0):+.2f}°) "
            f"| |ΔP|={extra.get('pitch_mae',0):.2f}° (bias {extra.get('pitch_bias',0):+.2f}°)",
            flush=True,
        )

    out_path = BENCH_DIR / "latest_suite_report.json"
    out_path.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nSaved report to {out_path}")


if __name__ == "__main__":
    main()
