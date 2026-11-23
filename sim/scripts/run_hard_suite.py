#!/usr/bin/env python3
import csv
import json
import time
import urllib.request
from pathlib import Path

BASE_URL = "http://127.0.0.1:9001"
BENCH_DIR = Path.home() / "sim_benchmarks"

SCENARIOS = [
    {
        "name": "H1. 5m 高速陀螺 (ω=12)",
        "distance_m": 5.0,
        "move_speed": 0.0,
        "move_span": 3.5,
        "spin_omega": 12.0,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "H2. 5m 极速陀螺 (ω=15)",
        "distance_m": 5.0,
        "move_speed": 0.0,
        "move_span": 3.5,
        "spin_omega": 15.0,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "H3. 7m 快移+快陀螺 (v=2.2,ω=12)",
        "distance_m": 7.0,
        "move_speed": 2.2,
        "move_span": 5.0,
        "spin_omega": 12.0,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "H4. 5m 己陀螺+双动+敌快陀螺",
        "distance_m": 5.0,
        "move_speed": 1.5,
        "move_span": 3.5,
        "spin_omega": 10.0,
        "self_spin": True,
        "self_move_speed": 1.2,
        "self_move_span": 3.0,
        "target_type": "infantry",
        "duration_s": 10.0,
    },
    {
        "name": "H5. 7m 英雄快移+陀螺",
        "distance_m": 7.0,
        "move_speed": 1.5,
        "move_span": 5.0,
        "spin_omega": 10.0,
        "self_spin": False,
        "self_move_speed": 0.0,
        "self_move_span": 3.0,
        "target_type": "hero",
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
    total, valid, fire = 0, 0, 0
    yaw_errs, pitch_errs = [], []
    if not csv_path.exists():
        return {}
    with csv_path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if float(row["t_s"]) < 0.5:
                continue
            total += 1
            if int(row["vis_valid"]) == 1:
                valid += 1
                yaw_errs.append(float(row["err_yaw_deg"]))
                pitch_errs.append(float(row["err_pitch_deg"]))
                if int(row["vis_fire"]) == 1:
                    fire += 1
    y_mae = sum(abs(x) for x in yaw_errs) / len(yaw_errs) if yaw_errs else 0.0
    p_mae = sum(abs(x) for x in pitch_errs) / len(pitch_errs) if pitch_errs else 0.0
    return {
        "valid_pct": (valid / total * 100.0) if total else 0.0,
        "fire_pct": (fire / total * 100.0) if total else 0.0,
        "yaw_mae": y_mae,
        "pitch_mae": p_mae,
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
            f"| 锁定 {extra['valid_pct']:.1f}% | 开火 {extra['fire_pct']:.1f}% "
            f"| |ΔY|={extra['yaw_mae']:.2f}° | |ΔP|={extra['pitch_mae']:.2f}°",
            flush=True,
        )

    out_path = BENCH_DIR / "latest_hard_report.json"
    out_path.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nSaved hard report to {out_path}")


if __name__ == "__main__":
    main()
