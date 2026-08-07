#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
benchmark_runner.py — 控制算法自动对比实验框架 (updated for wheel-level PMSM)

网格扫描: 3速度PI × 2前馈 × 3负载 = 18组
每组跑 Step / 正弦 / 轨迹 三个子实验
自动收集 RMSE / 超调 / 上升时间 / 最大偏差
导出 CSV + 汇总热力图

用法:
    python3 benchmark_runner.py              # 全18组
    python3 benchmark_runner.py --quick      # 快速6组
    python3 benchmark_runner.py --output-dir results/benchmark_run
"""

import os, sys, csv, time, argparse, itertools
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from control_prototype import (PMSM, run_motor_simulation)

# Grid definitions
VELOCITY_PI_GRID = [
    {"label": "Mild-PI",     "velocity_p": 0.08, "velocity_i": 0.8},
    {"label": "Std-PI",      "velocity_p": 0.15, "velocity_i": 1.5},
    {"label": "Aggressive",  "velocity_p": 0.30, "velocity_i": 3.0},
]
FEEDFORWARD_GRID = [
    {"label": "FF-Full",    "enable_ff": True,  "ff_j_hat": 6.8, "ff_b_hat": 2.5, "ff_tc": 0.5, "ff_lo": 50.0},
    {"label": "FF-Off",     "enable_ff": False, "ff_j_hat": 0.0, "ff_b_hat": 0.0, "ff_tc": 0.0, "ff_lo": 0.0},
]
LOAD_DISTURBANCE_GRID = [
    {"label": "Light",  "load_1": 1.0,  "load_2": 2.0},
    {"label": "Medium", "load_1": 2.0,  "load_2": 4.0},
    {"label": "Heavy",  "load_1": 4.0,  "load_2": 6.0},
]

def compute_metrics(t, ref, actual, wheel_radius=0.15):
    v_ref = ref * wheel_radius; v_act = actual * wheel_radius
    error = v_ref - v_act
    rmse = float(np.sqrt(np.mean(error**2)))
    max_error = float(np.max(np.abs(error)))
    steady_val = np.max(v_ref)
    if steady_val > 0.01:
        t10 = t[np.argmax(v_act > 0.1*steady_val)] if np.any(v_act > 0.1*steady_val) else t[0]
        t90 = t[np.argmax(v_act > 0.9*steady_val)] if np.any(v_act > 0.9*steady_val) else t[-1]
        rise_time = float(t90 - t10)
    else:
        rise_time = 0.0
    peak = np.max(v_act)
    overshoot_pct = float(max(0, (peak-steady_val)/steady_val*100)) if steady_val>0.01 else 0.0
    n_steady = max(1, len(error)//5)
    steady_error = float(np.mean(np.abs(error[-n_steady:])))
    return {"rmse": rmse, "max_error": max_error, "rise_time": rise_time,
            "overshoot_pct": overshoot_pct, "steady_error": steady_error}

def compute_sine_metrics(t, ref, actual, wheel_radius=0.15):
    v_ref = ref * wheel_radius; v_act = actual * wheel_radius
    error = v_ref - v_act
    rmse = float(np.sqrt(np.mean(error**2)))
    max_error = float(np.max(np.abs(error)))
    eps = 1e-9
    amp_ref = np.max(np.abs(v_ref[100:])); amp_act = np.max(np.abs(v_act[100:]))
    amp_decay_pct = float((1.0 - amp_act/max(amp_ref,eps))*100)
    return {"rmse": rmse, "max_error": max_error, "amp_decay_pct": amp_decay_pct}

def run_single_benchmark(pi_entry, ff_entry, load_entry):
    pi_params = {k: v for k, v in pi_entry.items() if k != "label"}
    ff_params = {k: v for k, v in ff_entry.items() if k != "label"}
    load_params = {k: v for k, v in load_entry.items() if k != "label"}
    
    ctrl_params = {"control": {
        **pi_params,
        "current_p": 0.5, "current_i": 10.0,
        "feedforward_j_hat": ff_params.get("ff_j_hat", 6.8),
        "feedforward_b_hat": ff_params.get("ff_b_hat", 2.5),
        "feedforward_tc": ff_params.get("ff_tc", 0.5),
        "feedforward_lo": ff_params.get("ff_lo", 50.0),
    }}
    
    enable_ff = ff_params.get("enable_ff", True)
    wheel_radius = 0.15; dt = 0.001
    results = {}

    # Step
    T = 2.0
    def step_ref(t): return (0.5/wheel_radius) if t > 0.2 else 0.0
    def step_load(t): return load_params["load_1"] if t > 1.0 else 0.0
    t, ref, omega, iq, uq, TL, Te = run_motor_simulation(dt, T, step_ref, step_load, enable_ff=enable_ff, ctrl_params=ctrl_params)
    results["step"] = compute_metrics(t, ref, omega, wheel_radius)

    # Sine
    T = 5.0; freq = 0.5; amp = 0.4/wheel_radius
    def sine_ref(t): return amp * np.sin(2*np.pi*freq*t)
    def sine_load(t): return 0.0
    t, ref, omega, iq, uq, TL, Te = run_motor_simulation(dt, T, sine_ref, sine_load, enable_ff=enable_ff, ctrl_params=ctrl_params)
    results["sine"] = compute_sine_metrics(t, ref, omega, wheel_radius)

    # Trajectory
    T = 10.0
    def traj_ref(t):
        if 0.5 <= t < 3.0: return 0.3/wheel_radius
        elif 3.0 <= t < 5.0: return np.interp(t, [3.0,5.0], [0.3,0.6])/wheel_radius
        elif 5.0 <= t < 8.0: return 0.6/wheel_radius
        elif 8.0 <= t < 9.5: return np.interp(t, [8.0,9.5], [0.6,0.0])/wheel_radius
        else: return 0.0
    def traj_load(t):
        if 2.0 < t < 2.5: return load_params["load_1"]
        elif 6.0 < t < 7.0: return load_params["load_2"]
        return 0.0
    t, ref, omega, iq, uq, TL, Te = run_motor_simulation(dt, T, traj_ref, traj_load, enable_ff=enable_ff, ctrl_params=ctrl_params)
    results["track"] = compute_metrics(t, ref, omega, wheel_radius)

    return results

CSV_COLUMNS = [
    "exp_id", "pi_label", "ff_label", "load_label",
    "step_rmse", "step_max_error", "step_rise_time", "step_overshoot_pct", "step_steady_error",
    "sine_rmse", "sine_max_error", "sine_amp_decay_pct",
    "track_rmse", "track_max_error", "track_rise_time", "track_overshoot_pct", "track_steady_error",
    "score",
]

def compute_score(row):
    return (row["step_rmse"]*10 + row["step_overshoot_pct"]*0.5 +
            row["sine_rmse"]*8 + row["sine_amp_decay_pct"]*0.3 +
            row["track_rmse"]*10 + row["track_overshoot_pct"]*0.3)

def save_csv(rows, filepath):
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for row in rows: writer.writerow(row)
    print(f"\nCSV saved: {filepath} ({len(rows)} rows)")

def plot_heatmap(rows, save_dir):
    pi_labels = sorted(set(r["pi_label"] for r in rows))
    ff_labels = sorted(set(r["ff_label"] for r in rows))
    load_labels = sorted(set(r["load_label"] for r in rows))
    n_rows = len(pi_labels)*len(ff_labels); n_cols = len(load_labels)
    matrix = np.zeros((n_rows, n_cols)); row_labels = []
    for i, (pi, ff) in enumerate(itertools.product(pi_labels, ff_labels)):
        row_labels.append(f"{pi}\n{ff}")
        for j, ld in enumerate(load_labels):
            m = [r for r in rows if r["pi_label"]==pi and r["ff_label"]==ff and r["load_label"]==ld]
            if m: matrix[i,j] = m[0]["score"]
    fig, ax = plt.subplots(figsize=(10, 6))
    im = ax.imshow(matrix, cmap='RdYlGn_r', aspect='auto', vmin=0, vmax=max(matrix.max(), 0.01))
    ax.set_xticks(range(n_cols)); ax.set_xticklabels(load_labels, fontsize=10)
    ax.set_yticks(range(n_rows)); ax.set_yticklabels(row_labels, fontsize=8)
    for i in range(n_rows):
        for j in range(n_cols):
            ax.text(j, i, f'{matrix[i,j]:.2f}', ha='center', va='center', fontsize=7,
                    color='black' if matrix[i,j] < matrix.max()*0.6 else 'white')
    ax.set_title("Benchmark Score (lower=better)", fontsize=12)
    plt.colorbar(im, ax=ax, label='Score'); plt.tight_layout()
    fp = os.path.join(save_dir, 'benchmark_heatmap.png'); plt.savefig(fp, dpi=150); plt.close()
    print(f"Heatmap: {fp}")

def plot_pareto(rows, save_dir):
    fig, ax = plt.subplots(figsize=(8, 6))
    colors = {'FF-Full': '#2ecc71', 'FF-Off': '#e74c3c'}
    markers = {'Light': 'o', 'Medium': 's', 'Heavy': '^'}
    for r in rows:
        ax.scatter(r["step_rmse"], r["step_overshoot_pct"],
                   c=colors.get(r["ff_label"], '#999'), marker=markers.get(r["load_label"], 'x'),
                   s=80, edgecolors='black', linewidth=0.5, alpha=0.7)
    from matplotlib.lines import Line2D
    leg_ff = [Line2D([0],[0], marker='o', color='w', markerfacecolor=c, markersize=8, label=l) for l,c in colors.items()]
    leg_ld = [Line2D([0],[0], marker=m, color='w', markerfacecolor='#666', markersize=8, label=l) for l,m in markers.items()]
    ax.legend(handles=leg_ff+leg_ld, loc='upper right', fontsize=8)
    ax.set_xlabel("Step RMSE (m/s)"); ax.set_ylabel("Overshoot (%)")
    ax.set_title("Pareto: RMSE vs Overshoot"); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fp = os.path.join(save_dir, 'benchmark_pareto.png'); plt.savefig(fp, dpi=150); plt.close()
    print(f"Pareto: {fp}")

def main():
    parser = argparse.ArgumentParser(description='控制算法对比实验')
    parser.add_argument('--quick', action='store_true')
    parser.add_argument('--output-dir', default=None,
                        help='输出目录，默认: ./benchmark_output')
    args = parser.parse_args()
    base_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = args.output_dir or os.path.join(base_dir, 'benchmark_output')
    os.makedirs(output_dir, exist_ok=True)

    if args.quick:
        pi_grid = [VELOCITY_PI_GRID[1]]
        ff_grid = FEEDFORWARD_GRID
        load_grid = LOAD_DISTURBANCE_GRID
        total = len(pi_grid)*len(ff_grid)*len(load_grid)
    else:
        pi_grid = VELOCITY_PI_GRID; ff_grid = FEEDFORWARD_GRID; load_grid = LOAD_DISTURBANCE_GRID
        total = len(pi_grid)*len(ff_grid)*len(load_grid)

    print("="*70)
    print(f"  Benchmark: {len(pi_grid)}PI x {len(ff_grid)}FF x {len(load_grid)}Load = {total} exps")
    print("="*70)

    rows = []; exp_id = 0
    for pi_entry, ff_entry, load_entry in itertools.product(pi_grid, ff_grid, load_grid):
        exp_id += 1
        t0 = time.time()
        print(f"  [{exp_id:2d}/{total}] PI={pi_entry['label']:12s} FF={ff_entry['label']:6s} Load={load_entry['label']:4s} ...", end=" ", flush=True)
        results = run_single_benchmark(pi_entry, ff_entry, load_entry)
        elapsed = time.time() - t0
        print(f"done ({elapsed:.1f}s)")
        row = {"exp_id": exp_id, "pi_label": pi_entry["label"], "ff_label": ff_entry["label"], "load_label": load_entry["label"],
               **{f"step_{k}": v for k, v in results["step"].items()},
               **{f"sine_{k}": v for k, v in results["sine"].items()},
               **{f"track_{k}": v for k, v in results["track"].items()}}
        row["score"] = compute_score(row)
        rows.append(row)

    csv_path = os.path.join(output_dir, 'benchmark_results.csv')
    save_csv(rows, csv_path)
    plot_heatmap(rows, output_dir)
    plot_pareto(rows, output_dir)

    best = min(rows, key=lambda r: r["score"])
    print("\n" + "="*70)
    print("  BEST: " + f"PI={best['pi_label']} FF={best['ff_label']} Load={best['load_label']}")
    print(f"  Score={best['score']:.2f}  Step RMSE={best['step_rmse']:.4f}  OS={best['step_overshoot_pct']:.1f}%")
    print("="*70)

if __name__ == "__main__":
    main()
