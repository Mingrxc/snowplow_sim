#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
terrain_experiment.py — Python 地形代理/灵敏度实验脚本

本脚本不控制 Gazebo，也不读取 Gazebo 接触/里程计数据。它只使用
control_prototype.py 的 PMSM 轮端模型和代理负载参数，快速观察不同地形
负载假设下的速度跟踪趋势。正式 Gazebo 地形数据请使用：

    terrain_spawn.py + gazebo_speed_test.py

用法:
    python3 terrain_experiment.py
    python3 terrain_experiment.py --include-extended
    python3 terrain_experiment.py --output-dir results/proxy_terrain/my_run

依赖:
    pip install numpy matplotlib pyyaml
"""

import os
import sys
import csv
import time
import argparse
import math
from datetime import datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from world_generator import TERRAIN_PRESETS

matplotlib.rcParams['axes.unicode_minus'] = False


# ================================================================
# 地形参数
# ================================================================

PROXY_LOAD_FACTORS = {
    "asphalt": 1.0,
    "concrete": 1.05,
    "grass": 1.3,
    "sand": 1.8,
    "snow": 2.5,
    "ice": 4.0,
}

TERRAIN_LABELS = {
    "asphalt": ("沥青", "Asphalt"),
    "concrete": ("水泥", "Concrete"),
    "grass": ("草地", "Grass"),
    "sand": ("沙地", "Sand"),
    "snow": ("冰雪", "Snow/Ice"),
    "ice": ("冰面", "Ice"),
}


def build_proxy_terrain_params() -> dict:
    params = {}
    for terrain, load_factor in PROXY_LOAD_FACTORS.items():
        preset = TERRAIN_PRESETS[terrain]
        label, plot_label = TERRAIN_LABELS[terrain]
        params[terrain] = {
            "mu": preset["mu"],
            "mu2": preset["mu2"],
            "load_factor": load_factor,
            "color": list(preset["color"][:3]),
            "label": label,
            "plot_label": plot_label,
        }
    return params


TERRAIN_PARAMS = build_proxy_terrain_params()

PROJECT_TERRAINS = ("concrete", "sand", "snow")
EXTENDED_TERRAINS = tuple(
    terrain for terrain in TERRAIN_PARAMS if terrain not in PROJECT_TERRAINS
)


# ================================================================
# 模拟模式：用 PMSM 模型仿真不同地形
# ================================================================

def simulate_terrain(terrain_name, terrain_params, enable_ff=True,
                     v_cmd=0.5, wheel_radius=0.15, dt=0.001):
    """在给定地形参数下跑一段轨迹并返回数据"""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from control_prototype import PMSM, CascadeController, rk4_step

    load_factor = terrain_params["load_factor"]

    # 摩托车参数：轮端模型
    motor_params = {"J_motor": 0.002, "B_motor": 0.001, "gear_ratio": 50.0,
                    "chassis_mass": 80.0, "wheel_radius": wheel_radius}
    ctrl_params = {"control": {
        "velocity_p": 0.10, "velocity_i": 1.5,
        "current_p": 0.5, "current_i": 10.0,
        "feedforward_j_hat": 3.4, "feedforward_b_hat": 1.25,
        "feedforward_tc": 0.25 * load_factor, "feedforward_lo": 50.0,
    }}

    motor = PMSM(motor_params)
    controller = CascadeController(motor, ctrl_params, dt=dt, enable_ff=enable_ff)

    # 轨迹设计：前进 -> 匀速 -> 停止
    T_total = 4.0
    N = int(T_total / dt)
    t = np.linspace(0, T_total, N)

    omega_ref = np.zeros(N)
    # 0-0.5s: 原地不动
    # 0.5-2.0s: 加速到目标速度
    mask_accel = (t >= 0.5) & (t < 2.0)
    omega_ref[mask_accel] = np.interp(t[mask_accel], [0.5, 2.0],
                                       [0.0, v_cmd / wheel_radius])
    # 2.0-3.0s: 匀速
    mask_cruise = (t >= 2.0) & (t < 3.0)
    omega_ref[mask_cruise] = v_cmd / wheel_radius
    # 3.0-4.0s: 减速停止
    mask_decel = (t >= 3.0) & (t < 4.0)
    omega_ref[mask_decel] = np.interp(t[mask_decel], [3.0, 4.0],
                                       [v_cmd / wheel_radius, 0.0])

    # 负载：与 load_factor 成正比的地面阻力
    T_L = np.ones(N) * 1.0 * load_factor
    # 上坡模拟：中间段额外负载
    T_L[(t > 2.5) & (t < 3.5)] += 2.0 * load_factor

    # 仿真
    i_d = np.zeros(N)
    i_q = np.zeros(N)
    omega = np.zeros(N)
    controller.reset()

    for k in range(N - 1):
        i_d_k = i_d[k]
        i_q_k = i_q[k]
        omega_k = omega[k]

        u_d, u_q_val = controller.step(omega_ref[k], omega_k, i_q_k, i_d_k)

        state = np.array([i_d_k, i_q_k, omega_k])
        def f(ts, y): return motor.ode_rhs(ts, y, u_d, u_q_val, T_L[k])
        new_state = rk4_step(f, t[k], state, dt)
        i_d[k + 1] = new_state[0]
        i_q[k + 1] = new_state[1]
        omega[k + 1] = new_state[2]

    return t, omega_ref, omega, T_L


# ================================================================
# 指标计算
# ================================================================

def compute_terrain_metrics(t, omega_ref, omega, wheel_radius=0.15):
    """从仿真数据提取对比指标"""
    v_ref = omega_ref * wheel_radius
    v_act = omega * wheel_radius
    error = v_ref - v_act

    # 仅评估运动段 (t > 0.5s, v_ref > 0.01 m/s)
    mask = t > 0.5
    t_eval = t[mask]
    error_eval = error[mask]
    v_ref_eval = v_ref[mask]
    v_act_eval = v_act[mask]

    rmse = float(np.sqrt(np.mean(error_eval ** 2)))
    max_error = float(np.max(np.abs(error_eval)))

    # 最大有效速度（实际达到的）
    v_achieved = float(np.max(v_act_eval))
    v_commanded = float(np.max(v_ref_eval)) if np.max(v_ref_eval) > 0.01 else 1.0
    speed_ratio = v_achieved / v_commanded  # 速度达成率

    # 滑移率近似（在匀速段）
    cruise = (t_eval > 1.5)  # t=2.0-3.0 in absolute
    if np.any(cruise) and np.any(v_ref_eval[cruise] > 1e-3):
        slip = float(np.mean(
            np.abs(v_ref_eval[cruise] - v_act_eval[cruise]) /
            np.maximum(v_ref_eval[cruise], 1e-3)))
    else:
        slip = 0.0

    # 稳态误差
    n_steady = max(1, len(error_eval) // 4)
    steady_error = float(np.mean(np.abs(error_eval[-n_steady:])))

    return {
        "rmse": rmse,
        "max_error": max_error,
        "speed_ratio": speed_ratio,
        "slip": slip,
        "steady_error": steady_error,
        "v_achieved": v_achieved,
        "v_commanded": v_commanded,
    }


# ================================================================
# 实验结果汇总表
# ================================================================

def run_all_terrains(enable_ff=True, v_cmd=0.5, terrain_names=None):
    """对每种地形跑一遍仿真"""
    results = {}
    all_traces = {}

    terrain_names = terrain_names or PROJECT_TERRAINS
    for tname in terrain_names:
        tparams = TERRAIN_PARAMS[tname]
        t, ref, omega, TL = simulate_terrain(tname, tparams, enable_ff=enable_ff, v_cmd=v_cmd)
        metrics = compute_terrain_metrics(t, ref, omega)
        results[tname] = metrics
        all_traces[tname] = (t, ref, omega, TL)

    return results, all_traces


def print_results(results, ff_on=True, terrain_names=None):
    """打印结果表格"""
    terrain_names = terrain_names or list(results)
    header = f"{'地形':8s} {'RMSE(m/s)':>10s} {'速度达成%':>9s} {'滑移%':>7s} {'最高速度(m/s)':>12s} {'稳态误差':>8s}"
    print("\n" + "=" * 70)
    print(f"  地形对比实验 (FF={'ON' if ff_on else 'OFF'}, v_cmd=0.5 m/s)")
    print("=" * 70)
    print(header)
    print("-" * 70)
    for tname in terrain_names:
        r = results.get(tname, {})
        print(f"{TERRAIN_PARAMS[tname]['label']:8s} {r.get('rmse',0):10.4f} {r.get('speed_ratio',0)*100:9.1f} {r.get('slip',0)*100:7.1f} {r.get('v_achieved',0):12.3f} {r.get('steady_error',0):8.4f}")
    print("-" * 70)

    # 总结
    best = min(results, key=lambda t: results[t]["rmse"])
    worst = max(results, key=lambda t: results[t]["rmse"])
    print(f"\n  最佳地形: {TERRAIN_PARAMS[best]['label']} (RMSE={results[best]['rmse']:.4f})")
    print(f"  最差地形: {TERRAIN_PARAMS[worst]['label']} (RMSE={results[worst]['rmse']:.4f})")
    if len(terrain_names) >= 2:
        corr = np.corrcoef(
            [TERRAIN_PARAMS[t]['mu'] for t in terrain_names],
            [results[t]['rmse'] for t in terrain_names],
        )[0, 1]
        print(f"  摩擦系数与RMSE相关系数: {corr:.3f}")

    return results


# ================================================================
# 绘图
# ================================================================

def plot_terrain_comparison(ff_data, noff_data, save_dir=None, terrain_names=None):
    """绘制所有地形的速度响应对比图"""
    wheel_radius = 0.15
    terrain_names = terrain_names or list(ff_data or noff_data or TERRAIN_PARAMS)
    cols = min(3, max(1, len(terrain_names)))
    rows = int(math.ceil(len(terrain_names) / cols))

    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 4.5 * rows))
    axes = np.atleast_1d(axes).flatten()

    for idx, tname in enumerate(terrain_names):
        ax = axes[idx]
        tparams = TERRAIN_PARAMS[tname]

        if ff_data and tname in ff_data:
            t_ff, ref_ff, omega_ff, _ = ff_data[tname]
            ax.plot(t_ff, omega_ff * wheel_radius, 'b-', linewidth=1.5, label='FF ON')
        if noff_data and tname in noff_data:
            t_nff, ref_nff, omega_nff, _ = noff_data[tname]
            ax.plot(t_nff, ref_nff * wheel_radius, 'k:', linewidth=1.5, label='Cmd')
            ax.plot(t_nff, omega_nff * wheel_radius, 'r--', linewidth=1.5, label='FF OFF')

        ax.set_title(f"{tparams['plot_label']} (mu={tparams['mu']})", fontsize=11)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.1, 0.8)

    for ax in axes[len(terrain_names):]:
        ax.axis('off')

    fig.suptitle('Terrain Comparison: Velocity Tracking', fontsize=14, y=1.01)
    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        fp = os.path.join(save_dir, 'terrain_comparison.png')
        plt.savefig(fp, dpi=150)
        print(f"Saved: {fp}")
    plt.close()


def plot_bar_chart(ff_results, noff_results, save_dir=None, terrain_names=None):
    """绘制柱状图：各地形 RMSE 和 speed_ratio 对比"""
    terrains = terrain_names or list(ff_results)
    labels = [TERRAIN_PARAMS[t]['plot_label'] for t in terrains]

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # RMSE
    ax = axes[0]
    x = np.arange(len(terrains))
    w = 0.35
    rmse_ff = [ff_results[t]['rmse'] for t in terrains]
    rmse_nff = [noff_results[t]['rmse'] for t in terrains]
    ax.bar(x - w/2, rmse_ff, w, label='FF ON', color='#2ecc71', edgecolor='black')
    ax.bar(x + w/2, rmse_nff, w, label='FF OFF', color='#e74c3c', edgecolor='black')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel('RMSE (m/s)')
    ax.set_title('Velocity Tracking RMSE by Terrain')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # Speed ratio
    ax = axes[1]
    sr_ff = [ff_results[t]['speed_ratio'] * 100 for t in terrains]
    sr_nff = [noff_results[t]['speed_ratio'] * 100 for t in terrains]
    ax.bar(x - w/2, sr_ff, w, label='FF ON', color='#2ecc71', edgecolor='black')
    ax.bar(x + w/2, sr_nff, w, label='FF OFF', color='#e74c3c', edgecolor='black')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel('Speed Achievement (%)')
    ax.set_title('Speed Achievement by Terrain')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    ax.axhline(y=100, color='gray', linestyle=':', alpha=0.5)

    fig.suptitle('Terrain Experiment Results', fontsize=14)
    plt.tight_layout()
    if save_dir:
        fp = os.path.join(save_dir, 'terrain_barchart.png')
        plt.savefig(fp, dpi=150)
        print(f"Saved: {fp}")
    plt.close()


def save_csv(results_ff, results_noff, save_dir, terrain_names=None):
    """保存 CSV"""
    terrain_names = terrain_names or list(results_ff)
    fp = os.path.join(save_dir, 'terrain_results.csv')
    columns = ["terrain", "label", "mu", "load_factor",
               "rmse_ff", "max_error_ff", "speed_ratio_ff", "slip_ff", "steady_error_ff",
               "rmse_noff", "max_error_noff", "speed_ratio_noff", "slip_noff", "steady_error_noff",
               "rmse_improvement_pct"]
    with open(fp, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for tname in terrain_names:
            tp = TERRAIN_PARAMS[tname]
            rf = results_ff[tname]
            rn = results_noff[tname]
            imp = (rn['rmse'] - rf['rmse']) / max(rn['rmse'], 1e-9) * 100
            writer.writerow({
                "terrain": tname, "label": tp['label'], "mu": tp['mu'],
                "load_factor": tp['load_factor'],
                "rmse_ff": round(rf['rmse'], 6), "max_error_ff": round(rf['max_error'], 6),
                "speed_ratio_ff": round(rf['speed_ratio'], 4), "slip_ff": round(rf['slip'], 4),
                "steady_error_ff": round(rf['steady_error'], 6),
                "rmse_noff": round(rn['rmse'], 6), "max_error_noff": round(rn['max_error'], 6),
                "speed_ratio_noff": round(rn['speed_ratio'], 4), "slip_noff": round(rn['slip'], 4),
                "steady_error_noff": round(rn['steady_error'], 6),
                "rmse_improvement_pct": round(imp, 1),
            })
    print(f"CSV: {fp}")


# ================================================================
# 主入口
# ================================================================

def main():
    parser = argparse.ArgumentParser(description='Python 地形代理/灵敏度实验')
    parser.add_argument('--gazebo', action='store_true',
                        help='只打印 Gazebo 正式实验入口提示，不自动控制 Gazebo')
    parser.add_argument('--include-extended', action='store_true',
                        help='额外运行 asphalt/grass/ice 扩展对照；默认只跑项目书三地形')
    parser.add_argument('--no-save', action='store_true', help='只打印结果，不保存图表和 CSV')
    parser.add_argument('--output-dir', default=None,
                        help='输出目录，默认: ./results/proxy_terrain/<time>')
    args = parser.parse_args()

    terrain_names = list(PROJECT_TERRAINS)
    if args.include_extended:
        terrain_names.extend(EXTENDED_TERRAINS)

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = args.output_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'results', 'proxy_terrain', run_id,
    )
    if not args.no_save:
        os.makedirs(save_dir, exist_ok=True)

    if args.gazebo:
        print("Gazebo 正式实验：此入口只提供提示，不会自动控制 Gazebo。")
        print("请先启动 Gazebo，再按地形名称重新 spawn，然后运行固定时长测速脚本。")
        print("\n示例:")
        print("  python3 terrain_spawn.py concrete")
        print("  python3 gazebo_speed_test.py --label concrete --speed 0.20 "
              "--settle 2 --duration 5 "
              "--run-dir results/gazebo_terrain/concrete_020")
        print("\n把 concrete 替换为 sand/snow，其他参数保持一致。")
        print("不要使用单次非零 `ros2 topic pub ... -1`，结束时必须发零速度。")
        return

    print("=" * 70)
    print("  扫雪车地形对比实验 — Python 代理模式")
    print("  默认只运行项目书三地形；结果不能替代 Gazebo 实测")
    print("=" * 70)

    # Run FF ON
    print("\n[1/2] Running with FeedForward ON ...")
    results_ff, traces_ff = run_all_terrains(
        enable_ff=True, terrain_names=terrain_names
    )
    print_results(results_ff, ff_on=True, terrain_names=terrain_names)

    # Run FF OFF
    print("\n[2/2] Running with FeedForward OFF ...")
    results_noff, traces_noff = run_all_terrains(
        enable_ff=False, terrain_names=terrain_names
    )
    print_results(results_noff, ff_on=False, terrain_names=terrain_names)

    # Comparison summary
    print("\n" + "=" * 70)
    print("  FF 改善汇总")
    print("=" * 70)
    for tname in terrain_names:
        imp = (results_noff[tname]['rmse'] - results_ff[tname]['rmse']) / max(results_noff[tname]['rmse'], 1e-9) * 100
        print(f"  {TERRAIN_PARAMS[tname]['label']:6s}: RMSE {results_noff[tname]['rmse']:.4f} -> {results_ff[tname]['rmse']:.4f} ({imp:+.1f}%)  "
              f"Speed {results_noff[tname]['speed_ratio']*100:.0f}% -> {results_ff[tname]['speed_ratio']*100:.0f}%")

    # Plots
    if not args.no_save:
        plot_terrain_comparison(
            traces_ff, traces_noff, save_dir, terrain_names=terrain_names
        )
        plot_bar_chart(
            results_ff, results_noff, save_dir, terrain_names=terrain_names
        )
        save_csv(results_ff, results_noff, save_dir, terrain_names=terrain_names)

    if args.no_save:
        print("\n[OK] 实验完成。未保存输出。")
    else:
        print("\n[OK] 实验完成。输出目录:", save_dir)


if __name__ == "__main__":
    main()
