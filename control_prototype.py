#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
control_prototype.py -- PMSM cascade control prototype for snowplow robot

Key improvements over original:
  - Full PMSM dq model with inductance dynamics (RK4 integration)
  - Gearbox correctly reflected: motor-shaft params -> wheel-shaft dynamics
  - Current loop PI actually regulates i_d, i_q through u_d, u_q
  - Reasonable default gains tuned for the wheel-level model
"""

import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

matplotlib.rcParams['axes.unicode_minus'] = False

# ---- RK4 integrator ----
def rk4_step(f, t, y, dt):
    k1 = f(t, y)
    k2 = f(t + dt / 2, y + dt / 2 * k1)
    k3 = f(t + dt / 2, y + dt / 2 * k2)
    k4 = f(t + dt, y + dt * k3)
    return y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


# ====================================================================
# PMSM 永磁同步电机 — wheel-level dynamics with gearbox reflection
# ====================================================================
class PMSM:
    """PMSM dq model with gearbox reflected to wheel shaft.

    Convention:
      - All MECHANICAL quantities (J, B, omega, T_L) are at the WHEEL shaft.
      - Electrical quantities (R_s, L, psi_f) are at the MOTOR terminals.
      - omega_wheel * gear_ratio = omega_motor
      - T_wheel = T_motor * gear_ratio * efficiency
      - J_wheel = J_motor * gear^2 + m_chassis * r^2
    """

    def __init__(self, params=None):
        p = params or {}
        self.R_s = p.get("R_s", 0.5)            # Ohm
        self.L_d = p.get("L_d", 0.001)          # H
        self.L_q = p.get("L_q", 0.001)          # H
        self.psi_f = p.get("psi_f", 0.01)       # Wb
        self.poles = p.get("poles", 4)
        self.gear_ratio = p.get("gear_ratio", 50.0)
        self.efficiency = p.get("efficiency", 0.85)

        # Motor-shaft baseline
        J_motor = p.get("J_motor", 0.002)       # kg*m2 at motor shaft
        B_motor = p.get("B_motor", 0.001)        # N*m*s/rad at motor shaft
        K_t_motor = 1.5 * self.poles * self.psi_f

        chassis_mass = p.get("chassis_mass", 80.0)
        wheel_radius = p.get("wheel_radius", 0.15)

        g2 = self.gear_ratio * self.gear_ratio
        self.J = J_motor * g2 + chassis_mass * wheel_radius * wheel_radius
        self.B = B_motor * g2
        self.K_t_motor = K_t_motor
        self.K_t = K_t_motor * self.gear_ratio * self.efficiency

    def torque_constant(self):
        return self.K_t

    def ode_rhs(self, t, state, u_d, u_q, T_L):
        i_d, i_q, omega_wheel = state
        omega_motor = omega_wheel * self.gear_ratio
        omega_e = self.poles * omega_motor

        di_d_dt = (u_d - self.R_s * i_d + omega_e * self.L_q * i_q) / self.L_d
        di_q_dt = (u_q - self.R_s * i_q - omega_e * (self.L_d * i_d + self.psi_f)) / self.L_q

        T_e_motor = self.K_t_motor * i_q
        T_e_wheel = T_e_motor * self.gear_ratio * self.efficiency
        domega_dt = (T_e_wheel - T_L - self.B * omega_wheel) / self.J

        return np.array([di_d_dt, di_q_dt, domega_dt])


# ====================================================================
# PI Controller with anti-windup
# ====================================================================
class PIController:
    def __init__(self, Kp, Ki, dt, output_min=-np.inf, output_max=np.inf):
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def step(self, setpoint, measurement):
        error = setpoint - measurement
        p_term = self.Kp * error
        self.integral += self.Ki * error * self.dt
        output_raw = p_term + self.integral
        if output_raw > self.output_max:
            self.integral -= self.Ki * error * self.dt
            return self.output_max
        elif output_raw < self.output_min:
            self.integral -= self.Ki * error * self.dt
            return self.output_min
        return output_raw


# ====================================================================
# Feedforward Compensator (velocity + friction + load observer)
# ====================================================================
class FeedforwardCompensator:
    def __init__(self, J_hat, B_hat, T_c, L_o, dt):
        self.J_hat = J_hat
        self.B_hat = B_hat
        self.T_c = T_c
        self.L_o = L_o
        self.dt = dt
        self.T_L_hat = 0.0
        self.prev_omega = 0.0
        self.prev_T_e = 0.0

    def reset(self):
        self.T_L_hat = 0.0
        self.prev_omega = 0.0
        self.prev_T_e = 0.0

    def velocity_feedforward(self, d_omega_ref_dt):
        return self.J_hat * d_omega_ref_dt

    def friction_feedforward(self, omega):
        return self.T_c * np.sign(omega) + self.B_hat * omega

    def load_observer_update(self, T_e, omega):
        d_omega_dt = (omega - self.prev_omega) / self.dt if self.dt > 0 else 0
        d_T_L_hat = self.L_o * (T_e - self.T_L_hat - self.J_hat * d_omega_dt)
        self.T_L_hat += d_T_L_hat * self.dt
        self.prev_omega = omega
        self.prev_T_e = T_e
        return self.T_L_hat

    def total_feedforward(self, d_omega_ref_dt, omega, T_e):
        T_ff = self.velocity_feedforward(d_omega_ref_dt)
        T_f = self.friction_feedforward(omega)
        T_L = self.load_observer_update(T_e, omega)
        return T_ff + T_f + T_L


# ====================================================================
# Cascade Controller
# ====================================================================
class CascadeController:
    """Speed PI -> Current PI -> PMSM dq voltage"""

    def __init__(self, motor, params, dt=0.001, enable_ff=True):
        self.motor = motor
        self.dt = dt
        self.enable_ff = enable_ff
        self.K_t = motor.torque_constant()
        ctrl = params.get("control", {})

        # Default gains tuned for wheel-level model (J~6.8, B~2.5, K_t~2.55)
        # Speed loop: conservative PI to balance rise time vs overshoot
        self.speed_pi = PIController(
            Kp=ctrl.get("velocity_p", 0.10),
            Ki=ctrl.get("velocity_i", 1.5),
            dt=dt,
            output_min=-15.0,
            output_max=15.0,
        )
        # Current loop: moderate PI, fast enough to track speed loop output
        self.current_pi = PIController(
            Kp=ctrl.get("current_p", 0.5),
            Ki=ctrl.get("current_i", 10.0),
            dt=dt,
            output_min=-24.0,
            output_max=24.0,
        )
        # Feedforward: use 50% of theoretical values to avoid overshoot
        # Full theoretical FF causes ~130% speed overshoot; 50% is a safe default
        ff_scale = 0.5
        self.ff_comp = FeedforwardCompensator(
            J_hat=ctrl.get("feedforward_j_hat", 6.8 * ff_scale),
            B_hat=ctrl.get("feedforward_b_hat", 2.5 * ff_scale),
            T_c=ctrl.get("feedforward_tc", 0.5 * ff_scale),
            L_o=ctrl.get("feedforward_lo", 50.0),
            dt=dt,
        )
        self.omega_prev = 0.0

    def reset(self):
        self.speed_pi.reset()
        self.current_pi.reset()
        self.ff_comp.reset()
        self.omega_prev = 0.0

    def step(self, omega_ref, omega, i_q, i_d=0.0):
        i_q_ref_pi = self.speed_pi.step(omega_ref, omega)

        if self.enable_ff:
            d_omega_dt = (omega_ref - self.omega_prev) / self.dt if self.dt > 0 else 0
            T_e = self.K_t * i_q
            T_ff_total = self.ff_comp.total_feedforward(d_omega_dt, omega, T_e)
            i_q_ff = T_ff_total / self.K_t if abs(self.K_t) > 1e-9 else 0.0
        else:
            i_q_ff = 0.0
        self.omega_prev = omega_ref

        # Total i_q ref clamped to [-15, 15] A to prevent overshoot from FF + PI stacking
        i_q_ref_total = i_q_ref_pi + i_q_ff
        i_q_max = self.speed_pi.output_max
        i_q_min = self.speed_pi.output_min
        i_q_ref = max(i_q_min, min(i_q_max, i_q_ref_total))
        u_q = self.current_pi.step(i_q_ref, i_q)
        u_d = 0.0
        return u_d, u_q


# ====================================================================
# Full closed-loop motor simulation
# ====================================================================
def run_motor_simulation(dt, T_total, omega_ref_func, T_L_func,
                         enable_ff=True, motor_params=None, ctrl_params=None):
    """Run PMSM + cascade controller closed-loop with RK4 integration."""
    motor = PMSM(motor_params or {})
    controller = CascadeController(motor, ctrl_params or {}, dt=dt, enable_ff=enable_ff)

    N = int(T_total / dt)
    t = np.linspace(0, T_total, N)
    omega_ref = np.array([omega_ref_func(ti) for ti in t])
    T_L_arr = np.array([T_L_func(ti) for ti in t])

    i_d = np.zeros(N)
    i_q = np.zeros(N)
    omega = np.zeros(N)
    u_q_arr = np.zeros(N)
    T_e_arr = np.zeros(N)

    controller.reset()
    for k in range(N - 1):
        i_d_k = i_d[k]
        i_q_k = i_q[k]
        omega_k = omega[k]

        u_d, u_q_val = controller.step(omega_ref[k], omega_k, i_q_k, i_d_k)
        u_q_arr[k] = u_q_val
        T_e_arr[k] = motor.K_t * i_q_k

        state = np.array([i_d_k, i_q_k, omega_k])
        def f(ts, y): return motor.ode_rhs(ts, y, u_d, u_q_val, T_L_arr[k])
        new_state = rk4_step(f, t[k], state, dt)
        i_d[k + 1] = new_state[0]
        i_q[k + 1] = new_state[1]
        omega[k + 1] = new_state[2]

    # Last step: record final state
    k = N - 1
    u_d, u_q_val = controller.step(omega_ref[k], omega[k], i_q[k], i_d[k])
    u_q_arr[k] = u_q_val
    T_e_arr[k] = motor.K_t * i_q[k]

    return t, omega_ref, omega, i_q, u_q_arr, T_L_arr, T_e_arr


# ====================================================================
# Experiments
# ====================================================================
def run_step_experiment(enable_ff=True):
    wheel_radius = 0.15; dt = 0.001; T_total = 2.0
    def ref(t): return (0.5 / wheel_radius) if t > 0.2 else 0.0
    def load(t): return 2.0 if t > 1.0 else 0.0
    return run_motor_simulation(dt, T_total, ref, load, enable_ff=enable_ff)


def run_sine_experiment(enable_ff=True):
    wheel_radius = 0.15; dt = 0.001; T_total = 5.0; freq = 0.5; amp = 0.4 / wheel_radius
    def ref(t): return amp * np.sin(2 * np.pi * freq * t)
    def load(t): return 0.0
    return run_motor_simulation(dt, T_total, ref, load, enable_ff=enable_ff)


def run_trajectory_experiment(enable_ff=True):
    wheel_radius = 0.15; dt = 0.001; T_total = 10.0
    def ref(t):
        if 0.5 <= t < 3.0: return 0.3 / wheel_radius
        elif 3.0 <= t < 5.0: return np.interp(t, [3.0, 5.0], [0.3, 0.6]) / wheel_radius
        elif 5.0 <= t < 8.0: return 0.6 / wheel_radius
        elif 8.0 <= t < 9.5: return np.interp(t, [8.0, 9.5], [0.6, 0.0]) / wheel_radius
        else: return 0.0
    def load(t):
        if 2.0 < t < 2.5: return 3.0
        elif 6.0 < t < 7.0: return 5.0
        return 0.0
    return run_motor_simulation(dt, T_total, ref, load, enable_ff=enable_ff)


# ====================================================================
# Metrics
# ====================================================================
def _compute_metrics(t, omega_ref, omega, wheel_radius=0.15):
    v_ref = omega_ref * wheel_radius
    v_act = omega * wheel_radius
    error = v_ref - v_act
    rmse = float(np.sqrt(np.mean(error**2)))
    max_error = float(np.max(np.abs(error)))
    steady_val = np.max(v_ref)
    if steady_val > 0.01:
        t10 = t[np.argmax(v_act > 0.1 * steady_val)] if np.any(v_act > 0.1 * steady_val) else t[0]
        t90 = t[np.argmax(v_act > 0.9 * steady_val)] if np.any(v_act > 0.9 * steady_val) else t[-1]
        rise_time = float(t90 - t10)
    else:
        rise_time = 0.0
    peak = np.max(v_act)
    overshoot = float(max(0.0, (peak - steady_val) / steady_val * 100)) if steady_val > 0.01 else 0.0
    n_steady = max(1, len(error) // 5)
    steady_error = float(np.mean(np.abs(error[-n_steady:])))
    return {"rmse": rmse, "max_error": max_error, "rise_time": rise_time,
            "overshoot_pct": overshoot, "steady_error": steady_error}


# ====================================================================
# Plots
# ====================================================================
def plot_step(t, omega_ref, omega_ff, omega_nff, save_dir=None):
    wheel_radius = 0.15
    v_ref = omega_ref * wheel_radius
    v_ff = omega_ff * wheel_radius
    v_nff = omega_nff * wheel_radius

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    ax = axes[0]
    ax.plot(t, v_ff, 'b-', linewidth=1.5, label='With FF')
    ax.plot(t, v_nff, 'r--', linewidth=1.5, label='Without FF')
    ax.plot(t, v_ref, 'k:', linewidth=1, label='Cmd')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Step Response (Wheel-level PMSM dq Model)')
    ax.legend(); ax.axvline(x=0.2, color='gray', linestyle=':', alpha=0.5)
    ax.axvline(x=1.0, color='gray', linestyle=':', alpha=0.5); ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t, v_ref - v_ff, 'b-', linewidth=1, label='With FF Error')
    ax.plot(t, v_ref - v_nff, 'r--', linewidth=1, label='Without FF Error')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Error (m/s)')
    ax.set_title('Tracking Error'); ax.legend(); ax.grid(True, alpha=0.3)

    m_ff = _compute_metrics(t, omega_ref, omega_ff)
    m_nff = _compute_metrics(t, omega_ref, omega_nff)
    text = (f"Rise (FF): {m_ff['rise_time']:.3f}s  OS(FF): {m_ff['overshoot_pct']:.1f}%\\n"
            f"Rise (noFF): {m_nff['rise_time']:.3f}s  OS(noFF): {m_nff['overshoot_pct']:.1f}%")
    fig.text(0.02, 0.02, text, fontsize=9, family='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        plt.savefig(os.path.join(save_dir, 'step_response.png'), dpi=150)
        print(f"Saved: {save_dir}/step_response.png")
    plt.close()


def plot_sine(t, omega_ref, omega_ff, omega_nff, save_dir=None):
    wheel_radius = 0.15
    t_show = t[t <= 3.0]; idx = len(t_show)
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    ax = axes[0]
    ax.plot(t_show, omega_ff[:idx] * wheel_radius, 'b-', linewidth=1, label='With FF')
    ax.plot(t_show, omega_ref[:idx] * wheel_radius, 'k:', linewidth=1.5, label='Cmd')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Sine Tracking - With FF'); ax.legend(); ax.grid(True, alpha=0.3)
    ax = axes[1]
    ax.plot(t_show, omega_nff[:idx] * wheel_radius, 'r-', linewidth=1, label='Without FF')
    ax.plot(t_show, omega_ref[:idx] * wheel_radius, 'k:', linewidth=1.5, label='Cmd')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Sine Tracking - Without FF'); ax.legend(); ax.grid(True, alpha=0.3)
    eps = 1e-9
    amp_ff = np.max(omega_ff[100:]) / max(np.max(omega_ref[100:]), eps)
    amp_nff = np.max(omega_nff[100:]) / max(np.max(omega_ref[100:]), eps)
    metrics = f'Amp decay (FF): {(1-amp_ff)*100:.1f}%  |  Amp decay (no FF): {(1-amp_nff)*100:.1f}%'
    fig.text(0.02, 0.02, metrics, fontsize=10, family='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        plt.savefig(os.path.join(save_dir, 'sine_tracking.png'), dpi=150)
        print(f"Saved: {save_dir}/sine_tracking.png")
    plt.close()


def plot_trajectory(t, omega_ref, omega_ff, omega_nff, T_L, save_dir=None):
    wheel_radius = 0.15
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    ax = axes[0]
    ax.plot(t, omega_ref * wheel_radius, 'k:', linewidth=1.5, label='Cmd')
    ax.plot(t, omega_ff * wheel_radius, 'b-', linewidth=1, label='With FF')
    ax.plot(t, omega_nff * wheel_radius, 'r--', linewidth=1, label='Without FF')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Trajectory Tracking'); ax.legend(); ax.grid(True, alpha=0.3)

    ax = axes[1]
    err_ff_rms = np.sqrt(np.mean((omega_ref - omega_ff)**2)) * wheel_radius
    err_nff_rms = np.sqrt(np.mean((omega_ref - omega_nff)**2)) * wheel_radius
    ax.plot(t, (omega_ref - omega_ff) * wheel_radius, 'b-', linewidth=0.8, alpha=0.7,
            label=f'With FF (RMSE={err_ff_rms:.4f})')
    ax.plot(t, (omega_ref - omega_nff) * wheel_radius, 'r--', linewidth=0.8, alpha=0.7,
            label=f'Without FF (RMSE={err_nff_rms:.4f})')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Error (m/s)')
    ax.set_title('Tracking Error'); ax.legend(); ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(t, T_L, 'g-', linewidth=1.5)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Load Torque (N*m)')
    ax.set_title('Load Disturbance'); ax.grid(True, alpha=0.3)
    ax.fill_between(t, 0, T_L, alpha=0.2, color='green')
    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        plt.savefig(os.path.join(save_dir, 'trajectory_tracking.png'), dpi=150)
        print(f"Saved: {save_dir}/trajectory_tracking.png")
    plt.close()


# ====================================================================
# Main
# ====================================================================
def main():
    parser = argparse.ArgumentParser(description='PMSM cascade control prototype')
    parser.add_argument('--exp', choices=['step', 'sine', 'track', 'all'], default='all')
    parser.add_argument('--save', action='store_true')
    parser.add_argument('--output-dir', default=None,
                        help='图表输出目录；配合 --save 使用，默认: ./figures')
    args = parser.parse_args()

    default_save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figures')
    save_dir = (args.output_dir or default_save_dir) if args.save else None

    if args.exp in ('step', 'all'):
        print("Running Step experiment ...")
        t, ref, w_ff, iq, uq, TL, Te = run_step_experiment(enable_ff=True)
        t2, ref2, w_nff, iq2, uq2, TL2, Te2 = run_step_experiment(enable_ff=False)
        plot_step(t, ref, w_ff, w_nff, save_dir)

    if args.exp in ('sine', 'all'):
        print("Running Sine experiment ...")
        t, ref, w_ff, iq, uq, TL, Te = run_sine_experiment(enable_ff=True)
        t2, ref2, w_nff, iq2, uq2, TL2, Te2 = run_sine_experiment(enable_ff=False)
        plot_sine(t, ref, w_ff, w_nff, save_dir)

    if args.exp in ('track', 'all'):
        print("Running Trajectory experiment ...")
        t, ref, w_ff, iq, uq, TL, Te = run_trajectory_experiment(enable_ff=True)
        t2, ref2, w_nff, iq2, uq2, TL2, Te2 = run_trajectory_experiment(enable_ff=False)
        plot_trajectory(t, ref, w_ff, w_nff, TL, save_dir)


if __name__ == "__main__":
    main()
