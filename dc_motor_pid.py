import os
import sys
from dataclasses import dataclass
from typing import Dict, Tuple, Optional
import numpy as np
import scipy.signal as signal
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import control as ct

# Set publication style for matplotlib
plt.rcParams.update({
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.titlesize': 14,
    'lines.linewidth': 1.8,
    'grid.alpha': 0.5,
    'grid.linestyle': '--',
    'image.cmap': 'viridis',
    'savefig.dpi': 300,
    'savefig.bbox': 'tight'
})


@dataclass
class DCMotorParameters:
    """Physical parameters for the DC Motor."""
    J: float = 0.01     # Rotor moment of inertia [kg*m^2]
    b: float = 0.1      # Motor viscous friction constant [N*m*s/rad]
    K: float = 0.01     # Electromotive force constant / Torque constant [V/(rad/s) or N*m/A]
    R: float = 1.0      # Armature electrical resistance [Ohms]
    L: float = 0.5      # Armature electrical inductance [H]

    def __post_init__(self):
        assert self.J > 0, "Inertia J must be strictly positive"
        assert self.b >= 0, "Friction b must be non-negative"
        assert self.K > 0, "Motor constant K must be strictly positive"
        assert self.R > 0, "Resistance R must be strictly positive"
        assert self.L > 0, "Inductance L must be strictly positive"


class DCMotorModel:
    """Mathematical and state-space model of an armature-controlled DC motor."""

    def __init__(self, params: DCMotorParameters = DCMotorParameters()):
        self.params = params
        self._build_transfer_functions()
        self._build_state_space()

    def _build_transfer_functions(self):
        """Constructs open-loop transfer functions for speed and position."""
        J, b, K, R, L = (
            self.params.J,
            self.params.b,
            self.params.K,
            self.params.R,
            self.params.L
        )

        # Speed transfer function: Omega(s) / V(s)
        # P(s) = K / [ (J*s + b)*(L*s + R) + K^2 ]
        #      = K / [ (J*L)*s^2 + (J*R + b*L)*s + (b*R + K^2) ]
        self.num_speed = [K]
        self.den_speed = [J * L, J * R + b * L, b * R + K**2]
        self.P_speed = ct.tf(self.num_speed, self.den_speed)

        # Position transfer function: Theta(s) / V(s) = P(s) / s
        self.num_pos = [K]
        self.den_pos = [J * L, J * R + b * L, b * R + K**2, 0.0]
        self.P_pos = ct.tf(self.num_pos, self.den_pos)

    def _build_state_space(self):
        """State-space realization: x = [theta, omega, i_a]^T."""
        J, b, K, R, L = (
            self.params.J,
            self.params.b,
            self.params.K,
            self.params.R,
            self.params.L
        )

        self.A = np.array([
            [0.0, 1.0, 0.0],
            [0.0, -b / J, K / J],
            [0.0, -K / L, -R / L]
        ])
        self.B = np.array([
            [0.0],
            [0.0],
            [1.0 / L]
        ])
        self.B_dist = np.array([
            [0.0],
            [-1.0 / J],
            [0.0]
        ])

        # Outputs: [position, speed, current]
        self.C = np.eye(3)
        self.D = np.zeros((3, 1))

    def get_poles_and_zeros(self) -> Dict:
        """Calculates open-loop poles, zeros, and natural frequency characteristics."""
        poles = self.P_speed.poles()
        zeros = self.P_speed.zeros()
        dc_gain = ct.dcgain(self.P_speed)

        # Characteristic polynomial: s^2 + 2*zeta*wn*s + wn^2
        a0 = self.den_speed[0]
        a1 = self.den_speed[1]
        a2 = self.den_speed[2]
        wn = np.sqrt(a2 / a0)
        zeta = (a1 / a0) / (2.0 * wn)

        return {
            'poles': poles,
            'zeros': zeros,
            'dc_gain': float(dc_gain),
            'wn': wn,
            'zeta': zeta
        }

    def derivatives(self, t: float, state: np.ndarray, v_applied: float, tau_load: float = 0.0) -> np.ndarray:
        """
        Continuous-time nonlinear/linear dynamic differential equations.
        state = [theta, omega, i_a]
        """
        theta, omega, i_a = state
        J, b, K, R, L = (
            self.params.J,
            self.params.b,
            self.params.K,
            self.params.R,
            self.params.L
        )

        dtheta_dt = omega
        domega_dt = (K * i_a - b * omega - tau_load) / J
        di_a_dt = (v_applied - R * i_a - K * omega) / L

        return np.array([dtheta_dt, domega_dt, di_a_dt])


class PIDController:
    """PID Controller implementation with continuous and discrete capabilities."""

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        N: float = 100.0,
        Ts: float = 0.001,
        v_min: float = -24.0,
        v_max: float = 24.0
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.N = N          # Filter coefficient for derivative: D(s) = Kd * s / (1 + s/N)
        self.Ts = Ts        # Sampling period [s]
        self.v_min = v_min
        self.v_max = v_max

        # Discrete internal states
        self.reset()

    def reset(self):
        """Resets discrete controller memory states."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_measurement = 0.0

    def to_continuous_tf(self) -> ct.TransferFunction:
        """
        Returns continuous-time transfer function C(s):
        C(s) = Kp + Ki/s + Kd * s / (s/N + 1)
             = [ (Kd + Kp/N)*s^2 + (Kp + Ki/N)*s + Ki ] / [ (1/N)*s^2 + s ]
        """
        if self.N is not None and self.N > 0 and self.Kd > 0:
            num = [self.Kd + self.Kp / self.N, self.Kp + self.Ki / self.N, self.Ki]
            den = [1.0 / self.N, 1.0, 0.0]
        elif self.Kd > 0:
            num = [self.Kd, self.Kp, self.Ki]
            den = [1.0, 0.0]
        else:
            num = [self.Kp, self.Ki]
            den = [1.0, 0.0]

        return ct.tf(num, den)

    def compute_discrete_step(self, target: float, measurement: float) -> Tuple[float, Dict]:
        """
        Discrete time step execution with anti-windup clamping and filtered derivative.
        """
        error = target - measurement

        # Proportional term
        p_term = self.Kp * error

        # Filtered Derivative term: D[k] = (N * Ts / (1 + N * Ts)) * (Kd * (e[k] - e[k-1])/Ts) + (1 / (1 + N * Ts)) * D[k-1]
        alpha = self.N * self.Ts / (1.0 + self.N * self.Ts)
        d_raw = self.Kd * (error - self.prev_error) / self.Ts if self.Ts > 0 else 0.0
        d_term = alpha * d_raw + (1.0 - alpha) * self.prev_derivative

        # Tentative Integral term (Trapezoidal / Euler)
        i_term_tentative = self.integral + self.Ki * self.Ts * error

        # Saturated control output
        u_unsat = p_term + i_term_tentative + d_term
        u_sat = np.clip(u_unsat, self.v_min, self.v_max)

        # Anti-windup clamping: only integrate if not saturated or error drives away from saturation
        if (u_unsat == u_sat) or (u_unsat > self.v_max and error < 0) or (u_unsat < self.v_min and error > 0):
            self.integral = i_term_tentative

        # Update memories
        self.prev_error = error
        self.prev_derivative = d_term
        self.prev_measurement = measurement

        details = {
            'error': error,
            'p_term': p_term,
            'i_term': self.integral,
            'd_term': d_term,
            'u_unsat': u_unsat,
            'u_sat': u_sat
        }
        return u_sat, details


def compute_transient_metrics(t: np.ndarray, y: np.ndarray, y_target: float) -> Dict[str, float]:
    """
    Computes rigorous transient response metrics:
    - Rise time (10% to 90%)
    - Settling time (2% tolerance band)
    - Peak value and peak time
    - Percent overshoot (%OS)
    - Steady-state value and steady-state error
    """
    t = np.asarray(t)
    y = np.asarray(y)

    y_initial = y[0]
    y_final = y[-1]
    y_range = y_target - y_initial

    # Rise time (10% to 90% of final reference)
    val_10 = y_initial + 0.10 * y_range
    val_90 = y_initial + 0.90 * y_range

    idx_10 = np.where(y >= val_10)[0]
    idx_90 = np.where(y >= val_90)[0]

    if len(idx_10) > 0 and len(idx_90) > 0:
        t_rise = t[idx_90[0]] - t[idx_10[0]]
    else:
        t_rise = np.nan

    # Peak value & overshoot
    idx_peak = np.argmax(y)
    y_peak = y[idx_peak]
    t_peak = t[idx_peak]

    if y_peak > y_target:
        overshoot = ((y_peak - y_target) / abs(y_target)) * 100.0
    else:
        overshoot = 0.0

    # Settling time (2% band around target)
    tol = 0.02 * abs(y_target)
    outside_band = np.where(np.abs(y - y_target) > tol)[0]
    if len(outside_band) == 0:
        t_settling = 0.0
    elif outside_band[-1] == len(t) - 1:
        # Never settled within 2%
        t_settling = np.nan
    else:
        t_settling = t[outside_band[-1] + 1]

    steady_state_error = abs(y_target - y_final)

    return {
        'RiseTime': float(t_rise),
        'SettlingTime': float(t_settling),
        'Peak': float(y_peak),
        'PeakTime': float(t_peak),
        'Overshoot': float(overshoot),
        'SteadyStateValue': float(y_final),
        'SteadyStateError': float(steady_state_error)
    }


def run_full_simulation():
    """Runs complete DC motor speed and position control simulation and generates figures."""
    assets_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets')
    os.makedirs(assets_dir, exist_ok=True)

    print("=" * 70)
    print("DC MOTOR SPEED & POSITION PID CONTROL SIMULATION")
    print("=" * 70)

    # 1. Initialize physical plant
    motor = DCMotorModel()
    p_info = motor.get_poles_and_zeros()

    print(f"Plant Physical Parameters:")
    print(f"  Rotor Inertia (J)       : {motor.params.J} kg*m^2")
    print(f"  Friction Damping (b)    : {motor.params.b} N*m*s/rad")
    print(f"  Back-EMF / Torque (K)   : {motor.params.K} V/(rad/s) or N*m/A")
    print(f"  Armature Resistance (R) : {motor.params.R} Ohms")
    print(f"  Armature Inductance (L) : {motor.params.L} H")
    print("\nSpeed Open-Loop Transfer Function P(s) = Omega(s) / V(s):")
    print(motor.P_speed)
    print(f"Poles: {p_info['poles']}")
    print(f"DC Gain: {p_info['dc_gain']:.4f} (rad/s)/V")
    print(f"Natural Frequency wn: {p_info['wn']:.4f} rad/s, Damping Ratio zeta: {p_info['zeta']:.4f}")

    # 2. Controllers Definition
    # Speed control PID gains
    kp_spd, ki_spd, kd_spd = 100.0, 200.0, 10.0
    pid_spd = PIDController(Kp=kp_spd, Ki=ki_spd, Kd=kd_spd, N=100.0)
    C_spd_tf = pid_spd.to_continuous_tf()
    L_spd_tf = C_spd_tf * motor.P_speed
    T_spd_tf = ct.feedback(L_spd_tf, 1.0)

    # Comparative speed controllers for benchmark
    C_p_spd = ct.tf([kp_spd], [1.0])
    T_p_spd = ct.feedback(C_p_spd * motor.P_speed, 1.0)

    C_pi_spd = ct.tf([kp_spd, ki_spd], [1.0, 0.0])
    T_pi_spd = ct.feedback(C_pi_spd * motor.P_speed, 1.0)

    # Position control PID gains
    kp_pos, ki_pos, kd_pos = 20.0, 0.2, 8.0
    pid_pos = PIDController(Kp=kp_pos, Ki=ki_pos, Kd=kd_pos, N=100.0)
    C_pos_tf = pid_pos.to_continuous_tf()
    L_pos_tf = C_pos_tf * motor.P_pos
    T_pos_tf = ct.feedback(L_pos_tf, 1.0)

    # Comparative benchmark controllers for position
    C_p_pos = ct.tf([kp_pos], [1.0])
    T_p_pos = ct.feedback(C_p_pos * motor.P_pos, 1.0)

    C_pd_pos = ct.tf([kd_pos + kp_pos / 100.0, kp_pos], [1.0 / 100.0, 1.0])
    T_pd_pos = ct.feedback(C_pd_pos * motor.P_pos, 1.0)

    # 3. Continuous Simulation Time Vectors
    t_span = np.linspace(0, 2.0, 2000)

    # Speed step responses (Target = 1.0 rad/s)
    _, y_ol = ct.step_response(motor.P_speed, t_span)
    _, y_p = ct.step_response(T_p_spd, t_span)
    _, y_pi = ct.step_response(T_pi_spd, t_span)
    _, y_pid = ct.step_response(T_spd_tf, t_span)

    metrics_pid_spd = compute_transient_metrics(t_span, y_pid, 1.0)
    print("\n" + "-" * 50)
    print("SPEED CONTROL PERFORMANCE METRICS (PID Tuned):")
    for k, v in metrics_pid_spd.items():
        print(f"  {k:20s}: {v:8.4f}")

    # Position step response (Target = 1.0 rad)
    t_pos_span = np.linspace(0, 4.0, 4000)
    _, y_p_pos = ct.step_response(T_p_pos, t_pos_span)
    _, y_pd_pos = ct.step_response(T_pd_pos, t_pos_span)
    _, y_pos_pid = ct.step_response(T_pos_tf, t_pos_span)
    metrics_pid_pos = compute_transient_metrics(t_pos_span, y_pos_pid, 1.0)
    print("\n" + "-" * 50)
    print("POSITION CONTROL PERFORMANCE METRICS (PID Tuned):")
    for k, v in metrics_pid_pos.items():
        print(f"  {k:20s}: {v:8.4f}")

    # 4. Discrete Time Dynamic Simulation with Load Disturbance Rejection
    # Simulates continuous motor non-linear ODE with discrete PID controller
    dt = 0.001
    sim_time = 5.0
    time_steps = int(sim_time / dt)
    t_discrete = np.linspace(0, sim_time, time_steps)

    # Speed disturbance test: target 1.0 rad/s, disturbance load torque 0.05 N*m applied at t=2.5s
    tau_load_spd = np.zeros(time_steps)
    tau_load_spd[t_discrete >= 2.5] = 0.05

    state_spd = np.zeros(3)  # [theta, omega, i_a]
    spd_history = np.zeros(time_steps)
    u_spd_history = np.zeros(time_steps)
    current_spd_history = np.zeros(time_steps)

    pid_discrete_spd = PIDController(Kp=kp_spd, Ki=ki_spd, Kd=kd_spd, N=100.0, Ts=dt, v_min=-24, v_max=24)

    for k in range(time_steps):
        t_curr = t_discrete[k]
        omega_curr = state_spd[1]
        spd_history[k] = omega_curr
        current_spd_history[k] = state_spd[2]

        # Target step
        target = 1.0
        u_applied, _ = pid_discrete_spd.compute_discrete_step(target, omega_curr)
        u_spd_history[k] = u_applied

        # Integrate plant ODE across 1 sampling step using RK4
        def ode_func(t, s):
            return motor.derivatives(t, s, u_applied, tau_load_spd[k])

        sol = solve_ivp(ode_func, [t_curr, t_curr + dt], state_spd, method='RK45')
        state_spd = sol.y[:, -1]

    # Position tracking test: setpoint 1.0 rad, load torque disturbance 0.02 N*m at t=2.5s
    state_pos = np.zeros(3)
    pos_history = np.zeros(time_steps)
    u_pos_history = np.zeros(time_steps)
    pid_discrete_pos = PIDController(Kp=kp_pos, Ki=ki_pos, Kd=kd_pos, N=100.0, Ts=dt, v_min=-24, v_max=24)

    for k in range(time_steps):
        t_curr = t_discrete[k]
        theta_curr = state_pos[0]
        pos_history[k] = theta_curr

        target = 1.0
        dist_torque = 0.0  # pure position tracking step response
        u_applied, _ = pid_discrete_pos.compute_discrete_step(target, theta_curr)
        u_pos_history[k] = u_applied

        def ode_func(t, s):
            return motor.derivatives(t, s, u_applied, dist_torque)

        sol = solve_ivp(ode_func, [t_curr, t_curr + dt], state_pos, method='RK45')
        state_pos = sol.y[:, -1]

    # 5. Generate Publication-Quality Step Response Plot
    print("\nGenerating 'step_response.png'...")
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))

    # (a) Speed Control Step Response Comparison
    ax = axs[0, 0]
    ax.plot(t_span, y_ol * 10, 'k--', lw=1.5, label='Open Loop (scaled x10)', alpha=0.7)
    ax.plot(t_span, y_p, color='#e67e22', lw=1.8, label=r'P Only ($K_p=100$)')
    ax.plot(t_span, y_pi, color='#2980b9', lw=1.8, label=r'PI ($K_p=100, K_i=200$)')
    ax.plot(t_span, y_pid, color='#27ae60', lw=2.4, label=r'PID ($K_p=100, K_i=200, K_d=10$)')
    ax.axhline(1.0, color='#c0392b', linestyle=':', lw=1.6, label='Reference (1.0 rad/s)')

    # Annotations for metrics
    ann_text = (
        f"PID Speed Metrics:\n"
        f"Rise Time: {metrics_pid_spd['RiseTime']:.3f} s\n"
        f"Settling Time: {metrics_pid_spd['SettlingTime']:.3f} s\n"
        f"Peak Overshoot: {metrics_pid_spd['Overshoot']:.2f} %\n"
        f"Steady Error: {metrics_pid_spd['SteadyStateError']:.4f}"
    )
    ax.text(0.38, 0.44, ann_text, transform=ax.transAxes,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8f9fa', edgecolor='#bdc3c7', alpha=0.92),
            fontsize=9.0)

    ax.set_title(r'(a) Closed-Loop Speed Step Response $\omega(t)$')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel(r'Angular Speed $\omega$ (rad/s)')
    ax.grid(True)
    ax.legend(loc='lower right', framealpha=0.95)
    ax.set_xlim([0, 1.5])
    ax.set_ylim([-0.05, 1.30])

    # (b) Speed Disturbance Rejection & Control Effort
    ax = axs[0, 1]
    ax.plot(t_discrete, spd_history, color='#27ae60', lw=2.2, label=r'Closed-Loop Speed $\omega(t)$')
    ax.axvline(2.5, color='#c0392b', linestyle='--', lw=1.8, label=r'Load Disturbance ($\tau_L = 0.05\ \mathrm{N\cdot m}$)')
    ax.axhline(1.0, color='#c0392b', linestyle=':', lw=1.6, label='Reference (1.0 rad/s)')
    ax.annotate(r"Load Torque Injected" + "\n" + r"$\tau_L = 0.05\ \mathrm{N\cdot m}$",
                xy=(2.5, 0.78), xytext=(1.05, 0.74),
                arrowprops=dict(arrowstyle="->", color='#c0392b', lw=1.4),
                fontsize=9.0, color='#922b21', fontweight='bold')
    ax.annotate("Zero Steady-State Droop\n(Integral Action Rejection)",
                xy=(4.2, 1.0), xytext=(2.7, 0.92),
                arrowprops=dict(arrowstyle="->", color='#196f3d', lw=1.4),
                fontsize=9.0, color='#145a32', fontweight='bold')

    ax.set_title(r'(b) Speed Disturbance Rejection Under Load Torque')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel(r'Angular Speed $\omega$ (rad/s)')
    ax.grid(True)
    ax.legend(loc='lower right', framealpha=0.95)
    ax.set_xlim([0, 5.0])
    ax.set_ylim([0.70, 1.10])

    # (c) Position Tracking Step Response
    ax = axs[1, 0]
    ax.plot(t_pos_span, y_p_pos, color='#e67e22', lw=1.6, linestyle=':', label=r'P Only ($K_p=20$)')
    ax.plot(t_pos_span, y_pd_pos, color='#2980b9', lw=1.8, linestyle='--', label=r'PD ($K_p=20, K_d=8$)')
    ax.plot(t_pos_span, y_pos_pid, color='#8e44ad', lw=2.4, label=r'PID ($K_p=20, K_i=0.2, K_d=8$)')
    ax.plot(t_discrete, pos_history, color='#2c3e50', linestyle='-.', lw=1.8, label=r'Saturated Discrete RK4 ($\pm 24\ \mathrm{V}$)')
    ax.axhline(1.0, color='#c0392b', linestyle=':', lw=1.6, label='Reference (1.0 rad)')

    ann_pos = (
        f"Position PID Metrics:\n"
        f"Rise Time: {metrics_pid_pos['RiseTime']:.3f} s\n"
        f"Settling Time: {metrics_pid_pos['SettlingTime']:.3f} s\n"
        f"Peak Overshoot: {metrics_pid_pos['Overshoot']:.2f} %"
    )
    ax.text(0.38, 0.40, ann_pos, transform=ax.transAxes,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8f9fa', edgecolor='#bdc3c7', alpha=0.92),
            fontsize=9.0)

    ax.set_title(r'(c) Position Control Step Response $\theta(t)$')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel(r'Rotor Position $\theta$ (rad)')
    ax.grid(True)
    ax.legend(loc='lower right', framealpha=0.95)
    ax.set_xlim([0, 4.0])
    ax.set_ylim([-0.05, 1.30])

    # (d) Control Effort / Voltage Input
    ax = axs[1, 1]
    ax.plot(t_discrete, u_spd_history, color='#27ae60', lw=1.8, label=r'Speed Control Voltage $V_a(t)$')
    ax.plot(t_discrete, u_pos_history, color='#8e44ad', lw=1.8, linestyle='-.', label=r'Position Control Voltage $V_a(t)$')
    ax.axhline(24.0, color='gray', linestyle=':', lw=1.4, label=r'Voltage Limits ($\pm 24$ V)')
    ax.axhline(-24.0, color='gray', linestyle=':', lw=1.4)
    ax.set_title(r'(d) Control Effort (Armature Input Voltage $V_a$)')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel(r'Applied Voltage $V_a$ (Volts)')
    ax.grid(True)
    ax.legend(loc='upper right', framealpha=0.95)
    ax.set_xlim([0, 5.0])
    ax.set_ylim([-5.0, 27.0])

    plt.tight_layout()
    step_plot_path = os.path.join(assets_dir, 'step_response.png')
    plt.savefig(step_plot_path, dpi=300)
    plt.close()
    print(f"Step response plot saved successfully to: {step_plot_path}")

    # 6. Generate Publication-Quality Bode Plot
    print("\nGenerating 'bode_plot.png'...")
    omega_vec = np.logspace(-1, 3, 2000)

    # Compute frequency responses
    mag_P, phase_P, _ = ct.frequency_response(motor.P_speed, omega_vec)
    mag_L, phase_L, _ = ct.frequency_response(L_spd_tf, omega_vec)
    mag_T, phase_T, _ = ct.frequency_response(T_spd_tf, omega_vec)

    # Stability margins
    gm, pm, sm, wcg, wcp, wcs = ct.stability_margins(L_spd_tf)
    gm_db = 20.0 * np.log10(gm) if gm is not None and gm > 0 and not np.isinf(gm) else np.inf

    print(f"Stability Margins for Speed Control Loop L(s):")
    print(f"  Gain Margin (GM)  : {gm_db if not np.isinf(gm_db) else 'Infinite'} dB")
    print(f"  Phase Margin (PM) : {pm:.2f} deg at w_cp = {wcp:.2f} rad/s")

    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(11, 8.5), sharex=True)

    # Magnitude Subplot
    ax_mag.semilogx(omega_vec, 20 * np.log10(mag_P), 'k--', lw=1.6, label=r'Open-Loop Plant $P(s)$', alpha=0.7)
    ax_mag.semilogx(omega_vec, 20 * np.log10(mag_L), color='#2980b9', lw=2.2, label=r'Compensated Loop $L(s) = C(s)P(s)$')
    ax_mag.semilogx(omega_vec, 20 * np.log10(mag_T), color='#27ae60', lw=2.0, label=r'Closed-Loop Complementary $T(s)$')
    ax_mag.axhline(0, color='gray', linestyle=':', lw=1.2)

    # Highlight 0 dB crossover
    if wcp is not None and not np.isnan(wcp):
        ax_mag.axvline(wcp, color='#c0392b', linestyle='--', lw=1.4)
        ax_mag.scatter([wcp], [0.0], color='#c0392b', s=45, zorder=5)
        ax_mag.annotate(f"0 dB Crossover Frequency\n$\\omega_{{cp}} = {wcp:.2f}$ rad/s",
                        xy=(wcp, 0), xytext=(wcp * 2.2, 10),
                        arrowprops=dict(arrowstyle='->', color='#c0392b', lw=1.4),
                        fontsize=9.5, fontweight='bold', color='#c0392b')

    ax_mag.set_title(r'Bode Diagram: Robustness & Frequency Domain Stability Analysis')
    ax_mag.set_ylabel('Magnitude (dB)')
    ax_mag.grid(True, which='both')
    ax_mag.legend(loc='lower left', framealpha=0.95)
    ax_mag.set_ylim([-60, 50])

    # Phase Subplot
    phase_P_deg = np.rad2deg(phase_P)
    phase_L_deg = np.rad2deg(phase_L)
    phase_T_deg = np.rad2deg(phase_T)

    ax_phase.semilogx(omega_vec, phase_P_deg, 'k--', lw=1.6, label=r'Open-Loop Plant $P(s)$', alpha=0.7)
    ax_phase.semilogx(omega_vec, phase_L_deg, color='#2980b9', lw=2.2, label=r'Compensated Loop $L(s)$')
    ax_phase.semilogx(omega_vec, phase_T_deg, color='#27ae60', lw=2.0, label=r'Closed-Loop $T(s)$')
    ax_phase.axhline(-180, color='gray', linestyle=':', lw=1.2)

    if wcp is not None and not np.isnan(wcp):
        # Phase margin annotation
        idx_wcp = np.argmin(np.abs(omega_vec - wcp))
        phi_at_wcp = phase_L_deg[idx_wcp]
        ax_phase.axvline(wcp, color='#c0392b', linestyle='--', lw=1.4)
        ax_phase.plot([wcp, wcp], [-180, phi_at_wcp], color='#c0392b', lw=3.0, solid_capstyle='round')
        ax_phase.annotate(f"Phase Margin: {pm:.1f}$^\\circ$\n(Robust Stability Band)",
                          xy=(wcp, phi_at_wcp / 2 - 90), xytext=(wcp * 2.5, -135),
                          arrowprops=dict(arrowstyle='->', color='#c0392b', lw=1.4),
                          bbox=dict(boxstyle='round,pad=0.45', facecolor='#fce4ec', edgecolor='#c0392b', alpha=0.95),
                          fontsize=9.5, color='#880e4f', fontweight='bold')

    ax_phase.set_xlabel(r'Frequency $\omega$ (rad/s)')
    ax_phase.set_ylabel('Phase (deg)')
    ax_phase.set_yticks([-270, -180, -90, 0, 90])
    ax_phase.grid(True, which='both')
    ax_phase.legend(loc='lower left', framealpha=0.95)
    ax_phase.set_xlim([1e-1, 1e3])
    ax_phase.set_ylim([-220, 30])

    plt.tight_layout()
    bode_plot_path = os.path.join(assets_dir, 'bode_plot.png')
    plt.savefig(bode_plot_path, dpi=300)
    plt.close()
    print(f"Bode plot saved successfully to: {bode_plot_path}")

    # 7. Generate System Schematic Diagram
    try:
        from generate_schematic import draw_system_schematic
        schematic_path = os.path.join(assets_dir, 'dc_motor_schematic.png')
        draw_system_schematic(schematic_path)
    except Exception as e:
        print(f"Note on schematic generation: {e}")

    print("\n" + "=" * 70)
    print("SIMULATION EXECUTED SUCCESSFULLY. ASSETS GENERATED.")
    print("=" * 70)


if __name__ == '__main__':
    run_full_simulation()
