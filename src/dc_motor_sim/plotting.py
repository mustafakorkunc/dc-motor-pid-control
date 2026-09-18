import os

import control as ct
import matplotlib.pyplot as plt
import numpy as np


def setup_plot_style():
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

def plot_step_responses(
    t_span, y_ol, y_p_spd, y_pi_spd, y_pid_spd,
    t_discrete, spd_history, u_spd_history,
    t_pos_span, y_p_pos, y_pd_pos, y_pid_pos, pos_history, u_pos_history,
    metrics_pid_spd, metrics_pid_pos,
    save_dir
):
    setup_plot_style()
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))

    # (a) Speed Control Step Response
    ax = axs[0, 0]
    ax2 = ax.twinx() # Secondary axis for open loop
    ax2.plot(t_span, y_ol, 'k--', lw=1.5, label='Open Loop (right axis)', alpha=0.7)
    
    ax.plot(t_span, y_p_spd, color='#e67e22', lw=1.8, label=r'P Only ($K_p=100$)')
    ax.plot(t_span, y_pi_spd, color='#2980b9', lw=1.8, label=r'PI ($K_p=100, K_i=200$)')
    ax.plot(t_span, y_pid_spd, color='#27ae60', lw=2.4, label=r'PID ($K_p=100, K_i=200, K_d=10$)')
    ax.axhline(1.0, color='#c0392b', linestyle=':', lw=1.6, label='Reference (1.0 rad/s)')

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
    ax2.set_ylabel(r'Open Loop Speed $\omega$ (rad/s)', color='k')
    ax.grid(True)
    
    # combine legends
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc='lower right', framealpha=0.95)
    
    ax.set_xlim([0, 1.5])
    ax.set_ylim([-0.05, 1.30])

    # (b) Speed Disturbance Rejection
    ax = axs[0, 1]
    ax.plot(t_discrete, spd_history, color='#27ae60', lw=2.2, label=r'Closed-Loop Speed $\omega(t)$')
    label_dist = r'Load Disturbance ($\tau_L = 0.05\ \mathrm{N\cdot m}$)'
    ax.axvline(2.5, color='#c0392b', linestyle='--', lw=1.8, label=label_dist)
    ax.axhline(1.0, color='#c0392b', linestyle=':', lw=1.6, label='Reference (1.0 rad/s)')

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
    ax.plot(t_pos_span, y_pid_pos, color='#8e44ad', lw=2.4, label=r'PID ($K_p=20, K_i=0.2, K_d=8$)')
    label_rk4 = r'Saturated Discrete RK4 ($\pm 24\ \mathrm{V}$)'
    ax.plot(t_discrete, pos_history, color='#2c3e50', linestyle='-.', lw=1.8, label=label_rk4)
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
    label_pos_u = r'Position Control Voltage $V_a(t)$'
    ax.plot(t_discrete, u_pos_history, color='#8e44ad', lw=1.8, linestyle='-.', label=label_pos_u)
    ax.axhline(24.0, color='gray', linestyle=':', lw=1.4, label=r'Voltage Limits ($\pm 24$ V)')
    ax.axhline(-24.0, color='gray', linestyle=':', lw=1.4)
    ax.set_title(r'(d) Control Effort (Armature Input Voltage $V_a$)')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel(r'Applied Voltage $V_a$ (Volts)')
    ax.grid(True)
    ax.legend(loc='upper right', framealpha=0.95)
    ax.set_xlim([0, 5.0])
    ax.set_ylim([-30.0, 30.0])

    plt.tight_layout()
    os.makedirs(save_dir, exist_ok=True)
    plt.savefig(os.path.join(save_dir, 'step_response.png'), dpi=300)
    plt.savefig(os.path.join(save_dir, 'step_response.svg'))
    plt.close()


def plot_bode(P_tf, L_tf, T_tf, pm, wcp, save_dir):
    setup_plot_style()
    omega_vec = np.logspace(-1, 3, 2000)

    mag_P, phase_P, _ = ct.frequency_response(P_tf, omega_vec)
    mag_L, phase_L, _ = ct.frequency_response(L_tf, omega_vec)
    mag_T, phase_T, _ = ct.frequency_response(T_tf, omega_vec)

    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(11, 8.5), sharex=True)

    # Magnitude
    ax_mag.semilogx(omega_vec, 20 * np.log10(mag_P), 'k--', lw=1.6, label=r'Open-Loop Plant $P(s)$', alpha=0.7)
    label_L = r'Compensated Loop $L(s) = C(s)P(s)$'
    ax_mag.semilogx(omega_vec, 20 * np.log10(mag_L), color='#2980b9', lw=2.2, label=label_L)
    label_T = r'Closed-Loop Complementary $T(s)$'
    ax_mag.semilogx(omega_vec, 20 * np.log10(mag_T), color='#27ae60', lw=2.0, label=label_T)
    ax_mag.axhline(0, color='gray', linestyle=':', lw=1.2)

    if wcp is not None and not np.isnan(wcp):
        ax_mag.axvline(wcp, color='#c0392b', linestyle='--', lw=1.4)
        ax_mag.scatter([wcp], [0.0], color='#c0392b', s=45, zorder=5)

    ax_mag.set_title(r'Bode Diagram: Robustness & Frequency Domain Stability Analysis')
    ax_mag.set_ylabel('Magnitude (dB)')
    ax_mag.grid(True, which='both')
    ax_mag.legend(loc='lower left', framealpha=0.95)
    ax_mag.set_ylim([-60, 50])

    # Phase
    phase_P_deg = np.rad2deg(phase_P)
    phase_L_deg = np.rad2deg(phase_L)
    phase_T_deg = np.rad2deg(phase_T)

    ax_phase.semilogx(omega_vec, phase_P_deg, 'k--', lw=1.6, label=r'Open-Loop Plant $P(s)$', alpha=0.7)
    ax_phase.semilogx(omega_vec, phase_L_deg, color='#2980b9', lw=2.2, label=r'Compensated Loop $L(s)$')
    ax_phase.semilogx(omega_vec, phase_T_deg, color='#27ae60', lw=2.0, label=r'Closed-Loop $T(s)$')
    ax_phase.axhline(-180, color='gray', linestyle=':', lw=1.2)

    if wcp is not None and not np.isnan(wcp):
        ax_phase.axvline(wcp, color='#c0392b', linestyle='--', lw=1.4)

    ax_phase.set_xlabel(r'Frequency $\omega$ (rad/s)')
    ax_phase.set_ylabel('Phase (deg)')
    ax_phase.set_yticks([-270, -180, -90, 0, 90])
    ax_phase.grid(True, which='both')
    ax_phase.legend(loc='lower left', framealpha=0.95)
    ax_phase.set_xlim([1e-1, 1e3])
    ax_phase.set_ylim([-220, 30])

    plt.tight_layout()
    os.makedirs(save_dir, exist_ok=True)
    plt.savefig(os.path.join(save_dir, 'bode_plot.png'), dpi=300)
    plt.savefig(os.path.join(save_dir, 'bode_plot.svg'))
    plt.close()
