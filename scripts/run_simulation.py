import os

import control as ct
import numpy as np

from dc_motor_sim.analysis import compute_stability_margins
from dc_motor_sim.config import ControllerConfig, DCMotorParameters, SimulationConfig
from dc_motor_sim.metrics import compute_bandwidth, compute_transient_metrics
from dc_motor_sim.motor import DCMotorModel
from dc_motor_sim.pid import PIDController
from dc_motor_sim.plotting import plot_bode, plot_step_responses
from dc_motor_sim.simulation import rk4_step


def generate_readme(context: dict, repo_root: str):
    template_path = os.path.join(repo_root, "README.md.template")
    out_path = os.path.join(repo_root, "README.md")
    
    if not os.path.exists(template_path):
        print("README template not found, skipping generation.")
        return
        
    with open(template_path, encoding='utf-8') as f:
        content = f.read()
        
    for k, v in context.items():
        placeholder = f"{{{{ {k} }}}}"
        if isinstance(v, float):
            v_str = f"{v:.4f}"
        else:
            v_str = str(v)
        content = content.replace(placeholder, v_str)
        
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print("Generated dynamic README.md")

def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    assets_dir = os.path.join(repo_root, "assets")
    
    sim_cfg = SimulationConfig()
    motor_params = DCMotorParameters()
    motor = DCMotorModel(motor_params)
    
    # --- Speed Control ---
    spd_cfg = ControllerConfig(Kp=100.0, Ki=200.0, Kd=10.0, N=100.0)
    pid_spd = PIDController(spd_cfg, Ts=sim_cfg.dt, v_min=sim_cfg.v_min, v_max=sim_cfg.v_max)
    C_spd_tf = pid_spd.to_continuous_tf()
    L_spd_tf = C_spd_tf * motor.P_speed
    T_spd_tf = ct.feedback(L_spd_tf, 1.0)
    
    # Continuous step responses for speed
    t_span = np.arange(0, 1.5, 0.001) # use arange instead of linspace
    _, y_ol = ct.step_response(motor.P_speed, t_span)
    _, y_p_spd = ct.step_response(ct.feedback(ct.tf([spd_cfg.Kp], [1.0]) * motor.P_speed, 1.0), t_span)
    _, y_pi_spd = ct.step_response(ct.feedback(ct.tf([spd_cfg.Kp, spd_cfg.Ki], [1.0, 0.0]) * motor.P_speed, 1.0), t_span)
    _, y_pid_spd = ct.step_response(T_spd_tf, t_span)
    
    metrics_pid_spd = compute_transient_metrics(t_span, y_pid_spd, sim_cfg.speed_target)
    bw_spd = compute_bandwidth(T_spd_tf)
    margins_spd = compute_stability_margins(L_spd_tf)
    
    # Discrete simulation for speed
    time_steps = int(sim_cfg.sim_time / sim_cfg.dt)
    t_discrete = np.arange(0, sim_cfg.sim_time, sim_cfg.dt)
    
    tau_load_spd = np.zeros(time_steps)
    tau_load_spd[t_discrete >= sim_cfg.speed_dist_time] = sim_cfg.speed_dist_torque
    
    state_spd = np.zeros(3) # theta, omega, i_a
    spd_history = np.zeros(time_steps)
    u_spd_history = np.zeros(time_steps)
    
    pid_spd.reset()
    for k in range(time_steps):
        omega_curr = state_spd[1]
        spd_history[k] = omega_curr
        
        u_applied, _ = pid_spd.compute_discrete_step(sim_cfg.speed_target, omega_curr)
        u_spd_history[k] = u_applied
        
        def ode_func_spd(t, s):
            return motor.derivatives(t, s, u_applied, tau_load_spd[k])
            
        state_spd = rk4_step(ode_func_spd, t_discrete[k], state_spd, sim_cfg.dt)
        # Apply current limits if needed
        state_spd[2] = np.clip(state_spd[2], -sim_cfg.max_current, sim_cfg.max_current)
        
    # --- Position Control ---
    pos_cfg = ControllerConfig(Kp=20.0, Ki=0.2, Kd=8.0, N=100.0)
    pid_pos = PIDController(pos_cfg, Ts=sim_cfg.dt, v_min=sim_cfg.v_min, v_max=sim_cfg.v_max)
    C_pos_tf = pid_pos.to_continuous_tf()
    L_pos_tf = C_pos_tf * motor.P_pos
    T_pos_tf = ct.feedback(L_pos_tf, 1.0)
    
    t_pos_span = np.arange(0, 4.0, 0.001)
    _, y_p_pos = ct.step_response(ct.feedback(ct.tf([pos_cfg.Kp], [1.0]) * motor.P_pos, 1.0), t_pos_span)
    
    C_pd_pos = ct.tf([pos_cfg.Kd + pos_cfg.Kp / pos_cfg.N, pos_cfg.Kp], [1.0 / pos_cfg.N, 1.0])
    _, y_pd_pos = ct.step_response(ct.feedback(C_pd_pos * motor.P_pos, 1.0), t_pos_span)
    _, y_pid_pos = ct.step_response(T_pos_tf, t_pos_span)
    
    metrics_pid_pos = compute_transient_metrics(t_pos_span, y_pid_pos, sim_cfg.pos_target)
    
    # Discrete simulation for position
    state_pos = np.zeros(3)
    pos_history = np.zeros(time_steps)
    u_pos_history = np.zeros(time_steps)
    
    tau_load_pos = np.zeros(time_steps)
    tau_load_pos[t_discrete >= sim_cfg.pos_dist_time] = sim_cfg.pos_dist_torque
    
    pid_pos.reset()
    for k in range(time_steps):
        theta_curr = state_pos[0]
        pos_history[k] = theta_curr
        
        u_applied, _ = pid_pos.compute_discrete_step(sim_cfg.pos_target, theta_curr)
        u_pos_history[k] = u_applied
        
        def ode_func_pos(t, s):
            return motor.derivatives(t, s, u_applied, tau_load_pos[k])
            
        state_pos = rk4_step(ode_func_pos, t_discrete[k], state_pos, sim_cfg.dt)
        state_pos[2] = np.clip(state_pos[2], -sim_cfg.max_current, sim_cfg.max_current)

    # --- Plotting ---
    print("Generating plots...")
    plot_step_responses(
        t_span, y_ol, y_p_spd, y_pi_spd, y_pid_spd,
        t_discrete, spd_history, u_spd_history,
        t_pos_span, y_p_pos, y_pd_pos, y_pid_pos, pos_history, u_pos_history,
        metrics_pid_spd, metrics_pid_pos,
        assets_dir
    )
    
    plot_bode(motor.P_speed, L_spd_tf, T_spd_tf, margins_spd['pm_deg'], margins_spd['wcp'], assets_dir)
    print(f"Plots saved to {assets_dir}")
    
    # --- Template Context ---
    ctx = {
        "bw_spd": bw_spd,
        "pm_deg": margins_spd['pm_deg'],
        "wcp": margins_spd['wcp'],
        "spd_rise": metrics_pid_spd['RiseTime'],
        "spd_settle": metrics_pid_spd['SettlingTime'],
        "spd_os": metrics_pid_spd['Overshoot'],
        "spd_ss_err": metrics_pid_spd['SteadyStateError'],
        "pos_rise": metrics_pid_pos['RiseTime'],
        "pos_settle": metrics_pid_pos['SettlingTime'],
        "pos_os": metrics_pid_pos['Overshoot'],
        "pos_ss_err": metrics_pid_pos['SteadyStateError'],
        "load_torque": sim_cfg.speed_dist_torque
    }
    generate_readme(ctx, repo_root)

if __name__ == '__main__':
    main()
