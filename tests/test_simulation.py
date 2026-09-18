import numpy as np

from dc_motor_sim.simulation import rk4_step


def test_rk4_step():
    # Test with a simple ODE: dy/dt = -y, y(0) = 1
    # Analytical solution: y(t) = e^(-t)
    def ode_func(t, y):
        return -y
        
    y_current = np.array([1.0])
    dt = 0.1
    y_next = rk4_step(ode_func, 0.0, y_current, dt)
    
    expected = np.exp(-dt)
    np.testing.assert_almost_equal(y_next[0], expected, decimal=5)

def test_end_to_end_simulation():
    from dc_motor_sim.config import ControllerConfig, DCMotorParameters, SimulationConfig
    from dc_motor_sim.motor import DCMotorModel
    from dc_motor_sim.pid import PIDController
    
    sim_cfg = SimulationConfig(sim_time=2.0, dt=0.01)
    motor = DCMotorModel(DCMotorParameters())
    
    # Very simple P controller for speed
    spd_cfg = ControllerConfig(Kp=10.0, Ki=0.0, Kd=0.0)
    pid = PIDController(spd_cfg, Ts=sim_cfg.dt)
    
    state = np.zeros(3)
    target = 1.0
    
    # Run a short simulation
    time_steps = int(sim_cfg.sim_time / sim_cfg.dt)
    for k in range(time_steps):
        u_applied, _ = pid.compute_discrete_step(target, state[1])
        
        def ode_func(t, s):
            return motor.derivatives(t, s, u_applied, 0.0)
            
        state = rk4_step(ode_func, k * sim_cfg.dt, state, sim_cfg.dt)
        
    # Since it's a P controller with Kp=10, it won't reach exactly 1.0, but should be positive and stable
    final_speed = state[1]
    assert 0.1 < final_speed < 1.0

