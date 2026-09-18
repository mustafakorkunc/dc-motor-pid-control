from dc_motor_sim.config import ControllerConfig
from dc_motor_sim.pid import PIDController


def test_pid_proportional():
    config = ControllerConfig(Kp=10.0, Ki=0.0, Kd=0.0)
    pid = PIDController(config, Ts=0.1, v_min=-24.0, v_max=24.0)
    
    u, details = pid.compute_discrete_step(target=1.0, measurement=0.0)
    assert u == 10.0
    assert details['p_term'] == 10.0

def test_pid_saturation_and_anti_windup():
    config = ControllerConfig(Kp=10.0, Ki=5.0, Kd=0.0)
    pid = PIDController(config, Ts=0.1, v_min=-10.0, v_max=10.0)
    
    # Large error that saturates output
    u, details = pid.compute_discrete_step(target=5.0, measurement=0.0)
    
    # Error = 5.0, Kp = 10 -> p_term = 50.0 > v_max
    assert u == 10.0
    assert details['u_unsat'] > 10.0
    
    # Since it's saturated and error is positive, integral should NOT increase (anti-windup)
    assert details['i_term'] == 0.0
