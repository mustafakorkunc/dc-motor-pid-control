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
