from collections.abc import Callable

import numpy as np


def rk4_step(ode_func: Callable[[float, np.ndarray], np.ndarray], t: float, y: np.ndarray, dt: float) -> np.ndarray:
    """
    Performs a single step of the classic 4th-order Runge-Kutta (RK4) integration.
    
    Args:
        ode_func: The derivative function dy/dt = f(t, y)
        t: Current time
        y: Current state vector
        dt: Time step
        
    Returns:
        Next state vector y(t + dt)
    """
    k1 = ode_func(t, y)
    k2 = ode_func(t + dt / 2.0, y + (dt / 2.0) * k1)
    k3 = ode_func(t + dt / 2.0, y + (dt / 2.0) * k2)
    k4 = ode_func(t + dt, y + dt * k3)
    
    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
