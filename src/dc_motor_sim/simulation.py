from collections.abc import Callable

import numpy as np


def rk4_step(ode_func: Callable[..., np.ndarray], t: float, y: np.ndarray, dt: float, *args) -> np.ndarray:
    """
    Performs a single step of the classic 4th-order Runge-Kutta (RK4) integration.
    
    Args:
        ode_func: The derivative function dy/dt = f(t, y, *args)
        t: Current time
        y: Current state vector
        dt: Time step
        *args: Additional arguments to pass to ode_func
        
    Returns:
        Next state vector y(t + dt)
    """
    k1 = ode_func(t, y, *args)
    k2 = ode_func(t + dt / 2.0, y + (dt / 2.0) * k1, *args)
    k3 = ode_func(t + dt / 2.0, y + (dt / 2.0) * k2, *args)
    k4 = ode_func(t + dt, y + dt * k3, *args)
    
    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
