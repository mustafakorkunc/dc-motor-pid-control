import control as ct
import numpy as np

from .config import ControllerConfig


class PIDController:
    """PID Controller implementation with continuous and discrete capabilities."""

    def __init__(
        self,
        config: ControllerConfig,
        Ts: float = 0.001,
        v_min: float = -24.0,
        v_max: float = 24.0
    ):
        self.Kp = config.Kp
        self.Ki = config.Ki
        self.Kd = config.Kd
        self.N = config.N          # Filter coefficient for derivative: D(s) = Kd * s / (1 + s/N)
        self.Ts = Ts               # Sampling period [s]
        self.v_min = v_min
        self.v_max = v_max

        # Discrete internal states
        self.reset()

    def reset(self):
        """Resets discrete controller memory states."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

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

    def compute_discrete_step(self, target: float, measurement: float) -> tuple[float, dict]:
        """
        Discrete time step execution with anti-windup clamping and filtered derivative.
        Integration uses the explicit forward Euler method.
        """
        error = target - measurement

        # Proportional term
        p_term = self.Kp * error

        # Filtered Derivative term: D[k] = (N * Ts / (1 + N * Ts)) * (Kd * (e[k] - e[k-1])/Ts) + (1 / (1 + N * Ts)) * D[k-1]
        alpha = self.N * self.Ts / (1.0 + self.N * self.Ts)
        d_raw = self.Kd * (error - self.prev_error) / self.Ts if self.Ts > 0 else 0.0
        d_term = alpha * d_raw + (1.0 - alpha) * self.prev_derivative

        # Integral term (Forward Euler)
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

        details = {
            'error': error,
            'p_term': p_term,
            'i_term': self.integral,
            'd_term': d_term,
            'u_unsat': u_unsat,
            'u_sat': u_sat
        }
        return u_sat, details
