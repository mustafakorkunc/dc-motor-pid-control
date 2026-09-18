
import control as ct
import numpy as np

from .config import DCMotorParameters


class DCMotorModel:
    """Mathematical and state-space model of an armature-controlled DC motor.
    
    Note: This is a linear model that assumes constant parameters, viscous friction
    only (no Coulomb/static friction), ideal measurements, and no PWM ripple.
    """

    def __init__(self, params: DCMotorParameters | None = None):
        self.params = params or DCMotorParameters()
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

    def get_poles_and_zeros(self) -> dict:
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
        Continuous-time linear dynamic differential equations.
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
