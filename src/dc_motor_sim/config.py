from dataclasses import dataclass


@dataclass
class DCMotorParameters:
    """Physical parameters for the DC Motor."""
    J: float = 0.01     # Rotor moment of inertia [kg*m^2]
    b: float = 0.1      # Motor viscous friction constant [N*m*s/rad]
    K: float = 0.01     # Electromotive force constant / Torque constant [V/(rad/s) or N*m/A]
    R: float = 1.0      # Armature electrical resistance [Ohms]
    L: float = 0.5      # Armature electrical inductance [H]

    def __post_init__(self):
        if self.J <= 0:
            raise ValueError("Inertia J must be strictly positive.")
        if self.b < 0:
            raise ValueError("Friction b must be non-negative.")
        if self.K <= 0:
            raise ValueError("Motor constant K must be strictly positive.")
        if self.R <= 0:
            raise ValueError("Resistance R must be strictly positive.")
        if self.L <= 0:
            raise ValueError("Inductance L must be strictly positive.")

@dataclass
class ControllerConfig:
    """PID Controller Configuration."""
    Kp: float
    Ki: float
    Kd: float
    N: float = 100.0    # Filter coefficient for derivative

@dataclass
class SimulationConfig:
    """Configuration for simulation execution."""
    dt: float = 0.001           # Sample time for discrete simulation [s]
    sim_time: float = 5.0       # Total simulation duration [s]
    v_min: float = -24.0        # Minimum control voltage [V]
    v_max: float = 24.0         # Maximum control voltage [V]
    max_current: float = 10.0   # Physical current limit [A]
    
    speed_target: float = 1.0   # Target speed [rad/s]
    speed_dist_torque: float = 0.05 # Load torque disturbance for speed test [N*m]
    speed_dist_time: float = 2.5    # Time when disturbance is applied [s]
    
    pos_target: float = 1.0     # Target position [rad]
    pos_dist_torque: float = 0.02   # Load torque disturbance for position test [N*m]
    pos_dist_time: float = 2.5      # Time when disturbance is applied [s]
