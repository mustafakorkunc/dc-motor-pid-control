
import control as ct
import numpy as np


def compute_transient_metrics(t: np.ndarray, y: np.ndarray, y_target: float) -> dict[str, float]:
    """
    Computes rigorous transient response metrics.
    - Rise time: Time to go from 10% to 90% of the final steady-state value.
    - Settling time: Time to stay within a 2% tolerance band of the target.
    - Peak value and peak time.
    - Percent overshoot (%OS).
    - Steady-state value: Computed using a window average of the final 10% of the signal.
    - Steady-state error.
    """
    t = np.asarray(t)
    y = np.asarray(y)

    y_initial = y[0]
    
    # Calculate steady state using the last 10% of the signal array
    window_size = max(1, len(y) // 10)
    y_final = float(np.mean(y[-window_size:]))
    
    y_range = y_target - y_initial

    # Rise time (10% to 90% of final reference)
    val_10 = y_initial + 0.10 * y_range
    val_90 = y_initial + 0.90 * y_range

    idx_10 = np.where(y >= val_10)[0]
    idx_90 = np.where(y >= val_90)[0]

    if len(idx_10) > 0 and len(idx_90) > 0:
        t_rise = t[idx_90[0]] - t[idx_10[0]]
    else:
        t_rise = np.nan

    # Peak value & overshoot
    idx_peak = np.argmax(y)
    y_peak = y[idx_peak]
    t_peak = t[idx_peak]

    if y_peak > y_target:
        overshoot = ((y_peak - y_target) / abs(y_target)) * 100.0
    else:
        overshoot = 0.0

    # Settling time (2% band around target)
    tol = 0.02 * abs(y_target)
    outside_band = np.where(np.abs(y - y_target) > tol)[0]
    if len(outside_band) == 0:
        t_settling = 0.0
    elif outside_band[-1] == len(t) - 1:
        # Never settled within 2%
        t_settling = np.nan
    else:
        t_settling = t[outside_band[-1] + 1]

    steady_state_error = abs(y_target - y_final)

    return {
        'RiseTime': float(t_rise),
        'SettlingTime': float(t_settling),
        'Peak': float(y_peak),
        'PeakTime': float(t_peak),
        'Overshoot': float(overshoot),
        'SteadyStateValue': float(y_final),
        'SteadyStateError': float(steady_state_error)
    }

def compute_bandwidth(T_tf: ct.TransferFunction) -> float:
    """
    Programmatically calculates the -3dB closed-loop bandwidth of a transfer function.
    """
    # Create frequency vector
    omega = np.logspace(-1, 4, 1000)
    mag, _, _ = ct.frequency_response(T_tf, omega)
    
    # dc magnitude
    mag_0 = mag[0]
    
    # 3dB drop
    mag_3db = mag_0 / np.sqrt(2)
    
    # find the first frequency where magnitude drops below 3dB
    idx = np.where(mag < mag_3db)[0]
    if len(idx) > 0:
        return float(omega[idx[0]])
    return float('nan')
