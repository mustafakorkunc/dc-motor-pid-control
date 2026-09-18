import control as ct
import numpy as np

from dc_motor_sim.metrics import compute_bandwidth, compute_transient_metrics


def test_compute_transient_metrics():
    t = np.linspace(0, 10, 1000)
    
    # Create a dummy underdamped step response
    # y(t) = 1 - exp(-t) * cos(3*t)
    y = 1.0 - np.exp(-t) * np.cos(3 * t)
    
    metrics = compute_transient_metrics(t, y, y_target=1.0)
    
    # Peak should be greater than 1.0 (overshoot)
    assert metrics['Peak'] > 1.0
    assert metrics['Overshoot'] > 0.0
    
    # Steady state should be close to 1.0
    np.testing.assert_almost_equal(metrics['SteadyStateValue'], 1.0, decimal=2)
    assert metrics['SteadyStateError'] < 0.01
    
    # Settling time should be a positive number (it eventually settles within 2%)
    assert metrics['SettlingTime'] > 0.0
    assert metrics['SettlingTime'] < 10.0

def test_compute_bandwidth():
    # Simple first order low-pass filter: 1 / (s + 1)
    # Bandwidth (-3dB) is exactly 1 rad/s
    T_tf = ct.tf([1], [1, 1])
    bw = compute_bandwidth(T_tf)
    
    np.testing.assert_almost_equal(bw, 1.0, decimal=2)
