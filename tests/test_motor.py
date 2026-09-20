import numpy as np
import pytest

from dc_motor_sim.config import DCMotorParameters
from dc_motor_sim.motor import DCMotorModel


def test_invalid_parameters():
    with pytest.raises(ValueError, match="Inertia J must be strictly positive."):
        DCMotorParameters(J=-0.01)
        
    with pytest.raises(ValueError, match="Friction b must be non-negative."):
        DCMotorParameters(b=-0.1)

def test_motor_model_creation():
    params = DCMotorParameters(J=0.01, b=0.1, K=0.01, R=1.0, L=0.5)
    model = DCMotorModel(params)
    
    # Check open loop transfer function (Speed)
    assert model.P_speed.num[0][0][0] == params.K
    
    # Check state space dimensions
    assert model.A.shape == (3, 3)
    assert model.B.shape == (3, 1)
    
def test_derivatives():
    params = DCMotorParameters(J=0.01, b=0.1, K=0.01, R=1.0, L=0.5)
    model = DCMotorModel(params)
    
    state = np.array([0.0, 1.0, 2.0]) # theta, omega, i_a
    v_applied = 5.0
    
    derivs = model.derivatives(0.0, state, v_applied, tau_load=0.0)
    
    # dtheta_dt = omega
    assert derivs[0] == 1.0
    
    # domega_dt = (K * i_a - b * omega - tau_load) / J
    expected_domega = (0.01 * 2.0 - 0.1 * 1.0) / 0.01
    np.testing.assert_almost_equal(derivs[1], expected_domega)
    
    # di_a_dt = (v_applied - R * i_a - K * omega) / L
    expected_di = (5.0 - 1.0 * 2.0 - 0.01 * 1.0) / 0.5
    np.testing.assert_almost_equal(derivs[2], expected_di)


def test_get_poles_and_zeros():
    params = DCMotorParameters(J=0.01, b=0.1, K=0.01, R=1.0, L=0.5)
    model = DCMotorModel(params)

    res = model.get_poles_and_zeros()

    assert "poles" in res
    assert "zeros" in res
    assert "dc_gain" in res
    assert "wn" in res
    assert "zeta" in res

    # Mathematical expectations for J=0.01, b=0.1, K=0.01, R=1.0, L=0.5
    # den = [J*L, J*R + b*L, b*R + K**2] = [0.005, 0.06, 0.1001]
    expected_wn = np.sqrt(0.1001 / 0.005)
    expected_zeta = (0.06 / 0.005) / (2.0 * expected_wn)
    expected_dc_gain = 0.01 / 0.1001

    np.testing.assert_almost_equal(res["wn"], expected_wn)
    np.testing.assert_almost_equal(res["zeta"], expected_zeta)
    np.testing.assert_almost_equal(res["dc_gain"], expected_dc_gain)

    assert len(res["poles"]) == 2
    assert len(res["zeros"]) == 0

    expected_poles = np.roots([0.005, 0.06, 0.1001])
    np.testing.assert_almost_equal(np.sort(res["poles"]), np.sort(expected_poles))
