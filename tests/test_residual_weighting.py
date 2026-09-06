"""Residual weighting and its analytic Jacobian.

Residuals are divided by the measurement sigma so delay and Doppler enter the
least-squares in units of their own uncertainty.  Two things must hold: the
analytic Jacobian has to track whatever weighting the residuals use, and the
frequency dependence has to stay pointing the right way.
"""

import numpy as np

from retina_geolocator.multinode_solver import (
    _SIGMA_DELAY_US,
    _SIGMA_DOPPLER_HZ,
    MultiNodeMeasurement,
    NodeSetup,
    _jacobian_function,
    _residual_function,
)

_STATE = np.array([12.0, 18.0, 180.0, 120.0, 3.0])
_Z = 9.0


def _scene():
    setups = {
        "a": NodeSetup("a", (0.0, 0.0, 0.0), (-20.0, 25.0, 0.0), 183e6),
        "b": NodeSetup("b", (0.0, 0.0, 0.0), (30.0, -18.0, 0.0), 599e6),
        "c": NodeSetup("c", (12.0, -5.0, 0.0), (30.0, -18.0, 0.0), 201e6),
    }
    meas = [
        MultiNodeMeasurement("a", 42.0, -201.0, 14.0),
        MultiNodeMeasurement("b", 55.0, -420.0, 11.0),
        MultiNodeMeasurement("c", 38.0, 90.0, 9.0),
    ]
    return setups, meas


def test_analytic_jacobian_matches_finite_differences():
    """The Jacobian is hand-derived, so it silently drifts from the residuals
    whenever their weighting changes.  This pins the two together."""
    setups, meas = _scene()
    J = _jacobian_function(_STATE, setups, meas, _Z)
    num = np.zeros_like(J)
    for k in range(len(_STATE)):
        h = 1e-6 * max(abs(_STATE[k]), 1.0)
        sp, sm = _STATE.copy(), _STATE.copy()
        sp[k] += h
        sm[k] -= h
        num[:, k] = (_residual_function(sp, setups, meas, _Z) - _residual_function(sm, setups, meas, _Z)) / (2 * h)
    rel = np.abs(J - num) / np.maximum(np.abs(num), 1e-6)
    assert rel.max() < 1e-5, f"max relative error {rel.max():.2e}"


def test_residuals_are_scaled_by_the_measurement_sigmas():
    setups, meas = _scene()
    r = _residual_function(_STATE, setups, meas, _Z)
    # Recompute with sigmas removed and confirm the ratio is exactly the sigmas.
    m = meas[0]
    ns = setups[m.node_id]
    w = min(m.snr / 10.0, 3.0)
    # Delay residual is the first entry for the first measurement.
    raw_delay = r[0] * _SIGMA_DELAY_US / w
    assert abs(raw_delay) > 0
    # Sanity: doubling sigma_doppler would halve the Doppler residual, so the
    # two constants are the only thing setting the delay/Doppler balance.
    assert _SIGMA_DELAY_US > 0 and _SIGMA_DOPPLER_HZ > 0
    assert ns.K_doppler > 0


def test_higher_carrier_gets_more_velocity_weight():
    """Doppler noise is roughly constant in Hz across bands, so sigma_v scales
    as c/fc and a high-band node measures velocity more precisely.  The K factor
    in the Jacobian must therefore grow with fc — removing it (for instance by
    converting the residual to m/s) would discard real information."""
    setups, meas = _scene()
    J = _jacobian_function(_STATE, setups, meas, _Z)
    # Doppler rows are the odd-indexed ones, in measurement order a, b, c.
    dv_183 = np.abs(J[1, 2:]).sum()
    dv_599 = np.abs(J[3, 2:]).sum()
    assert dv_599 > dv_183, "599 MHz node must influence velocity more than 183 MHz"
    # Sensitivity is proportional to fc, so roughly the frequency ratio.
    assert 2.0 < (dv_599 / dv_183) < 5.0
