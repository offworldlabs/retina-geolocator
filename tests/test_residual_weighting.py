"""Residual weighting and its analytic Jacobian.

Residuals are divided by the measurement sigma so delay and Doppler enter the
least-squares in units of their own uncertainty.  Two things must hold: the
analytic Jacobian has to track whatever weighting the residuals use, and the
frequency dependence has to stay pointing the right way.
"""

import json

import numpy as np

from retina_geolocator.multinode_solver import (
    MultiNodeMeasurement,
    NodeSetup,
    _jacobian_function,
    _jacobian_function_free,
    _residual_function,
    _residual_function_free,
    snr_weight,
    solve_multinode,
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


def _fd_max_relative_error(residual, jacobian, state):
    J = jacobian(state)
    num = np.zeros_like(J)
    for k in range(len(state)):
        h = 1e-6 * max(abs(state[k]), 1.0)
        sp, sm = state.copy(), state.copy()
        sp[k] += h
        sm[k] -= h
        num[:, k] = (residual(sp) - residual(sm)) / (2 * h)
    return (np.abs(J - num) / np.maximum(np.abs(num), 1e-6)).max()


def test_analytic_jacobian_matches_finite_differences():
    """The Jacobian is hand-derived, so it silently drifts from the residuals
    whenever their weighting changes.  This pins the two together, for the
    pinned and the free-altitude solve, with one measurement carrying its own
    weight and the others falling back to their SNR."""
    setups, meas = _scene()
    meas[1].weight = 0.4
    pinned = _fd_max_relative_error(
        lambda s: _residual_function(s, setups, meas, _Z),
        lambda s: _jacobian_function(s, setups, meas, _Z),
        _STATE,
    )
    free = _fd_max_relative_error(
        lambda s: _residual_function_free(s, setups, meas),
        lambda s: _jacobian_function_free(s, setups, meas),
        np.insert(_STATE, 2, _Z),
    )
    assert pinned < 1e-5, f"pinned max relative error {pinned:.2e}"
    assert free < 1e-5, f"free max relative error {free:.2e}"


def test_residuals_are_scaled_by_the_measurement_weight():
    setups, meas = _scene()
    meas[0].weight = 1.5
    state = np.insert(_STATE, 2, _Z)
    light = _residual_function_free(state, setups, meas)
    meas[0].weight = 3.0
    heavy = _residual_function_free(state, setups, meas)
    assert np.array_equal(heavy[:2], 2 * light[:2])
    assert np.array_equal(heavy[2:], light[2:])


def test_measurement_weight_reaches_the_solve():
    """A weight equal to the SNR fallback changes nothing; any other does."""
    configs = {
        nid: {"rx_lat": 34.85, "rx_lon": -82.40 + dx, "tx_lat": 34.95, "tx_lon": -82.10, "fc_hz": fc}
        for nid, dx, fc in [("a", -0.20, 183e6), ("b", 0.10, 599e6), ("c", 0.25, 201e6)]
    }
    meas = [
        {"node_id": "a", "delay_us": 42.0, "doppler_hz": -201.0, "snr": 14.0},
        {"node_id": "b", "delay_us": 55.0, "doppler_hz": -420.0, "snr": 11.0},
        {"node_id": "c", "delay_us": 38.0, "doppler_hz": 90.0, "snr": 9.0},
    ]

    def solve(measurements):
        guess = {"lat": 34.9, "lon": -82.3, "alt_km": 7.0}
        out = solve_multinode({"initial_guess": guess, "measurements": measurements}, configs)
        return json.dumps(out, sort_keys=True, default=str)

    plain = solve(meas)
    assert solve([{**m, "weight": snr_weight(m["snr"])} for m in meas]) == plain
    assert solve([{**meas[0], "weight": 0.2}, *meas[1:]]) != plain


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
