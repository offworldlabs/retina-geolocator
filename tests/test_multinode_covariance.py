"""Unit tests for solve_multinode's per-solve position covariance.

cov_en_km2 / pos_sigma_km feed the backend's Kalman track smoother, which
weights each solve's contribution by how precise it claims to be.  These
tests check the two keys are always present, that the reported covariance is
a legitimate 2x2 PSD matrix, that it shrinks as more independent nodes are
added (the entire point of reporting it), and that the result dict survives
the pickle/JSON round trip it takes crossing the ProcessPoolExecutor and API
boundaries.
"""

import json
import math
import pickle

import numpy as np
import pytest

from retina_geolocator.multinode_solver import _lla_to_enu_km, solve_multinode
from retina_geolocator.bistatic_models import bistatic_delay, bistatic_doppler


def _make_synthetic_input(node_configs, target_lat, target_lon, target_alt_km,
                           vel_east=0.0, vel_north=0.0, vel_up=0.0):
    """Build a solver_input from a known target state, mirroring
    tests/test_solver.py's helper of the same name."""
    ref_lat = target_lat
    ref_lon = target_lon

    target_enu = _lla_to_enu_km(target_lat, target_lon, target_alt_km * 1000,
                                 ref_lat, ref_lon, 0.0)
    measurements = []
    for nid, cfg in node_configs.items():
        rx_enu = _lla_to_enu_km(cfg["rx_lat"], cfg["rx_lon"],
                                 cfg["rx_alt_ft"] * 0.3048,
                                 ref_lat, ref_lon, 0.0)
        tx_enu = _lla_to_enu_km(cfg["tx_lat"], cfg["tx_lon"],
                                 cfg["tx_alt_ft"] * 0.3048,
                                 ref_lat, ref_lon, 0.0)
        fc = cfg.get("fc_hz", 100e6)

        delay = bistatic_delay(target_enu, tx_enu, rx_enu)
        doppler = bistatic_doppler(target_enu, (vel_east, vel_north, vel_up),
                                    tx_enu, rx_enu, fc)
        measurements.append({
            "node_id": nid,
            "delay_us": delay,
            "doppler_hz": doppler,
            "snr": 15.0,
        })

    return {
        "initial_guess": {
            "lat": target_lat + 0.01,  # slightly off, as in test_solver.py
            "lon": target_lon + 0.01,
            "alt_km": target_alt_km,
        },
        "measurements": measurements,
        "n_nodes": len(node_configs),
        "timestamp_ms": 1700000000000,
    }


# Four nodes spread around the target with distinct bearings, so that going
# from a 2-node subset to the full set is a genuine GDOP improvement rather
# than a near-duplicate baseline.
_FOUR_NODE_CONFIGS = {
    "node_a": {
        "rx_lat": 40.7128, "rx_lon": -74.0060, "rx_alt_ft": 100,
        "tx_lat": 40.78, "tx_lon": -73.95, "tx_alt_ft": 500,
        "fc_hz": 100e6,
    },
    "node_b": {
        "rx_lat": 40.75, "rx_lon": -73.90, "rx_alt_ft": 150,
        "tx_lat": 40.70, "tx_lon": -73.85, "tx_alt_ft": 400,
        "fc_hz": 100e6,
    },
    "node_c": {
        "rx_lat": 40.65, "rx_lon": -73.98, "rx_alt_ft": 120,
        "tx_lat": 40.62, "tx_lon": -73.88, "tx_alt_ft": 600,
        "fc_hz": 100e6,
    },
    "node_d": {
        "rx_lat": 40.80, "rx_lon": -73.75, "rx_alt_ft": 130,
        "tx_lat": 40.85, "tx_lon": -73.65, "tx_alt_ft": 550,
        "fc_hz": 100e6,
    },
}

_TARGET = (40.72, -73.93, 8.0)  # lat, lon, alt_km
_VEL_EAST, _VEL_NORTH = 150.0, 80.0


class TestCovarianceKeys:
    def test_covariance_keys_present_and_valid(self):
        """A >=3-node solve reports a well-formed 2x2 PSD east/north
        covariance and a finite, positive scalar sigma."""
        s_in = _make_synthetic_input(
            _FOUR_NODE_CONFIGS, *_TARGET, vel_east=_VEL_EAST, vel_north=_VEL_NORTH,
        )
        result = solve_multinode(s_in, _FOUR_NODE_CONFIGS)

        assert result is not None
        assert result["success"] is True

        cov = result["cov_en_km2"]
        assert cov is not None
        assert isinstance(cov, list) and len(cov) == 2
        assert all(isinstance(row, list) and len(row) == 2 for row in cov)

        exx, exy = cov[0][0], cov[0][1]
        eyx, eyy = cov[1][0], cov[1][1]
        for v in (exx, exy, eyx, eyy):
            assert isinstance(v, float)
            assert math.isfinite(v)

        # Symmetric by construction (same value written to both off-diagonal
        # slots), and the diagonal is a variance so it must be positive.
        assert exy == eyx
        assert exx > 0
        assert eyy > 0

        pos_sigma_km = result["pos_sigma_km"]
        assert isinstance(pos_sigma_km, float)
        assert math.isfinite(pos_sigma_km)
        assert pos_sigma_km > 0
        assert pos_sigma_km == pytest.approx(math.sqrt(0.5 * (exx + eyy)))

        eigvals = np.linalg.eigvalsh(np.array(cov))
        assert np.all(eigvals >= -1e-12)

    def test_more_nodes_shrink_sigma(self):
        """The same target solved with only 2 of the 4 available nodes should
        report a strictly larger position sigma than solving with all 4 —
        more independent bistatic baselines can only add information."""
        two_node_configs = {
            "node_a": _FOUR_NODE_CONFIGS["node_a"],
            "node_b": _FOUR_NODE_CONFIGS["node_b"],
        }

        s_in_2 = _make_synthetic_input(
            two_node_configs, *_TARGET, vel_east=_VEL_EAST, vel_north=_VEL_NORTH,
        )
        result_2 = solve_multinode(s_in_2, two_node_configs)

        s_in_4 = _make_synthetic_input(
            _FOUR_NODE_CONFIGS, *_TARGET, vel_east=_VEL_EAST, vel_north=_VEL_NORTH,
        )
        result_4 = solve_multinode(s_in_4, _FOUR_NODE_CONFIGS)

        assert result_2 is not None and result_2["success"] is True
        assert result_4 is not None and result_4["success"] is True
        assert result_2["pos_sigma_km"] is not None
        assert result_4["pos_sigma_km"] is not None

        assert result_4["pos_sigma_km"] < result_2["pos_sigma_km"]

    def test_result_pickles_and_serializes(self):
        """The return dict crosses a ProcessPoolExecutor boundary in
        production, so it must survive both pickle and JSON round trips —
        cov_en_km2 has to be a plain nested list of floats, not a numpy
        array, for exactly this reason."""
        s_in = _make_synthetic_input(
            _FOUR_NODE_CONFIGS, *_TARGET, vel_east=_VEL_EAST, vel_north=_VEL_NORTH,
        )
        result = solve_multinode(s_in, _FOUR_NODE_CONFIGS)
        assert result is not None

        pickled = pickle.dumps(result)
        restored = pickle.loads(pickled)
        assert restored["cov_en_km2"] == result["cov_en_km2"]
        assert restored["pos_sigma_km"] == result["pos_sigma_km"]

        serialized = json.dumps(result)
        deserialized = json.loads(serialized)
        assert deserialized["cov_en_km2"] == result["cov_en_km2"]
        assert deserialized["pos_sigma_km"] == pytest.approx(result["pos_sigma_km"])

    def test_sigma_none_tolerated(self):
        """Consumers can rely on both keys existing in every return dict from
        a successful solve, even though either can be None if the covariance
        computation fails — check presence, not just truthiness."""
        two_node_configs = {
            "node_a": _FOUR_NODE_CONFIGS["node_a"],
            "node_b": _FOUR_NODE_CONFIGS["node_b"],
        }
        s_in = _make_synthetic_input(two_node_configs, *_TARGET)
        result = solve_multinode(s_in, two_node_configs)

        assert result is not None
        assert "cov_en_km2" in result
        assert "pos_sigma_km" in result
