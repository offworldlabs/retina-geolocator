"""Velocity-observability guard in solve_multinode.

Near-colinear receivers make the bistatic Doppler Jacobian rows near-parallel,
so velocity is unobservable and the bounded solve rails to the ±v_max corner
(the 825 kt artifact). The guard detects this via the smallest singular value of
the geometry matrix together with a railed speed: it zeroes the velocity and
flags velocity_observable=False while keeping the still-valid position. A
well-conditioned ring must keep its solved velocity and report True.
"""

import math

import pytest

from retina_geolocator.multinode_solver import _lla_to_enu_km, solve_multinode
from retina_geolocator.bistatic_models import bistatic_delay, bistatic_doppler


def _synthetic_input(node_configs, target_lat, target_lon, target_alt_km,
                     vel_east=0.0, vel_north=0.0, vel_up=0.0):
    ref_lat, ref_lon = target_lat, target_lon
    target_enu = _lla_to_enu_km(target_lat, target_lon, target_alt_km * 1000,
                                ref_lat, ref_lon, 0.0)
    measurements = []
    for nid, cfg in node_configs.items():
        rx_enu = _lla_to_enu_km(cfg["rx_lat"], cfg["rx_lon"],
                                cfg["rx_alt_ft"] * 0.3048, ref_lat, ref_lon, 0.0)
        tx_enu = _lla_to_enu_km(cfg["tx_lat"], cfg["tx_lon"],
                                cfg["tx_alt_ft"] * 0.3048, ref_lat, ref_lon, 0.0)
        fc = cfg.get("fc_hz", 100e6)
        measurements.append({
            "node_id": nid,
            "delay_us": bistatic_delay(target_enu, tx_enu, rx_enu),
            "doppler_hz": bistatic_doppler(target_enu, (vel_east, vel_north, vel_up),
                                           tx_enu, rx_enu, fc),
            "snr": 15.0,
        })
    return {
        "initial_guess": {"lat": target_lat + 0.01, "lon": target_lon + 0.01,
                          "alt_km": target_alt_km},
        "measurements": measurements,
        "n_nodes": len(node_configs),
        "timestamp_ms": 1700000000000,
    }


def _ring_configs(core_lat=32.8968, core_lon=-97.0380, radius_km=18.0,
                  bearings=(0.0, 120.0, 240.0)):
    R = 6371.0
    configs = {}
    for i, brg in enumerate(bearings):
        br = math.radians(brg)
        dlat = (radius_km * math.cos(br)) / R
        dlon = (radius_km * math.sin(br)) / (R * math.cos(math.radians(core_lat)))
        configs[f"ring-{i}"] = {
            "rx_lat": core_lat + math.degrees(dlat),
            "rx_lon": core_lon + math.degrees(dlon),
            "rx_alt_ft": 300,
            "tx_lat": 32.7806, "tx_lon": -96.8006, "tx_alt_ft": 1600,
            "fc_hz": 195e6,
        }
    return configs


_COLINEAR_CONFIGS = {
    "node_a": {"rx_lat": 40.7128, "rx_lon": -74.0060, "rx_alt_ft": 100,
               "tx_lat": 40.78, "tx_lon": -73.95, "tx_alt_ft": 500, "fc_hz": 100e6},
    "node_b": {"rx_lat": 40.7138, "rx_lon": -74.0050, "rx_alt_ft": 110,
               "tx_lat": 40.781, "tx_lon": -73.949, "tx_alt_ft": 500, "fc_hz": 100e6},
}


class TestKeyAlwaysPresent:
    def test_well_conditioned_result_has_observable_key(self):
        s_in = _synthetic_input(_ring_configs(), 32.8968, -97.0380, 8.0,
                                vel_east=150.0, vel_north=80.0)
        result = solve_multinode(s_in, _ring_configs())
        assert result is not None
        assert "velocity_observable" in result
        assert math.isfinite(result["vel_east"])
        assert math.isfinite(result["vel_north"])

    def test_colinear_result_has_observable_key(self):
        s_in = _synthetic_input(_COLINEAR_CONFIGS, 40.73, -73.95, 8.0)
        result = solve_multinode(s_in, _COLINEAR_CONFIGS)
        assert result is not None
        assert "velocity_observable" in result


class TestGuardFires:
    def test_railed_unobservable_velocity_is_zeroed(self):
        s_in = _synthetic_input(_COLINEAR_CONFIGS, 40.73, -73.95, 8.0,
                                vel_east=2000.0, vel_north=2000.0)
        result = solve_multinode(s_in, _COLINEAR_CONFIGS,
                                 sigma_min_floor=1e9, v_max_ms=30.0)
        assert result is not None
        assert result["velocity_observable"] is False
        assert result["vel_east"] == 0.0
        assert result["vel_north"] == 0.0
        assert result["vel_up"] == 0.0

    def test_position_kept_when_velocity_dropped(self):
        s_in = _synthetic_input(_COLINEAR_CONFIGS, 40.73, -73.95, 8.0,
                                vel_east=2000.0, vel_north=2000.0)
        result = solve_multinode(s_in, _COLINEAR_CONFIGS,
                                 sigma_min_floor=1e9, v_max_ms=30.0)
        assert result["velocity_observable"] is False
        assert abs(result["lat"] - 40.73) < 0.1
        assert abs(result["lon"] - (-73.95)) < 0.1


class TestGuardDoesNotOverFire:
    def test_well_conditioned_ring_keeps_velocity(self):
        s_in = _synthetic_input(_ring_configs(), 32.8968, -97.0380, 8.0,
                                vel_east=150.0, vel_north=80.0)
        result = solve_multinode(s_in, _ring_configs())
        assert result is not None
        assert result["velocity_observable"] is True
        speed = math.hypot(result["vel_east"], result["vel_north"])
        assert speed > 100.0

    def test_default_params_do_not_zero_modest_velocity(self):
        s_in = _synthetic_input(_COLINEAR_CONFIGS, 40.73, -73.95, 8.0,
                                vel_east=50.0, vel_north=30.0)
        result = solve_multinode(s_in, _COLINEAR_CONFIGS)
        assert result is not None
        assert result["velocity_observable"] is True
        assert result["vel_east"] != 0.0 or result["vel_north"] != 0.0
