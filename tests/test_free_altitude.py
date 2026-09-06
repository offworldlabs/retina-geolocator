"""Free-altitude solve mode and the multi-start helper.

The pinned solve can only be as accurate as the altitude its caller hands it,
and the backend's caller searches a ladder of fixed layers 2 km apart — so its
altitude is systematically up to 1 km wrong, and that error lands in
``rms_delay``, which is the reject gate.  These scenes are noise-free: every
microsecond of residual a pinned solve reports here is the altitude error and
nothing else, which is exactly what makes the comparison legible.
"""

import math

import pytest

from retina_geolocator.bistatic_models import bistatic_delay, bistatic_doppler
from retina_geolocator.multinode_solver import (
    _Z_BOUND_MAX_KM,
    _lla_to_enu_km,
    solve_multinode,
    solve_multinode_multistart,
)

# The deployment's metro (Greenville SC), so the geometry these scenes are
# built on has the ellipsoid curvature and the ENU reference the live fleet
# does rather than an arbitrary flat patch.
REF_LAT, REF_LON = 34.85, -82.40

# The altitude layers backend/services/tasks/solver.py sweeps.  Copied rather
# than imported — this library must not depend on the backend — and the point
# of these tests is what happens when the truth falls BETWEEN them.
SWEEP_LAYERS_KM = [1.5, 3.0, 5.0, 7.0, 9.0, 11.0]


def _node_ring(n, radius_km=28.0, seed_deg=17.0):
    """``n`` receivers on a ring around the reference, each with its own TX.

    Spread over three bands (98/195/599 MHz) like the real fleet, and with the
    TX offset from its RX by a rotating bearing so no two nodes share a
    baseline — a degenerate layout would make altitude unobservable for
    reasons that have nothing to do with the mode under test.
    """
    cos_lat = math.cos(math.radians(REF_LAT))
    cfgs = {}
    for i in range(n):
        a = math.radians(seed_deg + 360.0 * i / n)
        rx_lat = REF_LAT + (radius_km * math.cos(a)) / 111.0
        rx_lon = REF_LON + (radius_km * math.sin(a)) / (111.0 * cos_lat)
        b = a + 0.9
        cfgs[f"node_{i}"] = {
            "rx_lat": rx_lat,
            "rx_lon": rx_lon,
            "rx_alt_ft": 900 + 40 * i,
            "tx_lat": rx_lat + (34.0 * math.cos(b)) / 111.0,
            "tx_lon": rx_lon + (34.0 * math.sin(b)) / (111.0 * cos_lat),
            "tx_alt_ft": 1800 + 60 * i,
            "fc_hz": [98.1e6, 195.0e6, 599.0e6][i % 3],
        }
    return cfgs


def _scene(node_configs, lat, lon, alt_km, guess_alt_km, ve=180.0, vn=-120.0, vu=0.0):
    """A noise-free solver input for a target at (lat, lon, alt_km).

    Measurements come from bistatic_delay/bistatic_doppler — the same forward
    model the solver inverts — so a solve at the true altitude has to reach
    rms 0, and any residual left over is attributable.  The horizontal guess
    is deliberately ~1.5 km off so the solve still has to converge.
    """
    target_enu = _lla_to_enu_km(lat, lon, alt_km * 1000, lat, lon, 0.0)
    measurements = []
    for nid, cfg in node_configs.items():
        rx_enu = _lla_to_enu_km(cfg["rx_lat"], cfg["rx_lon"], cfg["rx_alt_ft"] * 0.3048, lat, lon, 0.0)
        tx_enu = _lla_to_enu_km(cfg["tx_lat"], cfg["tx_lon"], cfg["tx_alt_ft"] * 0.3048, lat, lon, 0.0)
        measurements.append(
            {
                "node_id": nid,
                "delay_us": bistatic_delay(target_enu, tx_enu, rx_enu),
                "doppler_hz": bistatic_doppler(target_enu, (ve, vn, vu), tx_enu, rx_enu, cfg["fc_hz"]),
                "snr": 15.0,
            }
        )
    return {
        "initial_guess": {"lat": lat + 0.012, "lon": lon - 0.012, "alt_km": guess_alt_km},
        "initial_velocity": {"vel_east_ms": ve, "vel_north_ms": vn},
        "measurements": measurements,
        "n_nodes": len(node_configs),
        "timestamp_ms": 1700000000000,
    }


def _nearest_layer_km(alt_km):
    return min(SWEEP_LAYERS_KM, key=lambda layer: abs(layer - alt_km))


def _starts_km(alt_km):
    """The backend's free-mode starts: the nearest layer and its neighbours."""
    i = SWEEP_LAYERS_KM.index(_nearest_layer_km(alt_km))
    lo = max(0, min(i - 1, len(SWEEP_LAYERS_KM) - 3))
    return SWEEP_LAYERS_KM[lo : lo + 3]


def _pos_err_km(result, lat, lon):
    return math.hypot(
        (result["lat"] - lat) * 111.0,
        (result["lon"] - lon) * 111.0 * math.cos(math.radians(lat)),
    )


# True altitude, node count.  Each altitude sits ~0.9 km from its nearest
# sweep layer — the worst case that ladder can produce, and the case the dark
# lane lives in (5-12 km on the commercial envelope).
_SCENES = [(3.9, 4), (4.1, 7), (5.9, 5), (7.9, 6), (9.9, 8), (11.9, 4)]


class TestFreeAltitudeAccuracy:
    @pytest.mark.parametrize("alt_km,n_nodes", _SCENES)
    def test_free_beats_the_nearest_sweep_layer(self, alt_km, n_nodes):
        """Free mode fits noise-free measurements; the nearest layer does not.

        The pinned assertion is the point of the test, not scaffolding: it
        establishes that the residual budget the free solve reclaims is real
        — on clean data, at an altitude the sweep cannot land on, the pinned
        solve already spends more than 0.5 µs of the backend's 3 µs gate.
        """
        node_configs = _node_ring(n_nodes)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        layer_km = _nearest_layer_km(alt_km)

        pinned = solve_multinode(_scene(node_configs, lat, lon, alt_km, layer_km), node_configs)
        assert pinned is not None and pinned["success"]
        assert pinned["altitude_mode"] == "pinned"
        assert pinned["z_saturated"] is False
        assert pinned["rms_delay"] > 0.5
        # The pinned solve reports exactly the layer it was given (to within
        # the ENU/LLA round-trip's few metres), so its altitude error IS the
        # ladder's quantisation and nothing the solve did.
        assert abs(pinned["alt_m"] / 1000.0 - alt_km) == pytest.approx(abs(layer_km - alt_km), abs=0.01)

        free = solve_multinode(_scene(node_configs, lat, lon, alt_km, layer_km), node_configs, free_altitude=True)
        assert free is not None and free["success"]
        assert free["altitude_mode"] == "free"
        assert free["z_saturated"] is False
        assert free["rms_delay"] <= 0.05
        assert abs(free["alt_m"] / 1000.0 - alt_km) <= 0.2
        assert _pos_err_km(free, lat, lon) <= 0.2

    def test_free_keeps_the_vz_bound(self):
        """Altitude becoming an unknown does not loosen the vertical rate."""
        node_configs = _node_ring(6)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        free = solve_multinode(_scene(node_configs, lat, lon, 7.9, 7.0, vu=120.0), node_configs, free_altitude=True)
        assert free is not None
        assert abs(free["vel_up"]) <= 20.0
        assert free["vz_saturated"] is True

    def test_n2_falls_back_to_pinned(self):
        """At n=2 altitude is unobservable, so the pin applies regardless."""
        node_configs = _node_ring(2)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        result = solve_multinode(_scene(node_configs, lat, lon, 7.9, 7.0), node_configs, free_altitude=True)
        assert result is not None and result["success"]
        assert result["altitude_mode"] == "pinned"
        assert result["z_saturated"] is False
        # The pinned altitude, not a solved one.
        assert result["alt_m"] / 1000.0 == pytest.approx(7.0, abs=0.05)

    def test_z_saturation_is_flagged_above_the_bound(self):
        """A target outside the z bounds reports a bound, and says so."""
        node_configs = _node_ring(6)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        result = solve_multinode(
            _scene(node_configs, lat, lon, _Z_BOUND_MAX_KM + 6.0, 11.0),
            node_configs,
            free_altitude=True,
        )
        assert result is not None and result["success"]
        assert result["altitude_mode"] == "free"
        assert result["z_saturated"] is True
        assert result["alt_m"] / 1000.0 == pytest.approx(_Z_BOUND_MAX_KM, abs=0.05)

    def test_pinned_is_the_default(self):
        """The unflagged call is byte-for-byte the solve every caller had."""
        node_configs = _node_ring(5)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        s_in = _scene(node_configs, lat, lon, 7.9, 7.0)
        default = solve_multinode(s_in, node_configs)
        explicit = solve_multinode(s_in, node_configs, free_altitude=False)
        assert default["altitude_mode"] == "pinned"
        assert default["lat"] == explicit["lat"]
        assert default["lon"] == explicit["lon"]
        assert default["alt_m"] == explicit["alt_m"]
        assert default["rms_delay"] == explicit["rms_delay"]


class TestMultistart:
    def test_returns_the_best_start(self):
        """The winner is the lowest-rms start, and every start is reported."""
        node_configs = _node_ring(6)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        alt_km = 7.9
        starts = _starts_km(alt_km)
        result = solve_multinode_multistart(
            _scene(node_configs, lat, lon, alt_km, _nearest_layer_km(alt_km)),
            node_configs,
            starts,
        )
        assert result is not None and result["success"]
        assert result["altitude_mode"] == "free"
        assert result["alt_starts_km"] == starts
        assert len(result["rms_by_start"]) == len(starts)
        fitted = [r for r in result["rms_by_start"] if r is not None]
        assert fitted, "no start produced a solve"
        assert result["rms_delay"] == pytest.approx(min(fitted), abs=1e-12)
        assert result["rms_delay"] <= 0.05
        assert abs(result["alt_m"] / 1000.0 - alt_km) <= 0.2

    def test_recovers_from_a_start_two_layers_away(self):
        """A start 4 km wrong still lands on the truth, which is the point of
        keeping several: the free solve removes the ladder's quantisation, not
        the LM's locality."""
        node_configs = _node_ring(6)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        alt_km = 11.9
        result = solve_multinode_multistart(
            _scene(node_configs, lat, lon, alt_km, 11.0), node_configs, [5.0, 9.0, 11.0]
        )
        assert result is not None
        assert abs(result["alt_m"] / 1000.0 - alt_km) <= 0.2
        assert result["rms_delay"] <= 0.05

    def test_empty_starts_uses_the_input_guess(self):
        """No starts is not a reason to return nothing."""
        node_configs = _node_ring(5)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        result = solve_multinode_multistart(_scene(node_configs, lat, lon, 7.9, 7.0), node_configs, [])
        assert result is not None
        assert result["alt_starts_km"] == [7.0]
        assert abs(result["alt_m"] / 1000.0 - 7.9) <= 0.2

    def test_no_solve_returns_none(self):
        """Every start failing is reported as a failure, not as a bad fit."""
        s_in = {
            "initial_guess": {"lat": REF_LAT, "lon": REF_LON, "alt_km": 7.0},
            "measurements": [
                {"node_id": "ghost_a", "delay_us": 50, "doppler_hz": 10, "snr": 15},
                {"node_id": "ghost_b", "delay_us": 60, "doppler_hz": -5, "snr": 12},
                {"node_id": "ghost_c", "delay_us": 70, "doppler_hz": 3, "snr": 12},
            ],
            "n_nodes": 3,
            "timestamp_ms": 0,
        }
        assert solve_multinode_multistart(s_in, {}, [5.0, 7.0, 9.0]) is None

    def test_pinned_multistart_is_the_sweep(self):
        """free_altitude=False makes this the altitude sweep it replaces, so a
        caller that wants the old behaviour on one pool call can have it."""
        node_configs = _node_ring(6)
        lat, lon = REF_LAT + 0.05, REF_LON - 0.07
        result = solve_multinode_multistart(
            _scene(node_configs, lat, lon, 9.0, 9.0),
            node_configs,
            [5.0, 7.0, 9.0],
            free_altitude=False,
        )
        assert result is not None
        assert result["altitude_mode"] == "pinned"
        # 9.0 km is a layer, so the sweep can land exactly on the truth.
        assert result["alt_m"] / 1000.0 == pytest.approx(9.0, abs=0.05)
        assert result["rms_delay"] <= 0.05
