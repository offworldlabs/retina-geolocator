"""Pairwise-intersection consensus: the hypothesis stage ahead of the LM solve.

Synthetic geometry throughout: node_cfgs (rx/tx lat/lon/alt) plus delay_us
computed from a chosen truth position via the exact same ENU bistatic
differential the module itself uses (_lla_to_enu_km + math.dist) — these
tests pin the geometry, not the implementation, so they'd catch a real
regression in the ray/bearing bisection just as readily as a refactor bug.
"""

import math
import pickle

import pytest

from retina_geolocator.consensus import (
    _cluster_candidates,
    _cluster_score,
    _differential_at,
    _locus_point,
    _pair_candidates,
    select_consensus,
    solve_consensus,
)
from retina_geolocator.constants import C_KM_US
from retina_geolocator.multinode_solver import _lla_to_enu_km

# Dual-illuminator site ("a"/"b" share an RX, as a real deployment does) plus
# two more separately-sited nodes, spread around the truth position below —
# same scale/neighbourhood as test_constant_velocity_fit.py's known-good
# geometry.
_NODE_CFGS = {
    "a": {
        "rx_lat": 34.85,
        "rx_lon": -82.40,
        "rx_alt_ft": 1000,
        "tx_lat": 34.9412,
        "tx_lon": -82.4103,
        "tx_alt_ft": 2000,
        "fc_hz": 183e6,
    },
    "b": {
        "rx_lat": 34.85,
        "rx_lon": -82.40,
        "rx_alt_ft": 1000,
        "tx_lat": 34.9701,
        "tx_lon": -81.9484,
        "tx_alt_ft": 800,
        "fc_hz": 195e6,
    },
    "c": {
        "rx_lat": 34.70,
        "rx_lon": -82.55,
        "rx_alt_ft": 1100,
        "tx_lat": 34.75,
        "tx_lon": -82.20,
        "tx_alt_ft": 1400,
        "fc_hz": 199e6,
    },
    "d": {
        "rx_lat": 35.02,
        "rx_lon": -82.15,
        "rx_alt_ft": 950,
        "tx_lat": 34.96,
        "tx_lon": -82.55,
        "tx_alt_ft": 1300,
        "fc_hz": 201e6,
    },
}
TRUTH = (34.88, -82.35, 7.0)  # lat, lon, alt_km


def _haversine_km(lat1, lon1, lat2, lon2):
    r = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def _measured_delay_us(cfg, lat, lon, alt_km):
    """Bistatic delay (us) for a target at (lat, lon, alt_km), via the exact
    ENU construction consensus.py uses internally — geometry ground truth,
    independent of the module's own pipeline."""
    rx_enu = _lla_to_enu_km(cfg["rx_lat"], cfg["rx_lon"], (cfg.get("rx_alt_ft") or 0) * 0.3048, lat, lon, 0.0)
    tx_enu = _lla_to_enu_km(cfg["tx_lat"], cfg["tx_lon"], (cfg.get("tx_alt_ft") or 0) * 0.3048, lat, lon, 0.0)
    tgt_enu = _lla_to_enu_km(lat, lon, alt_km * 1000.0, lat, lon, 0.0)
    baseline = math.dist(rx_enu, tx_enu)
    diff_km = math.dist(tgt_enu, tx_enu) + math.dist(tgt_enu, rx_enu) - baseline
    return diff_km / C_KM_US


def _node_geom(cfg, ref, truth):
    """The internal node-geometry dict _locus_point/_differential_at/
    _pair_candidates expect, built directly for unit-testing those helpers
    without going through the public entry points."""
    ref_lat, ref_lon = ref[0], ref[1]
    rx_enu = _lla_to_enu_km(cfg["rx_lat"], cfg["rx_lon"], (cfg.get("rx_alt_ft") or 0) * 0.3048, ref_lat, ref_lon, 0.0)
    tx_enu = _lla_to_enu_km(cfg["tx_lat"], cfg["tx_lon"], (cfg.get("tx_alt_ft") or 0) * 0.3048, ref_lat, ref_lon, 0.0)
    diff_km = _measured_delay_us(cfg, *truth) * C_KM_US
    return {"rx_enu": rx_enu, "tx_enu": tx_enu, "baseline_km": math.dist(rx_enu, tx_enu), "diff_km": diff_km}


def _meas(nid, truth, cfgs=_NODE_CFGS, snr=15.0):
    lat, lon, alt_km = truth
    return {"node_id": nid, "delay_us": _measured_delay_us(cfgs[nid], lat, lon, alt_km), "doppler_hz": 0.0, "snr": snr}


def _build_s_in(measurements, guess, **overrides):
    s_in = {
        "initial_guess": {"lat": guess[0], "lon": guess[1], "alt_km": guess[2]},
        "measurements": measurements,
        "n_nodes": len(measurements),
        "timestamp_ms": 1_700_000_000_000,
    }
    s_in.update(overrides)
    return s_in


def _s_in(node_ids, truth=TRUTH, guess=None, cfgs=_NODE_CFGS, **overrides):
    guess = guess or truth
    return _build_s_in([_meas(nid, truth, cfgs) for nid in node_ids], guess, **overrides)


# ── _locus_point ──────────────────────────────────────────────────────────────


class TestLocusPoint:
    def test_bisection_reproduces_target_differential(self):
        node = _node_geom(_NODE_CFGS["a"], ref=TRUTH, truth=TRUTH)
        # Bearing from RX straight at the ENU origin — where truth sits in
        # this reference frame, so this bearing is guaranteed to cross the
        # locus (truth produced the measured diff_km in the first place).
        bearing = math.degrees(math.atan2(-node["rx_enu"][0], -node["rx_enu"][1])) % 360
        ceiling = node["diff_km"] / 2.0 + node["baseline_km"] + 5.0
        pt = _locus_point(node, bearing, ceiling, TRUTH[2])
        assert pt is not None
        assert _differential_at(node, pt) == pytest.approx(node["diff_km"], abs=0.1)

    def test_no_bracket_returns_none(self):
        # Sub-RX point (at the pinned altitude) already exceeds a tiny
        # target differential.
        node = {"rx_enu": (0.0, 0.0, 0.0), "tx_enu": (10.0, 0.0, 0.0), "baseline_km": 10.0, "diff_km": 0.001}
        assert _locus_point(node, 90.0, 20.0, z_km=20.0) is None

    def test_short_ceiling_returns_none(self):
        node = {"rx_enu": (0.0, 0.0, 0.0), "tx_enu": (10.0, 0.0, 0.0), "baseline_km": 10.0, "diff_km": 5.0}
        assert _locus_point(node, 90.0, 0.001, z_km=0.0) is None


# ── _pair_candidates ──────────────────────────────────────────────────────────


class TestPairCandidates:
    def test_clean_pair_yields_candidate_near_truth(self):
        node_a = _node_geom(_NODE_CFGS["a"], ref=TRUTH, truth=TRUTH)
        node_b = _node_geom(_NODE_CFGS["b"], ref=TRUTH, truth=TRUTH)
        candidates = _pair_candidates(node_a, node_b, TRUTH[2])
        assert candidates
        best = min(candidates, key=lambda c: math.hypot(c["e"], c["n"]))
        assert math.hypot(best["e"], best["n"]) < 1.0

    def test_no_crossing_pair_yields_empty(self):
        node_a = _node_geom(_NODE_CFGS["a"], ref=TRUTH, truth=TRUTH)
        node_b = dict(_node_geom(_NODE_CFGS["b"], ref=TRUTH, truth=TRUTH))
        # An unreachable target differential: no point anywhere near A's
        # locus can satisfy it, so B's residual never changes sign.
        node_b["diff_km"] = 10_000.0
        assert _pair_candidates(node_a, node_b, TRUTH[2]) == []


# ── _cluster_candidates / _cluster_score ──────────────────────────────────────


class TestClustering:
    def test_within_radius_merges(self):
        candidates = [
            {"e": 0.0, "n": 0.0, "residual": 0.1, "pair": ("a", "b")},
            {"e": 1.0, "n": 0.0, "residual": 0.2, "pair": ("a", "c")},
        ]
        clusters = _cluster_candidates(candidates)
        assert len(clusters) == 1
        assert clusters[0]["nodes"] == {"a", "b", "c"}
        assert clusters[0]["pairs"] == {("a", "b"), ("a", "c")}

    def test_outside_radius_splits(self):
        candidates = [
            {"e": 0.0, "n": 0.0, "residual": 0.1, "pair": ("a", "b")},
            {"e": 10.0, "n": 0.0, "residual": 0.2, "pair": ("c", "d")},
        ]
        assert len(_cluster_candidates(candidates)) == 2

    def test_score_orders_nodes_then_pairs_then_residual(self):
        fewer_nodes = {"nodes": {"a", "b"}, "pairs": {("a", "b")}, "residuals": [0.01]}
        more_nodes = {"nodes": {"a", "b", "c"}, "pairs": {("a", "b"), ("b", "c")}, "residuals": [5.0]}
        assert _cluster_score(more_nodes) > _cluster_score(fewer_nodes)

        same_nodes_fewer_pairs = {"nodes": {"a", "b", "c"}, "pairs": {("a", "b"), ("b", "c")}, "residuals": [5.0]}
        same_nodes_more_pairs = {
            "nodes": {"a", "b", "c"},
            "pairs": {("a", "b"), ("b", "c"), ("a", "c")},
            "residuals": [5.0],
        }
        assert _cluster_score(same_nodes_more_pairs) > _cluster_score(same_nodes_fewer_pairs)

        worse_residual = {"nodes": {"a", "b"}, "pairs": {("a", "b")}, "residuals": [5.0]}
        better_residual = {"nodes": {"a", "b"}, "pairs": {("a", "b")}, "residuals": [0.1]}
        assert _cluster_score(better_residual) > _cluster_score(worse_residual)


# ── select_consensus (production entry point) ─────────────────────────────────


class TestSelectConsensus:
    def test_four_node_clean_scene_selects_all(self):
        sel = select_consensus(_s_in(["a", "b", "c", "d"]), _NODE_CFGS)
        assert sel is not None
        assert sel["node_ids"] == ["a", "b", "c", "d"]
        assert _haversine_km(sel["lat"], sel["lon"], TRUTH[0], TRUTH[1]) < 5.0
        for key in (
            "node_ids",
            "input_node_ids",
            "lat",
            "lon",
            "alt_km",
            "n_candidates",
            "n_clusters",
            "n_corroborated_clusters",
            "n_pairs",
            "mean_residual_us",
            "centroid_offset_km",
        ):
            assert key in sel

    def test_contaminated_node_excluded(self):
        # "d" is fed a delay from a second, distant aircraft — a-b-c still
        # corroborate each other; d should not be pulled into the winner.
        ghost_truth = (35.10, -82.60, 8.0)
        measurements = [_meas(nid, TRUTH) for nid in ("a", "b", "c")]
        measurements.append(_meas("d", ghost_truth))
        sel = select_consensus(_build_s_in(measurements, TRUTH), _NODE_CFGS)
        assert sel is not None
        assert "d" not in sel["node_ids"]
        assert set(sel["node_ids"]) <= {"a", "b", "c"}

    def test_no_initial_guess_abstains(self):
        s_in = {"measurements": [_meas("a", TRUTH), _meas("b", TRUTH)]}
        assert select_consensus(s_in, _NODE_CFGS) is None

    def test_single_usable_node_abstains(self):
        assert select_consensus(_s_in(["a"]), _NODE_CFGS) is None

    def test_picklable_for_spawn_pool(self):
        restored = pickle.loads(pickle.dumps(select_consensus))
        s_in = _s_in(["a", "b", "c", "d"])
        assert restored(s_in, _NODE_CFGS) == select_consensus(s_in, _NODE_CFGS)


# ── solve_consensus (bench entry point) ────────────────────────────────────────


class TestSolveConsensus:
    def test_result_shape_matches_solve_multinode(self):
        out = solve_consensus(_s_in(["a", "b", "c", "d"]), _NODE_CFGS)
        assert out is not None
        for key in (
            "success",
            "lat",
            "lon",
            "alt_m",
            "vel_east",
            "vel_north",
            "vel_up",
            "rms_delay",
            "rms_doppler",
            "n_nodes",
            "n_measurements",
            "contributing_node_ids",
            "timestamp_ms",
        ):
            assert key in out

    def test_velocity_passthrough_and_no_doppler(self):
        s_in = _s_in(["a", "b", "c", "d"], initial_velocity={"vel_east_ms": 123.0, "vel_north_ms": -45.0})
        out = solve_consensus(s_in, _NODE_CFGS)
        assert out["vel_east"] == pytest.approx(123.0)
        assert out["vel_north"] == pytest.approx(-45.0)
        assert out["rms_doppler"] == 0.0

    def test_n2_returns_candidate_nearest_guess(self):
        out = solve_consensus(_s_in(["a", "b"]), _NODE_CFGS)
        assert out is not None
        assert set(out["contributing_node_ids"]) == {"a", "b"}
        assert _haversine_km(out["lat"], out["lon"], TRUTH[0], TRUTH[1]) < 5.0

    def test_refine_fn_copies_back_but_keeps_consensus_identity(self):
        s_in = _s_in(["a", "b", "c", "d"])

        def fake_refine(s_sub, cfgs):
            return {
                "success": True,
                "lat": 40.0,
                "lon": -90.0,
                "alt_m": 5000.0,
                "vel_east": 1.0,
                "vel_north": 2.0,
                "vel_up": 3.0,
                "rms_delay": 0.01,
                "rms_doppler": 0.5,
                "n_nodes": 99,
                "contributing_node_ids": ["zzz"],
            }

        out = solve_consensus(s_in, _NODE_CFGS, refine_fn=fake_refine)
        assert out["lat"] == 40.0 and out["lon"] == -90.0
        assert out["rms_delay"] == 0.01
        # Refine solved the SAME subset — its own n_nodes/ids are not
        # trusted; the consensus winner set is the identity of record.
        assert out["n_nodes"] == 4
        assert set(out["contributing_node_ids"]) == {"a", "b", "c", "d"}

    def test_refine_fn_raising_is_swallowed(self):
        s_in = _s_in(["a", "b", "c", "d"])

        def boom(s_sub, cfgs):
            raise RuntimeError("refine blew up")

        out = solve_consensus(s_in, _NODE_CFGS, refine_fn=boom)
        assert out is not None and out["success"] is True
        assert _haversine_km(out["lat"], out["lon"], TRUTH[0], TRUTH[1]) < 5.0
