"""The constant-velocity batch fit, which is what makes n=2 testable.

A single epoch from two nodes gives 4 measurements against 6 unknowns, so both
rms_delay and rms_doppler go to zero for a cross pairing exactly as they do for
a real target.  Over K epochs the count becomes 4K against the same 6, and a
cross pairing then has to make two independent aircraft look like one rigid
point flying straight.  These tests pin that: the fit recovers a known
trajectory, its χ² is calibrated, and it separates true pairings from crossed
ones.
"""

import math
import random

import pytest

from retina_geolocator.multinode_solver import (
    _sigma_for_snr,
    fit_constant_velocity,
)

_C_KM_S = 299792.458

# Two nodes at one receiver on two transmitters — a dual-illuminator site, the
# arrangement the n=2 case actually arises in.
_CFGS = {
    "a": {"rx_lat": 34.85, "rx_lon": -82.40, "rx_alt_ft": 1000,
          "tx_lat": 34.9412, "tx_lon": -82.4103, "tx_alt_ft": 2000,
          "fc_hz": 183e6},
    "b": {"rx_lat": 34.85, "rx_lon": -82.40, "rx_alt_ft": 1000,
          "tx_lat": 34.9701, "tx_lon": -81.9484, "tx_alt_ft": 800,
          "fc_hz": 195e6},
}

_R_EARTH_KM = 6371.0


def _enu_km(lat, lon, alt_km, ref_lat, ref_lon, ref_alt_km):
    east = math.radians(lon - ref_lon) * _R_EARTH_KM * math.cos(math.radians(ref_lat))
    north = math.radians(lat - ref_lat) * _R_EARTH_KM
    return (east, north, alt_km - ref_alt_km)


def _measure(cfg, lat, lon, alt_km, ve, vn, vu, rng=None):
    """Bistatic delay (µs) and Doppler (Hz) for a target at a known state."""
    ref_alt = cfg["rx_alt_ft"] * 0.0003048
    tgt = _enu_km(lat, lon, alt_km, cfg["rx_lat"], cfg["rx_lon"], ref_alt)
    tx = _enu_km(cfg["tx_lat"], cfg["tx_lon"], cfg["tx_alt_ft"] * 0.0003048,
                 cfg["rx_lat"], cfg["rx_lon"], ref_alt)
    d_tx = math.dist(tgt, tx)
    d_rx = math.dist(tgt, (0.0, 0.0, 0.0))
    baseline = math.dist(tx, (0.0, 0.0, 0.0))
    delay_us = (d_tx + d_rx - baseline) / 0.299792458

    # Doppler is the velocity projected on the bistatic bisector b = u_tx + u_rx,
    # divided by the wavelength.  Velocity is in m/s here, so the wavelength has
    # to be in metres too: λ = c/fc with c in m/s.
    b = [(tx[i] - tgt[i]) / d_tx + (0.0 - tgt[i]) / d_rx for i in range(3)]
    lam_m = (_C_KM_S * 1000.0) / cfg["fc_hz"]
    doppler_hz = (ve * b[0] + vn * b[1] + vu * b[2]) / lam_m

    if rng is not None:
        delay_us += rng.gauss(0, 0.1)
        doppler_hz += rng.gauss(0, 2.0)
    return delay_us, doppler_hz


def _trajectory(lat, lon, alt_km, ve, vn, n_epochs, dt_s, rng=None,
                node_ids=("a", "b")):
    """K epochs of measurements from a straight, level, constant-speed target."""
    epochs = []
    for k in range(n_epochs):
        t = k * dt_s
        la = lat + vn * t / 111_320.0
        lo = lon + ve * t / (111_320.0 * math.cos(math.radians(lat)))
        meas = []
        for nid in node_ids:
            d, f = _measure(_CFGS[nid], la, lo, alt_km, ve, vn, 0.0, rng)
            meas.append({"node_id": nid, "delay_us": d, "doppler_hz": f,
                         "snr": 15.0})
        epochs.append({"t_s": t, "measurements": meas})
    return epochs


def _crossed(rng, n_epochs=5, dt_s=4.0):
    """Node a sees one aircraft, node b sees a different one.

    This is the pairing the whole exercise exists to reject: both halves are
    genuine detections of genuine aircraft, so every single-epoch residual gate
    passes it.
    """
    p = _trajectory(34.88, -82.35, 7.0, 180.0, -90.0, n_epochs, dt_s, rng)
    q = _trajectory(34.92, -82.28, 9.0, -120.0, 150.0, n_epochs, dt_s, rng)
    return [
        {"t_s": x["t_s"],
         "measurements": [x["measurements"][0], y["measurements"][1]]}
        for x, y in zip(p, q)
    ]


def _fit(epochs, lat=34.88, lon=-82.35, alt_km=7.0, vel=None):
    return fit_constant_velocity(
        {"initial_guess": {"lat": lat, "lon": lon, "alt_km": alt_km},
         "initial_velocity": vel,
         "epochs": epochs},
        _CFGS,
    )


class TestRecoversAKnownTrajectory:
    def test_noiseless_fit_is_exact(self):
        """Position, altitude and velocity all come back, from a wrong start.

        The reported position is the trajectory at the *last* epoch — where the
        target is now — so the expectation is the start point dead-reckoned
        across the window, not the start point itself.
        """
        lat, lon, alt, ve, vn = 34.88, -82.35, 7.0, 180.0, -90.0
        n, dt = 5, 2.0
        epochs = _trajectory(lat, lon, alt, ve, vn, n, dt)
        # Deliberately start ~2 km off in position and 2 km off in altitude.
        out = _fit(epochs, lat=lat + 0.02, lon=lon - 0.02, alt_km=9.0,
                   vel={"vel_east_ms": 150.0, "vel_north_ms": -60.0})

        assert out is not None and out["success"]
        span = (n - 1) * dt
        exp_lat = lat + vn * span / 111_320.0
        exp_lon = lon + ve * span / (111_320.0 * math.cos(math.radians(lat)))
        err_km = math.hypot((out["lat"] - exp_lat) * 111.32,
                            (out["lon"] - exp_lon) * 91.3)
        assert err_km < 0.2, f"position off by {err_km:.3f} km"
        # Altitude is a free parameter here — pinning it is what removes the
        # extra constraint the test depends on — so it has to be solved for.
        assert out["alt_m"] == pytest.approx(alt * 1000, abs=300)
        assert out["vel_east"] == pytest.approx(ve, abs=2.0)
        assert out["vel_north"] == pytest.approx(vn, abs=2.0)
        assert out["chi2_per_dof"] < 1e-3

    def test_degrees_of_freedom_count(self):
        """4 measurements per epoch against 6 unknowns."""
        for k in (2, 3, 5):
            out = _fit(_trajectory(34.88, -82.35, 7.0, 180.0, -90.0, k, 2.0))
            assert out["n_epochs"] == k
            assert out["dof"] == 2 * 2 * k - 6

    def test_one_epoch_is_refused(self):
        """K=1 is under-determined, so there is nothing to test and no fit."""
        assert _fit(_trajectory(34.88, -82.35, 7.0, 180.0, -90.0, 1, 2.0)) is None


class TestChiSquaredIsCalibrated:
    def test_true_pairing_sits_near_one(self):
        """With the real sigmas, a true pairing's χ²/dof should be O(1).

        This is why _sigma_for_snr exists rather than reusing the position
        solver's min(snr/10, 3) emphasis: an absolute threshold is only
        meaningful against the real measurement noise.
        """
        rng = random.Random(7)
        vals = [
            _fit(_trajectory(34.88, -82.35, 7.0, 180.0, -90.0, 5, 4.0, rng))["chi2_per_dof"]
            for _ in range(40)
        ]
        median = sorted(vals)[len(vals) // 2]
        assert 0.1 < median < 3.0, f"χ²/dof median {median:.2f} is not O(1)"

    def test_sigma_tracks_snr(self):
        """A marginal detection is weighted less than a strong one."""
        weak_d, weak_f = _sigma_for_snr(4.0)
        strong_d, strong_f = _sigma_for_snr(30.0)
        assert weak_d > strong_d and weak_f > strong_f
        # Floored, so an implausibly strong detection cannot claim zero noise.
        assert _sigma_for_snr(1e6) == _sigma_for_snr(50.0)


class TestSeparatesCrossPairings:
    def test_crossed_pairing_fails_where_single_epoch_gates_cannot(self):
        """The measured claim: separation at a 16 s baseline.

        Five epochs 4 s apart gives dof 14.  Every one of these crossed
        pairings passes rms_delay and rms_doppler at a single epoch, because
        with 4 residuals against 6 unknowns those residuals are structurally
        zero.
        """
        rng = random.Random(11)
        true_chi = sorted(
            _fit(_trajectory(34.88, -82.35, 7.0, 180.0, -90.0, 5, 4.0, rng))["chi2_per_dof"]
            for _ in range(40)
        )
        false_chi = [_fit(_crossed(rng, 5, 4.0))["chi2_per_dof"] for _ in range(40)]

        # Threshold at the 95th percentile of true pairings — keep 95% of real
        # targets — and count how many crossed pairings that rejects.
        threshold = true_chi[int(0.95 * (len(true_chi) - 1))]
        rejected = sum(1 for x in false_chi if x > threshold) / len(false_chi)
        assert rejected > 0.7, (
            f"only {rejected:.0%} of crossed pairings rejected at 95% TPR "
            f"(threshold χ²/dof {threshold:.2f})"
        )

    def test_separation_improves_with_baseline(self):
        """Epoch count alone is not enough — the window has to be long enough.

        A crossed pairing is close to constant-velocity over a short window; it
        is the accumulated curvature that gives it away.  Measured, five epochs
        over 8 s separate far less well than the same five over 16 s, which is
        the thing to remember when choosing the association cadence.
        """
        rng = random.Random(13)

        def rejection(dt_s):
            true_chi = sorted(
                _fit(_trajectory(34.88, -82.35, 7.0, 180.0, -90.0, 5, dt_s, rng))["chi2_per_dof"]
                for _ in range(30)
            )
            thr = true_chi[int(0.95 * (len(true_chi) - 1))]
            false_chi = [_fit(_crossed(rng, 5, dt_s))["chi2_per_dof"] for _ in range(30)]
            return sum(1 for x in false_chi if x > thr) / len(false_chi)

        assert rejection(4.0) > rejection(2.0)


class TestReportedPositionIsCurrent:
    """lat/lon are the trajectory at the last epoch, not at the state's t0.

    The state is parameterised at the start of the window, which for a 20-sample
    history is up to 40 s in the past — 8 km at 200 m/s.  Reporting that as "the
    position" made the fit measure *worse* than a single-epoch re-solve (3.94 km
    median against 2.62 km) despite using 4K measurements, and fed a stale guess
    to the solver's 2 km displacement gate.
    """

    def test_position_advances_with_the_window(self):
        lat, lon, ve, vn = 34.88, -82.35, 200.0, 0.0
        short = _fit(_trajectory(lat, lon, 7.0, ve, vn, 3, 2.0))
        long = _fit(_trajectory(lat, lon, 7.0, ve, vn, 9, 2.0))

        # Due east at 200 m/s: 4 s of window vs 16 s is 2.4 km of separation.
        east_km_short = (short["lon"] - lon) * 91.3
        east_km_long = (long["lon"] - lon) * 91.3
        assert east_km_long - east_km_short == pytest.approx(2.4, abs=0.4)

    def test_a_stationary_target_reports_where_it_is(self):
        """With no velocity there is nothing to propagate, so t0 == t_last."""
        lat, lon = 34.88, -82.35
        out = _fit(_trajectory(lat, lon, 7.0, 0.0, 0.0, 6, 2.0))
        err_km = math.hypot((out["lat"] - lat) * 111.32, (out["lon"] - lon) * 91.3)
        assert err_km < 0.3
