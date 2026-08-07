"""
Multi-node least-squares solver for passive radar geolocation.

Takes bistatic delay/Doppler measurements from multiple receiver nodes
observing the same target and solves for target position and velocity.

State vector: [x, y, z, vx, vy, vz]
  - Position in km (ENU relative to a common reference point)
  - Velocity in m/s (ENU)

Each node contributes two residuals per measurement:
  - Delay residual: measured_delay - predicted_delay (μs)
  - Doppler residual: measured_doppler - predicted_doppler (Hz)

Both are divided by the corresponding measurement sigma so the least-squares is
properly weighted and the two are commensurable.  Previously the delay residual
was used raw and the Doppler residual scaled by a bare 0.1 — implying
σ_delay = 1 μs and σ_doppler = 10 Hz, against a sensor model that produces
0.1 μs and 2.0 Hz.  That under-weighted delay by 2× relative to Doppler.

A note on frequency, because it is easy to get backwards: the Doppler Jacobian
carries K = fc/c, so a high-band node influences the velocity solution more than
a low-band one.  That is correct, not a bug.  Doppler noise is set by
integration time and is therefore roughly the same in Hz across bands, so
σ_v = σ_f·λ = σ_f·c/fc is *smaller* at high fc — a 599 MHz node genuinely
measures velocity ~3× more precisely than a 183 MHz one and has earned the
extra weight.  Converting the residual to m/s would remove exactly that
weighting and make the estimator worse.
"""

import math

import numpy as np
from scipy.optimize import least_squares

from .Geometry import Geometry

# Speed of light in km/µs (for delay) and km/s (for Doppler).
from retina_geolocator.constants import C_KM_S as _C_KM_S
from retina_geolocator.constants import C_KM_US as _C_KM_US

# Measurement sigmas, matching the sensor model the detections are generated
# under (world.generate_detections_for_node: 0.1 µs and 2.0 Hz at reference
# SNR, both scaled by the same noise factor).  Residuals are divided by these
# so delay and Doppler enter the least-squares in units of their own
# uncertainty rather than at an arbitrary relative scale.
#
# Only the *ratio* affects the solution — an overall factor cancels in the
# argmin — and the reported rms_delay / rms_doppler are recomputed unweighted
# in physical units afterwards, so the solver's rms gates are unaffected.
_SIGMA_DELAY_US = 0.1
_SIGMA_DOPPLER_HZ = 2.0

# Per-component velocity bound for the solve, m/s.  Also clips the seed from
# the association stage, whose own limit is on total speed and so can produce a
# single component beyond this — least_squares raises on an infeasible x0.
_V_BOUND_MS = 300.0
# Vertical velocity bound for the single-epoch solve, m/s.  Deliberately much
# tighter than the horizontal bound — see the note at the bounds in
# solve_multinode.  (fit_constant_velocity keeps a looser ±100: its multi-epoch
# span constrains velocity through the trajectory, not just the instantaneous
# Doppler set.)
_VZ_BOUND_MS = 20.0


def _sigma_for_snr(snr: float) -> tuple[float, float]:
    """Per-measurement (σ_delay µs, σ_doppler Hz) from SNR.

    Mirrors the sensor model the detections are generated under
    (world.generate_detections_for_node): a base 0.1 µs / 2.0 Hz scaled by
    ``max(0.5, 2.0 - (snr - 4)/18)``, so a marginal detection is twice as noisy
    as the reference and a strong one half.

    This is deliberately *not* the ``min(snr/10, 3.0)`` weight the position
    solver applies.  That weight is a relative emphasis — it only has to get the
    ordering right, and the docstring above notes an overall factor cancels in
    the argmin.  fit_constant_velocity's output is a χ², a number compared
    against an absolute threshold, so its σ has to be the real one or the
    statistic is not a χ² at all.  At SNR 20 the two disagree by 2×, which would
    be the difference between a threshold that rejects everything and one that
    rejects nothing.
    """
    scale = max(0.5, 2.0 - (snr - 4.0) / 18.0) if snr > 0 else 1.0
    return _SIGMA_DELAY_US * scale, _SIGMA_DOPPLER_HZ * scale


def _lla_to_enu_km(lat, lon, alt_m, ref_lat, ref_lon, ref_alt_m):
    """Convert LLA to ENU (km) relative to a reference point."""
    ecef = Geometry.lla2ecef(lat, lon, alt_m)
    enu_m = Geometry.ecef2enu(ecef[0], ecef[1], ecef[2],
                              ref_lat, ref_lon, ref_alt_m)
    return (enu_m[0] / 1000, enu_m[1] / 1000, enu_m[2] / 1000)


def _enu_km_to_lla(east_km, north_km, up_km, ref_lat, ref_lon, ref_alt_m):
    """Convert ENU (km) to LLA."""
    ecef = Geometry.enu2ecef(east_km * 1000, north_km * 1000, up_km * 1000,
                             ref_lat, ref_lon, ref_alt_m)
    return Geometry.ecef2lla(ecef[0], ecef[1], ecef[2])


class NodeSetup:
    """Pre-computed geometry for a single node used by the solver."""

    def __init__(self, node_id, rx_enu_km, tx_enu_km, fc_hz):
        self.node_id = node_id
        # Store as plain tuples so the inner loops use fast Python attribute access
        # without numpy array creation overhead on every residual evaluation.
        self.rx_enu = tuple(rx_enu_km)
        self.tx_enu = tuple(tx_enu_km)
        self.fc_hz = fc_hz
        # Pre-compute constants that are independent of the solver state.
        rx, tx = self.rx_enu, self.tx_enu
        self.d_baseline_km = math.sqrt(
            (rx[0] - tx[0]) ** 2 + (rx[1] - tx[1]) ** 2 + (rx[2] - tx[2]) ** 2
        )
        # fc / c  (Hz per km/s) — used for Doppler prediction.
        self.K_doppler = fc_hz / _C_KM_S


class MultiNodeMeasurement:
    """A single delay/Doppler measurement from one node."""

    def __init__(self, node_id, delay_us, doppler_hz, snr=0.0):
        self.node_id = node_id
        self.delay_us = delay_us
        self.doppler_hz = doppler_hz
        self.snr = snr


def _residual_function(state, node_setups, measurements, z_fixed_km):
    """Compute residuals for all measurements.

    Uses inlined bistatic geometry (math.sqrt instead of np.linalg.norm) to
    avoid numpy array allocation overhead on every function evaluation — the
    numerical Jacobian calls this O(iterations × n_params) times.

    Args:
        state: [x, y, vx, vy, vz] — horizontal position km, velocity m/s
        node_setups: dict[node_id] → NodeSetup
        measurements: list of MultiNodeMeasurement
        z_fixed_km: altitude in ENU km, pinned from initial guess

    Returns:
        Array of residuals (2 per measurement).
    """
    px = state[0]
    py = state[1]
    pz = z_fixed_km
    # Velocity in km/s (bistatic Doppler model uses km/s internally).
    vx = state[2] * 1e-3
    vy = state[3] * 1e-3
    vz = state[4] * 1e-3

    residuals = []

    for m in measurements:
        ns = node_setups[m.node_id]
        tx = ns.tx_enu
        rx = ns.rx_enu

        # TX → target distance (km)
        dptx = math.sqrt((px - tx[0]) ** 2 + (py - tx[1]) ** 2 + (pz - tx[2]) ** 2)
        # target → RX distance (km)
        dprx = math.sqrt((px - rx[0]) ** 2 + (py - rx[1]) ** 2 + (pz - rx[2]) ** 2)
        pred_delay = (dptx + dprx - ns.d_baseline_km) / _C_KM_US

        # Unit vectors from target toward TX and toward RX
        inv_dptx = 1.0 / dptx
        inv_dprx = 1.0 / dprx
        utx0 = (tx[0] - px) * inv_dptx
        utx1 = (tx[1] - py) * inv_dptx
        utx2 = (tx[2] - pz) * inv_dptx
        urx0 = (rx[0] - px) * inv_dprx
        urx1 = (rx[1] - py) * inv_dprx
        urx2 = (rx[2] - pz) * inv_dprx

        # Bistatic Doppler (Hz)
        v_tx = vx * utx0 + vy * utx1 + vz * utx2
        v_rx = vx * urx0 + vy * urx1 + vz * urx2
        pred_doppler = ns.K_doppler * (v_tx + v_rx)

        weight = min(m.snr / 10.0, 3.0) if m.snr > 0 else 1.0
        residuals.append((m.delay_us - pred_delay) * weight / _SIGMA_DELAY_US)
        residuals.append((m.doppler_hz - pred_doppler) * weight / _SIGMA_DOPPLER_HZ)

    return np.array(residuals, dtype=np.float64)


def _jacobian_function(state, node_setups, measurements, z_fixed_km):
    """Analytical Jacobian of the residuals w.r.t. state = [x, y, vx, vy, vz].

    Eliminates the costly finite-difference Jacobian used by scipy TRF
    (which calls the residual function n_params extra times per iteration).

    For state [x, y, vx, vy, vz] and residuals [r_delay, r_doppler] per node:

    wd = w/σ_delay,  wf = w/σ_doppler

    ∂r_delay/∂x  = -wd/c * ((px-tx_x)/d_tx + (px-rx_x)/d_rx)
    ∂r_delay/∂y  = -wd/c * ((py-tx_y)/d_tx + (py-rx_y)/d_rx)
    ∂r_delay/∂v* = 0

    ∂r_doppler/∂x   = -wf*K * [(-vx_kms + v_tx*utx_x)/d_tx
                                + (-vx_kms + v_rx*urx_x)/d_rx]
    ∂r_doppler/∂vx  = -wf*K * (utx_x + urx_x) / 1000
    (similarly for y, vy, vz)
    """
    px = state[0]
    py = state[1]
    pz = z_fixed_km
    vx = state[2] * 1e-3
    vy = state[3] * 1e-3
    vz = state[4] * 1e-3

    inv_C = 1.0 / _C_KM_US
    INV1000 = 1e-3

    rows = []
    for m in measurements:
        ns = node_setups[m.node_id]
        tx = ns.tx_enu
        rx = ns.rx_enu

        dptx = math.sqrt((px - tx[0]) ** 2 + (py - tx[1]) ** 2 + (pz - tx[2]) ** 2)
        dprx = math.sqrt((px - rx[0]) ** 2 + (py - rx[1]) ** 2 + (pz - rx[2]) ** 2)

        inv_dptx = 1.0 / dptx
        inv_dprx = 1.0 / dprx
        utx0 = (tx[0] - px) * inv_dptx
        utx1 = (tx[1] - py) * inv_dptx
        utx2 = (tx[2] - pz) * inv_dptx
        urx0 = (rx[0] - px) * inv_dprx
        urx1 = (rx[1] - py) * inv_dprx
        urx2 = (rx[2] - pz) * inv_dprx

        v_tx = vx * utx0 + vy * utx1 + vz * utx2
        v_rx = vx * urx0 + vy * urx1 + vz * urx2

        w = min(m.snr / 10.0, 3.0) if m.snr > 0 else 1.0
        wd = w / _SIGMA_DELAY_US
        wf = w / _SIGMA_DOPPLER_HZ
        K = ns.K_doppler

        # ── Delay row ──────────────────────────────────────────────────────
        dr1_dx = -wd * inv_C * ((px - tx[0]) * inv_dptx + (px - rx[0]) * inv_dprx)
        dr1_dy = -wd * inv_C * ((py - tx[1]) * inv_dptx + (py - rx[1]) * inv_dprx)
        rows.append([dr1_dx, dr1_dy, 0.0, 0.0, 0.0])

        # ── Doppler row ─────────────────────────────────────────────────────
        w01 = -wf * K
        dr2_dx = w01 * ((-vx + v_tx * utx0) * inv_dptx + (-vx + v_rx * urx0) * inv_dprx)
        dr2_dy = w01 * ((-vy + v_tx * utx1) * inv_dptx + (-vy + v_rx * urx1) * inv_dprx)
        dr2_dvx = w01 * (utx0 + urx0) * INV1000
        dr2_dvy = w01 * (utx1 + urx1) * INV1000
        dr2_dvz = w01 * (utx2 + urx2) * INV1000
        rows.append([dr2_dx, dr2_dy, dr2_dvx, dr2_dvy, dr2_dvz])

    return np.array(rows, dtype=np.float64)


def _cv_residuals(state, node_setups, epochs, return_jac=False):
    """Residuals (and optionally the Jacobian) for a constant-velocity fit.

    state: [x, y, z, vx, vy, vz] — position km ENU at t0, velocity m/s ENU.
    epochs: list of (dt_s, [(_CVMeas), ...]) with dt measured from t0.

    Position at each epoch is p(t) = p0 + v·dt, so a measurement taken later in
    the window constrains velocity through geometry as well as through Doppler.
    That coupling is the whole point: it is what makes the system
    over-determined and therefore testable.
    """
    px0, py0, pz0 = state[0], state[1], state[2]
    vx = state[3] * 1e-3   # km/s
    vy = state[4] * 1e-3
    vz = state[5] * 1e-3

    inv_C = 1.0 / _C_KM_US
    res = []
    rows = [] if return_jac else None

    for dt, meas in epochs:
        # km per (m/s) — the chain-rule factor from velocity to position.
        dt_km_per_ms = dt * 1e-3
        px = px0 + vx * dt
        py = py0 + vy * dt
        pz = pz0 + vz * dt

        for m in meas:
            ns = node_setups[m.node_id]
            tx, rx = ns.tx_enu, ns.rx_enu

            dptx = math.sqrt((px - tx[0]) ** 2 + (py - tx[1]) ** 2 + (pz - tx[2]) ** 2)
            dprx = math.sqrt((px - rx[0]) ** 2 + (py - rx[1]) ** 2 + (pz - rx[2]) ** 2)
            inv_dptx = 1.0 / dptx
            inv_dprx = 1.0 / dprx

            utx0 = (tx[0] - px) * inv_dptx
            utx1 = (tx[1] - py) * inv_dptx
            utx2 = (tx[2] - pz) * inv_dptx
            urx0 = (rx[0] - px) * inv_dprx
            urx1 = (rx[1] - py) * inv_dprx
            urx2 = (rx[2] - pz) * inv_dprx

            pred_delay = (dptx + dprx - ns.d_baseline_km) * inv_C
            v_tx = vx * utx0 + vy * utx1 + vz * utx2
            v_rx = vx * urx0 + vy * urx1 + vz * urx2
            pred_doppler = ns.K_doppler * (v_tx + v_rx)

            inv_sd = 1.0 / m.sigma_delay
            inv_sf = 1.0 / m.sigma_doppler
            res.append((m.delay_us - pred_delay) * inv_sd)
            res.append((m.doppler_hz - pred_doppler) * inv_sf)

            if not return_jac:
                continue

            # ── Delay row ────────────────────────────────────────────────────
            dx = -inv_sd * inv_C * ((px - tx[0]) * inv_dptx + (px - rx[0]) * inv_dprx)
            dy = -inv_sd * inv_C * ((py - tx[1]) * inv_dptx + (py - rx[1]) * inv_dprx)
            dz = -inv_sd * inv_C * ((pz - tx[2]) * inv_dptx + (pz - rx[2]) * inv_dprx)
            rows.append([dx, dy, dz,
                         dx * dt_km_per_ms, dy * dt_km_per_ms, dz * dt_km_per_ms])

            # ── Doppler row ──────────────────────────────────────────────────
            wk = -inv_sf * ns.K_doppler
            fx = wk * ((-vx + v_tx * utx0) * inv_dptx + (-vx + v_rx * urx0) * inv_dprx)
            fy = wk * ((-vy + v_tx * utx1) * inv_dptx + (-vy + v_rx * urx1) * inv_dprx)
            fz = wk * ((-vz + v_tx * utx2) * inv_dptx + (-vz + v_rx * urx2) * inv_dprx)
            # Velocity enters twice: directly through the Doppler projection and
            # indirectly through where the target has moved to by this epoch.
            gvx = wk * (utx0 + urx0) * 1e-3 + fx * dt_km_per_ms
            gvy = wk * (utx1 + urx1) * 1e-3 + fy * dt_km_per_ms
            gvz = wk * (utx2 + urx2) * 1e-3 + fz * dt_km_per_ms
            rows.append([fx, fy, fz, gvx, gvy, gvz])

    if return_jac:
        return np.array(rows, dtype=np.float64)
    return np.array(res, dtype=np.float64)


class _CVMeas:
    """One delay/Doppler measurement with its own sigmas, for the batch fit."""

    __slots__ = ("node_id", "delay_us", "doppler_hz", "sigma_delay", "sigma_doppler")

    def __init__(self, node_id, delay_us, doppler_hz, snr=0.0):
        self.node_id = node_id
        self.delay_us = delay_us
        self.doppler_hz = doppler_hz
        self.sigma_delay, self.sigma_doppler = _sigma_for_snr(snr)


def fit_constant_velocity(fit_input, node_configs):
    """Fit one constant-velocity trajectory to K epochs of bistatic measurements.

    This is the test that works at n=2, where the single-epoch residual gates
    structurally cannot.  A single epoch from two nodes gives 4 measurements
    against 6 unknowns (x, y, z, vx, vy, vz) — under-determined, so both
    rms_delay and rms_doppler go to zero for a cross pairing exactly as they do
    for a real target, and no threshold on them carries information.  Pinning
    altitude, as solve_multinode does, trades one unknown for the ambiguity and
    still leaves nothing to test: differentiating the two delay equations shows
    the phantom's own velocity satisfies the *same* 2x2 system that the two
    Doppler measurements do, so a cross pairing is self-consistent to first
    order for as long as both aircraft are tracked.

    Time is the only remaining information.  Over K epochs the count becomes 4K
    measurements against the same 6 unknowns — over-determined from K=2, and
    comfortably so by the 5 epochs a confirmed per-node track already carries.
    A real target fits because it is one rigid point flying straight; a cross
    pairing has to make two independent aircraft look like one through a
    nonlinear geometric map, and the trajectory it needs is curved.

    The fit runs in measurement space (σ ≈ 0.1 µs ≈ 30 m), not position space,
    so it is not defeated by the GDOP amplification that puts blind n=2 position
    error in the kilometres.

    Args:
        fit_input: {
            "initial_guess": {"lat", "lon", "alt_km"},
            "initial_velocity": {"vel_east_ms", "vel_north_ms"} | None,
            "epochs": [{"t_s": float,
                        "measurements": [{"node_id", "delay_us",
                                          "doppler_hz", "snr"}, ...]}, ...],
        }
        node_configs: dict[node_id] → config with rx/tx lat/lon/alt and fc_hz.

    Returns:
        dict with success, lat, lon, alt_m, vel_east/north/up, chi2, dof,
        chi2_per_dof, n_epochs, n_measurements, contributing_node_ids,
        timestamp_ms — or None if the input is too thin or the fit fails.

        lat/lon/alt_m are the trajectory evaluated at the *last* epoch, i.e.
        where the target is now, not where the state vector is parameterised.
    """
    guess = fit_input.get("initial_guess") or {}
    raw_epochs = fit_input.get("epochs") or []
    if len(raw_epochs) < 2 or not guess:
        return None

    ref_lat = guess["lat"]
    ref_lon = guess["lon"]
    ref_alt_m = 0.0

    node_setups = {}
    for ep in raw_epochs:
        for m in ep.get("measurements", ()):
            nid = m["node_id"]
            if nid in node_setups:
                continue
            cfg = node_configs.get(nid)
            if cfg is None:
                continue
            rx_enu = _lla_to_enu_km(
                cfg.get("rx_lat", 0), cfg.get("rx_lon", 0),
                cfg.get("rx_alt_ft", 0) * 0.3048, ref_lat, ref_lon, ref_alt_m,
            )
            tx_enu = _lla_to_enu_km(
                cfg.get("tx_lat", 0), cfg.get("tx_lon", 0),
                cfg.get("tx_alt_ft", 0) * 0.3048, ref_lat, ref_lon, ref_alt_m,
            )
            node_setups[nid] = NodeSetup(
                nid, rx_enu, tx_enu, cfg.get("fc_hz", cfg.get("FC", 195e6)),
            )

    # Epoch times are relative to the first, so dt=0 at the state's reference
    # and the position parameters stay well-scaled.
    t0 = float(raw_epochs[0]["t_s"])
    epochs = []
    n_meas = 0
    for ep in raw_epochs:
        meas = [
            _CVMeas(m["node_id"], m["delay_us"], m["doppler_hz"], m.get("snr", 0))
            for m in ep.get("measurements", ())
            if m["node_id"] in node_setups
        ]
        if not meas:
            continue
        epochs.append((float(ep["t_s"]) - t0, meas))
        n_meas += len(meas)

    n_resid = 2 * n_meas
    dof = n_resid - 6
    if len(epochs) < 2 or dof < 1:
        return None

    guess_enu = _lla_to_enu_km(
        guess["lat"], guess["lon"], float(guess.get("alt_km", 7.0)) * 1000,
        ref_lat, ref_lon, ref_alt_m,
    )
    _v0 = fit_input.get("initial_velocity") or {}
    _seed_e = min(_V_BOUND_MS, max(-_V_BOUND_MS, float(_v0.get("vel_east_ms") or 0.0)))
    _seed_n = min(_V_BOUND_MS, max(-_V_BOUND_MS, float(_v0.get("vel_north_ms") or 0.0)))
    # Position starts at the ENU origin for the same reason solve_multinode does
    # it: guess_enu[0:2] are ~1e-13 rather than exactly zero, and a near-zero
    # ||x0|| collapses the TRF trust region into immediate false convergence.
    x0 = np.array([0.0, 0.0, guess_enu[2], _seed_e, _seed_n, 0.0])

    lb = [-60.0, -60.0, 0.05, -_V_BOUND_MS, -_V_BOUND_MS, -100.0]
    ub = [60.0, 60.0, 20.0, _V_BOUND_MS, _V_BOUND_MS, 100.0]
    x0 = np.clip(x0, lb, ub)

    try:
        # Plain L2, unlike solve_multinode's Huber.  Huber exists there to stop
        # one bad channel dragging a position fix; here the sum of squares *is*
        # the output, and down-weighting the large residuals would flatten
        # exactly the signal the test reads.
        result = least_squares(
            _cv_residuals,
            x0,
            jac=lambda s, *a: _cv_residuals(s, *a, return_jac=True),
            args=(node_setups, epochs),
            method="trf",
            bounds=(lb, ub),
            max_nfev=200,
            ftol=1e-8,
            xtol=1e-8,
        )
    except Exception:
        return None

    chi2 = float(np.sum(result.fun ** 2))
    state = result.x

    # Report the position at the *last* epoch, not at t0.  The state is
    # parameterised at the start of the window, which for a 20-sample track
    # history is up to 40 s in the past — at 200 m/s that is 8 km of staleness
    # in a quantity every caller reads as "where the target is".  It also feeds
    # solver.py's 2 km displacement gate, which would reject good solves for
    # disagreeing with a deliberately old position.  Measured: reporting t0 put
    # the fit at 3.94 km median error against 2.62 km for a single-epoch
    # re-solve, which is the wrong way round given the fit uses 4K measurements.
    dt_last = epochs[-1][0]
    px = state[0] + state[3] * 1e-3 * dt_last
    py = state[1] + state[4] * 1e-3 * dt_last
    pz = state[2] + state[5] * 1e-3 * dt_last
    lat, lon, alt_m = _enu_km_to_lla(px, py, pz, ref_lat, ref_lon, ref_alt_m)

    return {
        "success": True,
        "lat": float(lat),
        "lon": float(lon),
        "alt_m": float(alt_m),
        "vel_east": float(state[3]),
        "vel_north": float(state[4]),
        "vel_up": float(state[5]),
        "chi2": chi2,
        "dof": int(dof),
        "chi2_per_dof": chi2 / dof,
        "n_epochs": len(epochs),
        "n_measurements": n_meas,
        "contributing_node_ids": sorted(node_setups),
        "timestamp_ms": fit_input.get("timestamp_ms", 0),
    }


def solve_multinode(solver_input, node_configs):
    """Solve for target position using multi-node measurements.

    Args:
        solver_input: dict from InterNodeAssociator.format_candidates_for_solver():
            {
                "initial_guess": {"lat", "lon", "alt_km"},
                "measurements": [{"node_id", "delay_us", "doppler_hz", "snr"}, ...],
                "n_nodes": int,
                "timestamp_ms": int,
            }
        node_configs: dict[node_id] → config dict with
            rx_lat, rx_lon, rx_alt_ft, tx_lat, tx_lon, tx_alt_ft, fc_hz/FC

    Returns:
        dict with:
            success: bool
            lat, lon, alt_m: solved position
            vel_east, vel_north, vel_up: velocity m/s
            rms_delay, rms_doppler: fit quality
            n_nodes: number of nodes used
            timestamp_ms: from input
        or None if solve fails
    """
    guess = solver_input["initial_guess"]
    meas_list = solver_input["measurements"]

    if len(meas_list) < 2:
        return None

    # Choose a common reference point (the initial guess location)
    ref_lat = guess["lat"]
    ref_lon = guess["lon"]
    ref_alt_m = 0.0  # reference at sea level

    # Build NodeSetup objects for each node
    node_setups = {}
    for m in meas_list:
        nid = m["node_id"]
        if nid in node_setups:
            continue
        cfg = node_configs.get(nid)
        if cfg is None:
            continue

        rx_alt_m = cfg.get("rx_alt_ft", 0) * 0.3048
        tx_alt_m = cfg.get("tx_alt_ft", 0) * 0.3048

        rx_enu = _lla_to_enu_km(
            cfg.get("rx_lat", 0), cfg.get("rx_lon", 0), rx_alt_m,
            ref_lat, ref_lon, ref_alt_m,
        )
        tx_enu = _lla_to_enu_km(
            cfg.get("tx_lat", 0), cfg.get("tx_lon", 0), tx_alt_m,
            ref_lat, ref_lon, ref_alt_m,
        )
        fc_hz = cfg.get("fc_hz", cfg.get("FC", 195e6))
        node_setups[nid] = NodeSetup(nid, rx_enu, tx_enu, fc_hz)

    # Build measurement objects
    measurements = []
    for m in meas_list:
        if m["node_id"] not in node_setups:
            continue
        measurements.append(MultiNodeMeasurement(
            node_id=m["node_id"],
            delay_us=m["delay_us"],
            doppler_hz=m["doppler_hz"],
            snr=m.get("snr", 0),
        ))

    if len(measurements) < 2:
        return None

    # Pin altitude from initial guess (the grid altitude layer that best matched
    # the measured delays during association). With altitude fixed, the 2D
    # position solve (x, y) becomes exactly determined by 2 bistatic delay
    # equations — eliminating the altitude-drift ambiguity that causes 5-10 km
    # errors in unconstrained 3D solves with only 2 nodes.
    guess_enu = _lla_to_enu_km(
        guess["lat"], guess["lon"], guess["alt_km"] * 1000,
        ref_lat, ref_lon, ref_alt_m,
    )
    z_fixed_km = guess_enu[2]  # altitude fixed in ENU km

    # State: [x, y, vx, vy, vz] — altitude removed from state vector.
    # Initial position is the ENU origin (= the guess location by construction),
    # so x=0, y=0 exactly. Using guess_enu[0:2] directly gives floating-point
    # residuals ~1e-13 that make ||x0|| ≈ 0, collapsing the TRF trust region
    # to zero and triggering immediate false-convergence on the first step.
    #
    # Velocity is seeded from the association stage's level-flight estimate
    # when one is available.  With n=2 the system is under-determined in
    # velocity — two Doppler projections for three components — so from a zero
    # start the optimiser slides along the null direction and stops wherever
    # the trust region runs out, which is where the physically impossible
    # speeds came from.  The seed puts it on the branch the measurements
    # actually imply; it changes nothing when the problem is well-determined.
    # Clipped to _V_BOUND_MS: association accepts a speed up to its own limit,
    # which a single component can exceed, and least_squares raises on an
    # infeasible x0.
    _v0 = solver_input.get("initial_velocity") or {}
    _seed_e = min(_V_BOUND_MS, max(-_V_BOUND_MS, float(_v0.get("vel_east_ms") or 0.0)))
    _seed_n = min(_V_BOUND_MS, max(-_V_BOUND_MS, float(_v0.get("vel_north_ms") or 0.0)))
    x0 = np.array([0.0, 0.0, _seed_e, _seed_n, 0.0])

    # Bounds: horizontal position ±60 km from guess, horizontal velocity
    # ±_V_BOUND_MS.  Vertical velocity is pinned near level flight (±20 m/s
    # covers commercial climb/descent): with n Doppler equations against
    # three velocity unknowns the fit is exactly determined at n=3, and a
    # ±100 m/s vz absorbed the entire Doppler misfit — rms_doppler came out
    # 0.0 on every staging n=3 solve (an inert reject gate) while horizontal
    # velocity landed ~70 m/s off and dead-reckoned the map marker km away.
    # Tightening vz makes the velocity part overdetermined again, so
    # vel_east/vel_north are honest and rms_doppler is a live residual.
    lb = [x0[0] - 60, x0[1] - 60, -_V_BOUND_MS, -_V_BOUND_MS, -_VZ_BOUND_MS]
    ub = [x0[0] + 60, x0[1] + 60,  _V_BOUND_MS,  _V_BOUND_MS,  _VZ_BOUND_MS]

    try:
        # `loss="huber"` — robust regression instead of plain L2.
        # After lifting the N=2 EWMA gate, residual long tails remain at higher
        # N (e.g. N=4 mean 2.81 km vs median 0.25 km — one bad channel in five
        # drags the LS fit). Huber treats residuals up to f_scale quadratically
        # and falls back to linear beyond, so a single misbehaving measurement
        # can no longer dominate the cost. f_scale = 1.0 is in the same units
        # as the SNR-weighted residuals (delay σ ≈ 0.1 µs × SNR weight cap 3 ≈
        # 0.3, doppler weighted ≈ same), placing the cap ~3σ above the noise
        # floor. Under pure Gaussian noise Huber behaves identically to plain
        # L2, so well-behaved frames are unaffected.
        result = least_squares(
            _residual_function,
            x0,
            jac=_jacobian_function,
            args=(node_setups, measurements, z_fixed_km),
            method="trf",
            loss="huber",
            f_scale=1.0,
            bounds=(lb, ub),
            max_nfev=200,
            ftol=1e-8,
            xtol=1e-8,
        )
    except Exception:
        return None

    if not result.success and result.cost > 1000:
        return None

    state = result.x
    px, py = state[0], state[1]
    pz = z_fixed_km
    vx_kms = state[2] * 1e-3
    vy_kms = state[3] * 1e-3
    vz_kms = state[4] * 1e-3

    # Compute RMS delay and Doppler using the same inlined geometry as the solver.
    delay_residuals = []
    doppler_residuals = []
    for m in measurements:
        ns = node_setups[m.node_id]
        tx = ns.tx_enu
        rx = ns.rx_enu
        dptx = math.sqrt((px - tx[0]) ** 2 + (py - tx[1]) ** 2 + (pz - tx[2]) ** 2)
        dprx = math.sqrt((px - rx[0]) ** 2 + (py - rx[1]) ** 2 + (pz - rx[2]) ** 2)
        pred_d = (dptx + dprx - ns.d_baseline_km) / _C_KM_US
        inv_dptx = 1.0 / dptx
        inv_dprx = 1.0 / dprx
        utx0 = (tx[0] - px) * inv_dptx
        utx1 = (tx[1] - py) * inv_dptx
        utx2 = (tx[2] - pz) * inv_dptx
        urx0 = (rx[0] - px) * inv_dprx
        urx1 = (rx[1] - py) * inv_dprx
        urx2 = (rx[2] - pz) * inv_dprx
        pred_f = ns.K_doppler * (
            vx_kms * utx0 + vy_kms * utx1 + vz_kms * utx2
            + vx_kms * urx0 + vy_kms * urx1 + vz_kms * urx2
        )
        delay_residuals.append(m.delay_us - pred_d)
        doppler_residuals.append(m.doppler_hz - pred_f)

    rms_delay = float(np.sqrt(np.mean(np.array(delay_residuals) ** 2)))
    rms_doppler = float(np.sqrt(np.mean(np.array(doppler_residuals) ** 2)))

    # Convert solution ENU back to LLA (z is fixed)
    lat, lon, alt_m = _enu_km_to_lla(
        state[0], state[1], z_fixed_km,
        ref_lat, ref_lon, ref_alt_m,
    )

    return {
        "success": True,
        "lat": float(lat),
        "lon": float(lon),
        "alt_m": float(alt_m),
        "vel_east": float(state[2]),
        "vel_north": float(state[3]),
        "vel_up": float(state[4]),
        "rms_delay": rms_delay,
        "rms_doppler": rms_doppler,
        "n_nodes": len(node_setups),
        "contributing_node_ids": list({m.node_id for m in measurements}),
        "n_measurements": len(measurements),
        "cost": float(result.cost),
        "timestamp_ms": solver_input.get("timestamp_ms", 0),
    }
