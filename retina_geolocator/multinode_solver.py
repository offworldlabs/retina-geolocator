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
_C_KM_US = 0.299792458
_C_KM_S = 299792.458

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

    # Bounds: horizontal position ±60 km from guess, velocity ±_V_BOUND_MS
    lb = [x0[0] - 60, x0[1] - 60, -_V_BOUND_MS, -_V_BOUND_MS, -100]
    ub = [x0[0] + 60, x0[1] + 60,  _V_BOUND_MS,  _V_BOUND_MS,  100]

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
        utx0 = (tx[0] - px) * inv_dptx; utx1 = (tx[1] - py) * inv_dptx; utx2 = (tx[2] - pz) * inv_dptx
        urx0 = (rx[0] - px) * inv_dprx; urx1 = (rx[1] - py) * inv_dprx; urx2 = (rx[2] - pz) * inv_dprx
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
