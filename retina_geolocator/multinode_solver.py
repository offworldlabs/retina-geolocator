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
"""

import numpy as np
from scipy.optimize import least_squares

from .Geometry import Geometry
from .bistatic_models import bistatic_delay, bistatic_doppler


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
        self.rx_enu = rx_enu_km
        self.tx_enu = tx_enu_km
        self.fc_hz = fc_hz


class MultiNodeMeasurement:
    """A single delay/Doppler measurement from one node."""

    def __init__(self, node_id, delay_us, doppler_hz, snr=0.0):
        self.node_id = node_id
        self.delay_us = delay_us
        self.doppler_hz = doppler_hz
        self.snr = snr


def _residual_function(state, node_setups, measurements, z_fixed_km):
    """Compute residuals for all measurements.

    Args:
        state: [x, y, vx, vy, vz] — horizontal position km, velocity m/s
        node_setups: dict[node_id] → NodeSetup
        measurements: list of MultiNodeMeasurement
        z_fixed_km: altitude in ENU km, pinned from initial guess

    Returns:
        Array of residuals (2 per measurement).
    """
    pos = np.array([state[0], state[1], z_fixed_km])
    vel = state[2:5]

    residuals = []

    for m in measurements:
        ns = node_setups[m.node_id]

        pred_delay = bistatic_delay(pos, ns.tx_enu, ns.rx_enu)
        pred_doppler = bistatic_doppler(pos, vel, ns.tx_enu, ns.rx_enu, ns.fc_hz)

        # Weight by SNR (higher SNR = tighter constraint)
        weight = 1.0
        if m.snr > 0:
            weight = min(m.snr / 10.0, 3.0)

        residuals.append((m.delay_us - pred_delay) * weight)
        residuals.append((m.doppler_hz - pred_doppler) * weight * 0.1)

    return np.array(residuals)


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
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # Bounds: horizontal position ±60 km from guess, velocity ±300 m/s
    lb = [x0[0] - 60, x0[1] - 60, -300, -300, -100]
    ub = [x0[0] + 60, x0[1] + 60,  300,  300,  100]

    try:
        result = least_squares(
            _residual_function,
            x0,
            args=(node_setups, measurements, z_fixed_km),
            method="trf",
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
    pos_sol = np.array([state[0], state[1], z_fixed_km])
    vel_sol = state[2:5]

    # Compute RMS delay and doppler
    delay_residuals = []
    doppler_residuals = []
    for m in measurements:
        ns = node_setups[m.node_id]
        pred_d = bistatic_delay(pos_sol, ns.tx_enu, ns.rx_enu)
        pred_f = bistatic_doppler(pos_sol, vel_sol, ns.tx_enu, ns.rx_enu, ns.fc_hz)
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
        "vel_east": float(vel_sol[0]),
        "vel_north": float(vel_sol[1]),
        "vel_up": float(vel_sol[2]),
        "rms_delay": rms_delay,
        "rms_doppler": rms_doppler,
        "n_nodes": len(node_setups),
        "contributing_node_ids": list({m.node_id for m in measurements}),
        "n_measurements": len(measurements),
        "cost": float(result.cost),
        "timestamp_ms": solver_input.get("timestamp_ms", 0),
    }
