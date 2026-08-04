"""
Baseline geometry calculations for single-node passive radar.
Computes TX-RX baseline azimuth, antenna boresight direction, and beam pattern.
"""

import numpy as np
from .Geometry import Geometry


def calculate_baseline_geometry(rx_lla, tx_lla, beam_azimuth_deg=None):
    """
    Calculate baseline geometry between receiver and transmitter.

    Args:
        rx_lla: (lat, lon, alt) of receiver in degrees and meters
        tx_lla: (lat, lon, alt) of transmitter in degrees and meters
        beam_azimuth_deg: the node's configured antenna aim, if it declares
            one.  When absent the boresight defaults to broadside —
            (baseline_azimuth + 90) % 360 — matching
            retina_analytics.constants.resolve_beam_azimuth_deg, so the two
            libraries no longer give different answers for the same node.
            (This replaces a hardcoded pick-the-option-closest-to-225°
            heuristic from one specific south-west-facing deployment, which
            silently mis-aimed every node whose antenna faced anywhere else.)

    Returns:
        dict with:
            - baseline_azimuth: azimuth from RX to TX in degrees (0-360)
            - baseline_distance: distance in km
            - antenna_boresight: azimuth of the antenna aim in degrees
            - antenna_boresight_vector: unit vector in ENU frame
    """
    # Convert TX position to ENU relative to RX
    # First convert both to ECEF, then TX to ENU relative to RX
    tx_ecef = Geometry.lla2ecef(tx_lla[0], tx_lla[1], tx_lla[2])
    tx_enu = Geometry.ecef2enu(tx_ecef[0], tx_ecef[1], tx_ecef[2],
                                 rx_lla[0], rx_lla[1], rx_lla[2])

    # Calculate azimuth from RX to TX (ENU is in meters, convert to km)
    east, north, up = tx_enu
    east_km, north_km, up_km = east/1000, north/1000, up/1000

    baseline_azimuth = np.degrees(np.arctan2(east, north))
    if baseline_azimuth < 0:
        baseline_azimuth += 360

    # Calculate baseline distance
    baseline_distance = np.sqrt(east_km**2 + north_km**2 + up_km**2)  # km

    # Antenna boresight: configured aim when the node declares one, otherwise
    # broadside to the baseline — the same convention as
    # retina_analytics.constants.resolve_beam_azimuth_deg.
    if beam_azimuth_deg is not None and np.isfinite(beam_azimuth_deg):
        antenna_boresight = float(beam_azimuth_deg) % 360
    else:
        antenna_boresight = (baseline_azimuth + 90) % 360

    # Convert boresight azimuth to unit vector in ENU
    az_rad = np.radians(antenna_boresight)
    # Assume horizontal beam (elevation = 0)
    antenna_boresight_vector = np.array([
        np.sin(az_rad),  # East
        np.cos(az_rad),  # North
        0.0              # Up
    ])

    return {
        'baseline_azimuth': baseline_azimuth,
        'baseline_distance': baseline_distance,
        'antenna_boresight': antenna_boresight,
        'antenna_boresight_vector': antenna_boresight_vector
    }


def antenna_gain_pattern(target_azimuth, boresight_azimuth, beamwidth_deg=48):
    """
    Calculate antenna gain for a target at given azimuth.
    Uses Gaussian beam pattern.

    Args:
        target_azimuth: azimuth to target in degrees (0-360)
        boresight_azimuth: antenna boresight azimuth in degrees (0-360)
        beamwidth_deg: 3dB beamwidth in degrees (default 48 for 12-element yagi)

    Returns:
        gain: relative gain (0-1), where 1 is on boresight, 0.5 is at 3dB points
    """
    # Calculate angle from boresight
    angle_diff = abs(target_azimuth - boresight_azimuth)
    if angle_diff > 180:
        angle_diff = 360 - angle_diff

    # Gaussian pattern: G(θ) = exp(-θ²/(2σ²))
    # At 3dB point (G=0.5): θ = beamwidth/2
    # 0.5 = exp(-(beamwidth/2)²/(2σ²))
    # ln(0.5) = -(beamwidth/2)²/(2σ²)
    # σ = beamwidth / (2 * sqrt(-2*ln(0.5))) = beamwidth / 2.355
    sigma = beamwidth_deg / 2.355

    gain = np.exp(-(angle_diff**2) / (2 * sigma**2))

    return gain


def calculate_target_azimuth(target_enu):
    """
    Calculate azimuth from RX to target.

    Args:
        target_enu: (east, north, up) position in km

    Returns:
        azimuth in degrees (0-360)
    """
    east, north, up = target_enu
    azimuth = np.degrees(np.arctan2(east, north))
    if azimuth < 0:
        azimuth += 360
    return azimuth


if __name__ == "__main__":
    # Test with config values
    rx_lla = (33.93918, -84.65191, 290)  # meters
    tx_lla = (33.75667, -84.33184, 488)  # meters

    geometry = calculate_baseline_geometry(rx_lla, tx_lla)

    print("Baseline Geometry:")
    print(f"  RX→TX azimuth: {geometry['baseline_azimuth']:.2f}°")
    print(f"  Baseline distance: {geometry['baseline_distance']:.2f} km")
    print(f"  Antenna boresight: {geometry['antenna_boresight']:.2f}°")
    print(f"  Boresight vector (ENU): {geometry['antenna_boresight_vector']}")

    # Test antenna pattern
    print("\nAntenna Gain Pattern (48° beamwidth):")
    for angle_off in [0, 10, 15, 24, 30, 45, 60, 90]:
        test_az = (geometry['antenna_boresight'] + angle_off) % 360
        gain = antenna_gain_pattern(test_az, geometry['antenna_boresight'])
        gain_db = 10 * np.log10(gain) if gain > 0 else -np.inf
        print(f"  {angle_off:3d}° off boresight: gain = {gain:.3f} ({gain_db:+.1f} dB)")
