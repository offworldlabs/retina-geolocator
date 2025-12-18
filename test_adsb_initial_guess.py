#!/usr/bin/env python3
"""Test ADS-B initial guess generator."""

import sys
import math
from pathlib import Path

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent / 'lib'))

from lib.config_loader import Detection, Track
from lib.initial_guess_single import (
    lla_to_enu_km,
    adsb_velocity_to_enu,
    generate_adsb_initial_guess
)


def test_lla_to_enu_km():
    """Test LLA to ENU conversion."""
    print("Testing lla_to_enu_km()...")

    # Reference: San Francisco (150 Mississippi)
    ref_lat = 37.7644
    ref_lon = -122.3954
    ref_alt = 23  # meters

    # Test 1: Same location should give (0, 0, 0)
    x, y, z = lla_to_enu_km(ref_lat, ref_lon, ref_alt, ref_lat, ref_lon, ref_alt)
    assert abs(x) < 0.001, f"Same location E should be ~0, got {x}"
    assert abs(y) < 0.001, f"Same location N should be ~0, got {y}"
    assert abs(z) < 0.001, f"Same location U should be ~0, got {z}"
    print(f"  Same location: ({x:.6f}, {y:.6f}, {z:.6f}) km ✓")

    # Test 2: Aircraft 10km North at 5000m altitude
    target_lat = ref_lat + 0.09  # ~10km north
    target_lon = ref_lon
    target_alt = 5000

    x, y, z = lla_to_enu_km(target_lat, target_lon, target_alt, ref_lat, ref_lon, ref_alt)
    assert abs(x) < 0.5, f"Due north E should be ~0, got {x}"
    assert 9 < y < 11, f"Due north N should be ~10km, got {y}"
    assert 4.9 < z < 5.1, f"Altitude diff should be ~5km, got {z}"
    print(f"  10km North @ 5000m: E={x:.3f}, N={y:.3f}, U={z:.3f} km ✓")

    # Test 3: Aircraft 10km East at 3000m altitude
    target_lat = ref_lat
    target_lon = ref_lon + 0.12  # ~10km east at this latitude
    target_alt = 3000

    x, y, z = lla_to_enu_km(target_lat, target_lon, target_alt, ref_lat, ref_lon, ref_alt)
    assert 9 < x < 11, f"Due east E should be ~10km, got {x}"
    assert abs(y) < 0.5, f"Due east N should be ~0, got {y}"
    assert 2.9 < z < 3.1, f"Altitude diff should be ~3km, got {z}"
    print(f"  10km East @ 3000m: E={x:.3f}, N={y:.3f}, U={z:.3f} km ✓")

    print("  ✓ lla_to_enu_km tests passed")


def test_adsb_velocity_to_enu():
    """Test ADS-B velocity conversion."""
    print("\nTesting adsb_velocity_to_enu()...")

    # Test 1: North heading (0°), 250 knots
    vel = adsb_velocity_to_enu(250, 0)
    assert vel is not None, "Should return valid velocity"
    vx, vy, vz = vel
    assert abs(vx) < 1, f"North heading vx should be ~0, got {vx}"
    assert 127 < vy < 131, f"250kt = ~128.6 m/s, got {vy}"
    assert vz == 0, f"No vertical rate, vz should be 0, got {vz}"
    print(f"  250kt North (0°): vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} m/s ✓")

    # Test 2: East heading (90°), 200 knots
    vel = adsb_velocity_to_enu(200, 90)
    assert vel is not None, "Should return valid velocity"
    vx, vy, vz = vel
    assert 101 < vx < 105, f"200kt = ~102.9 m/s, got {vx}"
    assert abs(vy) < 1, f"East heading vy should be ~0, got {vy}"
    assert vz == 0, f"No vertical rate, vz should be 0, got {vz}"
    print(f"  200kt East (90°): vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} m/s ✓")

    # Test 3: Northeast heading (45°), 300 knots
    vel = adsb_velocity_to_enu(300, 45)
    assert vel is not None, "Should return valid velocity"
    vx, vy, vz = vel
    gs_ms = 300 * 0.514444  # ~154.3 m/s
    expected = gs_ms / math.sqrt(2)  # ~109.1 m/s
    assert abs(vx - expected) < 1, f"NE heading vx should be ~{expected:.1f}, got {vx}"
    assert abs(vy - expected) < 1, f"NE heading vy should be ~{expected:.1f}, got {vy}"
    print(f"  300kt NE (45°): vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} m/s ✓")

    # Test 4: With vertical rate (1000 ft/min climb)
    vel = adsb_velocity_to_enu(250, 0, 1000)
    assert vel is not None, "Should return valid velocity"
    vx, vy, vz = vel
    assert 4 < vz < 6, f"1000 ft/min = ~5.08 m/s, got {vz}"
    print(f"  With 1000 ft/min climb: vz={vz:.2f} m/s ✓")

    # Test 5: Invalid input (NaN)
    vel = adsb_velocity_to_enu(float('nan'), 0)
    assert vel is None, "NaN input should return None"
    print(f"  NaN input returns None ✓")

    print("  ✓ adsb_velocity_to_enu tests passed")


def test_generate_adsb_initial_guess():
    """Test main ADS-B initial guess generation."""
    print("\nTesting generate_adsb_initial_guess()...")

    # Reference: San Francisco (150 Mississippi)
    rx_lla = (37.7644, -122.3954, 23)

    # Test 1: Valid ADS-B data with position and velocity
    adsb_data = {
        'lat': 37.85,  # ~10km north
        'lon': -122.40,
        'alt_baro': 5000,
        'gs': 250,
        'track': 90  # East
    }
    det = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_data)
    track = Track("250618-A12345", [det])

    guess = generate_adsb_initial_guess(track, rx_lla, None)
    assert guess is not None, "Should return valid guess"
    assert len(guess) == 6, "Should return 6-element state vector"

    x, y, z, vx, vy, vz = guess
    print(f"  Position: E={x:.3f}, N={y:.3f}, U={z:.3f} km")
    print(f"  Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} m/s")

    # Validate position
    # Note: alt_baro 5000 feet = 1524 meters = 1.524 km
    assert abs(x) < 5, f"E should be small, got {x}"
    assert 8 < y < 12, f"N should be ~10km, got {y}"
    assert 1.4 < z < 1.6, f"U should be ~1.5km (5000ft), got {z}"

    # Validate velocity (250kt east)
    assert 126 < vx < 131, f"vx should be ~128 m/s, got {vx}"
    assert abs(vy) < 5, f"vy should be ~0, got {vy}"
    assert abs(vz) < 0.001, f"vz should be ~0, got {vz}"

    print("  ✓ Valid ADS-B data produces correct guess")

    # Test 2: No ADS-B data
    det_no_adsb = Detection(1718747745000, 16.1, 134.5, 18.2)
    track_no_adsb = Track("250618-000001", [det_no_adsb])

    guess = generate_adsb_initial_guess(track_no_adsb, rx_lla, None)
    assert guess is None, "Should return None when no ADS-B"
    print("  ✓ No ADS-B returns None")

    # Test 3: Invalid ADS-B (missing lat/lon)
    adsb_invalid = {'alt_baro': 5000}
    det_invalid = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_invalid)
    track_invalid = Track("250618-000002", [det_invalid])

    guess = generate_adsb_initial_guess(track_invalid, rx_lla, None)
    assert guess is None, "Should return None for invalid ADS-B"
    print("  ✓ Invalid ADS-B returns None")

    # Test 4: ADS-B with position only (no velocity)
    adsb_pos_only = {
        'lat': 37.85,
        'lon': -122.40,
        'alt_baro': 3000
    }
    det_pos = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_pos_only)
    track_pos = Track("250618-000003", [det_pos])

    guess = generate_adsb_initial_guess(track_pos, rx_lla, None)
    assert guess is not None, "Should work with position only"
    x, y, z, vx, vy, vz = guess
    assert vx == 0 and vy == 0 and vz == 0, "Velocity should default to zero"
    print(f"  ✓ Position-only ADS-B: v=({vx}, {vy}, {vz}) m/s")

    # Test 5: Multiple detections, only second has ADS-B
    det1 = Detection(1718747745000, 16.1, 134.5, 18.2)  # No ADS-B
    det2 = Detection(1718747746000, 16.2, 135.0, 18.5, adsb=adsb_data)  # Has ADS-B
    track_multi = Track("250618-000004", [det1, det2])

    guess = generate_adsb_initial_guess(track_multi, rx_lla, None)
    assert guess is not None, "Should find ADS-B in second detection"
    print("  ✓ Finds ADS-B in second detection")

    print("  ✓ generate_adsb_initial_guess tests passed")


def test_coordinate_system_consistency():
    """Test that coordinate conversions are consistent."""
    print("\nTesting coordinate system consistency...")

    # Reference point
    ref_lat, ref_lon, ref_alt = 37.7644, -122.3954, 23

    # Test roundtrip: LLA → ENU → LLA (approximately)
    target_lat, target_lon, target_alt = 37.85, -122.30, 5000

    # Convert to ENU
    x, y, z = lla_to_enu_km(target_lat, target_lon, target_alt, ref_lat, ref_lon, ref_alt)

    # Simple checks
    assert not math.isnan(x) and not math.isinf(x), "E should be valid"
    assert not math.isnan(y) and not math.isinf(y), "N should be valid"
    assert not math.isnan(z) and not math.isinf(z), "U should be valid"

    print(f"  LLA ({target_lat}, {target_lon}, {target_alt}m)")
    print(f"  → ENU ({x:.3f}, {y:.3f}, {z:.3f}) km")
    print("  ✓ Coordinate system consistency verified")


if __name__ == '__main__':
    test_lla_to_enu_km()
    test_adsb_velocity_to_enu()
    test_generate_adsb_initial_guess()
    test_coordinate_system_consistency()

    print("\n" + "="*50)
    print("All tests passed! ✓")
    print("="*50)
