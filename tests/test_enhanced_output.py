#!/usr/bin/env python3
"""Test enhanced output metadata."""

import numpy as np

from retina_geolocator.config_loader import Detection, Track
from retina_geolocator.initial_guess_single import generate_adsb_initial_guess, generate_initial_guess


def test_integration_with_adsb():
    """Integration test: process event with ADS-B data."""
    print("\nTesting integration with ADS-B track...")

    # Create mock track with ADS-B
    adsb_data = {"lat": 37.85, "lon": -122.40, "alt_baro": 5000, "gs": 250, "track": 90}
    det = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_data)
    event_data = {"track_id": "250618-A12345", "adsb_hex": "a12345", "adsb_initialized": True}
    track = Track("250618-A12345", [det], event_data)

    # Generate initial guess
    rx_lla = (37.7644, -122.3954, 23)
    initial_guess = generate_adsb_initial_guess(track, rx_lla, None)

    assert initial_guess is not None, "Should generate ADS-B initial guess"
    assert len(initial_guess) == 6, "Should have 6 elements"

    # Verify structure
    x, y, z, vx, vy, vz = initial_guess
    assert isinstance(x, (int, float)), "Position should be numeric"
    assert isinstance(vx, (int, float)), "Velocity should be numeric"

    print(f"  Initial guess: pos=({x:.3f}, {y:.3f}, {z:.3f}) km")
    print(f"  Initial vel: v=({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s")
    print("  ✓ Integration test with ADS-B passed")


def test_integration_without_adsb():
    """Integration test: process event without ADS-B data."""
    print("\nTesting integration without ADS-B...")

    # Create track without ADS-B
    det = Detection(1718747745000, 16.1, 134.5, 18.2)
    track = Track("250618-000001", [det])

    # TX position and parameters
    tx_enu = (50, 0, 0.783)
    boresight_vector = np.array([0.8, 0.6, 0])
    frequency = 503e6

    # Generate geometric initial guess
    initial_guess = generate_initial_guess(track, tx_enu, boresight_vector, frequency)

    assert initial_guess is not None, "Should generate geometric initial guess"
    assert len(initial_guess) == 6, "Should have 6 elements"

    # Verify structure
    x, y, z, vx, vy, vz = initial_guess
    assert isinstance(x, (int, float)), "Position should be numeric"
    assert abs(z - 2.0) < 0.5, "Default altitude should be ~2 km"

    print(f"  Initial guess: pos=({x:.3f}, {y:.3f}, {z:.3f}) km")
    print("  ✓ Integration test without ADS-B passed")


# Deleted as vacuous: test_output_structure, test_adsb_fields, test_initial_guess_formats, test_convergence_metrics, test_backward_compatibility, test_json_serialization, test_position_delta_calculation, test_field_types, test_output_type_safety, test_adsb_conditional_fields.
# None of them called the code under test — they printed field-name
# lists or asserted arithmetic on their own literals, so they could
# not fail.  The two integration tests above are the file's real
# coverage (generate_adsb_initial_guess / generate_initial_guess).
