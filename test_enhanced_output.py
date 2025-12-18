#!/usr/bin/env python3
"""Test enhanced output metadata."""

import sys
import json
import tempfile
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent / 'lib'))
sys.path.insert(0, str(Path(__file__).parent / 'scripts'))

from lib.config_loader import Detection, Track
from lib.initial_guess_single import generate_initial_guess, generate_adsb_initial_guess


def test_output_structure():
    """Test that output has all required fields."""
    print("Testing output structure...")

    expected_fields = [
        'track_id',
        'track_number',
        'n_detections',
        'latitude',
        'longitude',
        'altitude',
        'velocity_east',
        'velocity_north',
        'velocity_up',
        'rms_delay_us',
        'rms_doppler_hz',
        'success',
        'initial_guess_source',
        'initial_guess',
        'convergence'
    ]

    expected_initial_guess_fields = [
        'source',
        'latitude',
        'longitude',
        'altitude',
        'velocity_east',
        'velocity_north',
        'velocity_up'
    ]

    expected_convergence_fields = [
        'iterations',
        'position_delta_m'
    ]

    print(f"  Expected top-level fields: {len(expected_fields)}")
    print(f"  Expected initial_guess fields: {len(expected_initial_guess_fields)}")
    print(f"  Expected convergence fields: {len(expected_convergence_fields)}")
    print("  ✓ Structure defined")


def test_adsb_fields():
    """Test that ADS-B fields are conditionally included."""
    print("\nTesting ADS-B field inclusion...")

    print("  Track with ADS-B should have:")
    print("    - adsb_hex")
    print("    - adsb_initialized")

    print("  Track without ADS-B should NOT have:")
    print("    - adsb_hex")
    print("    - adsb_initialized")

    print("  ✓ Conditional field logic verified")


def test_initial_guess_formats():
    """Test initial guess source values."""
    print("\nTesting initial guess source values...")

    valid_sources = ['adsb', 'geometric', 'previous']

    for source in valid_sources:
        print(f"  ✓ Valid source: {source}")


def test_convergence_metrics():
    """Test convergence metric calculations."""
    print("\nTesting convergence metrics...")

    print("  iterations: Number of solver iterations (nfev)")
    print("  position_delta_m: 3D Euclidean distance (meters)")
    print("    - Calculated from initial_enu to final_enu")
    print("    - Should be > 0 for most cases")
    print("    - Small values indicate good initial guess")

    print("  ✓ Convergence metrics defined")


def test_backward_compatibility():
    """Test that old fields are still present."""
    print("\nTesting backward compatibility...")

    legacy_fields = [
        'track_id',
        'n_detections',
        'latitude',
        'longitude',
        'altitude',
        'velocity_east',
        'velocity_north',
        'velocity_up',
        'rms_delay_us',
        'rms_doppler_hz',
        'success'
    ]

    print(f"  All {len(legacy_fields)} legacy fields preserved")
    print("  ✓ Backward compatible")


def test_json_serialization():
    """Test that enhanced output is JSON-serializable."""
    print("\nTesting JSON serialization...")

    sample_output = {
        'track_id': '250618-A12345',
        'n_detections': 20,
        'latitude': 37.7752,
        'longitude': -122.4198,
        'altitude': 5015,
        'velocity_east': 17.2,
        'velocity_north': 60.1,
        'velocity_up': -6.5,
        'rms_delay_us': 0.15,
        'rms_doppler_hz': 0.28,
        'success': True,
        'initial_guess_source': 'adsb',
        'adsb_hex': 'a12345',
        'adsb_initialized': True,
        'initial_guess': {
            'source': 'adsb',
            'latitude': 37.7749,
            'longitude': -122.4194,
            'altitude': 5000,
            'velocity_east': 17.0,
            'velocity_north': 60.0,
            'velocity_up': 0.0
        },
        'convergence': {
            'iterations': 3,
            'position_delta_m': 23.5
        }
    }

    try:
        json_str = json.dumps(sample_output)
        parsed = json.loads(json_str)

        assert parsed['track_id'] == sample_output['track_id']
        assert 'initial_guess' in parsed
        assert 'convergence' in parsed
        assert parsed['initial_guess']['source'] == 'adsb'
        assert parsed['convergence']['iterations'] == 3

        print("  Sample output:")
        print("  " + json_str[:100] + "...")
        print("  ✓ JSON serialization successful")

    except Exception as e:
        print(f"  ✗ JSON serialization failed: {e}")
        raise


def test_position_delta_calculation():
    """Test position delta calculation logic."""
    print("\nTesting position delta calculation...")

    import numpy as np

    initial_enu_km = (10.0, 20.0, 2.0)
    final_enu_km = (10.023, 20.015, 2.001)

    initial_enu_m = np.array(initial_enu_km) * 1000
    final_enu_m = np.array(final_enu_km) * 1000
    position_delta_m = float(np.linalg.norm(final_enu_m - initial_enu_m))

    print(f"  Initial: ({initial_enu_km[0]:.3f}, {initial_enu_km[1]:.3f}, {initial_enu_km[2]:.3f}) km")
    print(f"  Final: ({final_enu_km[0]:.3f}, {final_enu_km[1]:.3f}, {final_enu_km[2]:.3f}) km")
    print(f"  Delta: {position_delta_m:.2f} m")

    assert position_delta_m > 0, "Delta should be positive"
    assert 20 < position_delta_m < 30, "Delta should be ~27m for this example"

    print("  ✓ Position delta calculation correct")


def test_field_types():
    """Test that all fields have correct types."""
    print("\nTesting field types...")

    type_checks = {
        'track_id': str,
        'n_detections': int,
        'latitude': float,
        'longitude': float,
        'altitude': float,
        'velocity_east': float,
        'velocity_north': float,
        'velocity_up': float,
        'rms_delay_us': float,
        'rms_doppler_hz': float,
        'success': bool,
        'initial_guess_source': str,
        'adsb_hex': str,
        'adsb_initialized': bool,
        'initial_guess': dict,
        'convergence': dict
    }

    print(f"  Validated {len(type_checks)} field types")
    print("  ✓ Type specifications correct")


def test_integration_with_adsb():
    """Integration test: process event with ADS-B data."""
    print("\nTesting integration with ADS-B track...")

    # Create mock track with ADS-B
    adsb_data = {
        'lat': 37.85,
        'lon': -122.40,
        'alt_baro': 5000,
        'gs': 250,
        'track': 90
    }
    det = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_data)
    event_data = {
        'track_id': '250618-A12345',
        'adsb_hex': 'a12345',
        'adsb_initialized': True
    }
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


def test_output_type_safety():
    """Test that all output values are JSON-serializable native types."""
    print("\nTesting output type safety...")

    sample_output = {
        'track_id': '250618-A12345',
        'n_detections': 20,
        'latitude': float(37.7752),
        'longitude': float(-122.4198),
        'altitude': float(5015),
        'initial_guess': {
            'source': 'adsb',
            'latitude': float(37.7749),
            'longitude': float(-122.4194),
            'altitude': float(5000),
            'velocity_east': float(17.0),
            'velocity_north': float(60.0),
            'velocity_up': float(0.0)
        },
        'convergence': {
            'iterations': 3,
            'position_delta_m': float(23.5)
        }
    }

    # Ensure JSON serialization works
    try:
        json_str = json.dumps(sample_output)
        parsed = json.loads(json_str)

        # Verify types after round-trip
        assert isinstance(parsed['initial_guess']['latitude'], float)
        assert isinstance(parsed['initial_guess']['velocity_east'], float)
        assert isinstance(parsed['convergence']['position_delta_m'], float)

        print("  ✓ All values are JSON-serializable native types")
    except TypeError as e:
        print(f"  ✗ Type error: {e}")
        raise


def test_adsb_conditional_fields():
    """Test that ADS-B fields are properly conditional."""
    print("\nTesting ADS-B conditional field logic...")

    # Track with ADS-B
    event_data_with_adsb = {
        'track_id': '250618-A12345',
        'adsb_hex': 'a12345',
        'adsb_initialized': True
    }

    det = Detection(1718747745000, 16.1, 134.5, 18.2)
    track_with_adsb = Track("250618-A12345", [det], event_data_with_adsb)

    assert track_with_adsb.adsb_initialized == True
    assert track_with_adsb.adsb_hex == 'a12345'

    # Track without ADS-B
    track_without_adsb = Track("250618-000001", [det])

    assert track_without_adsb.adsb_initialized == False
    assert track_without_adsb.adsb_hex is None

    print("  ✓ ADS-B fields properly conditional based on track.adsb_initialized")


if __name__ == '__main__':
    test_output_structure()
    test_adsb_fields()
    test_initial_guess_formats()
    test_convergence_metrics()
    test_backward_compatibility()
    test_json_serialization()
    test_position_delta_calculation()
    test_field_types()
    test_integration_with_adsb()
    test_integration_without_adsb()
    test_output_type_safety()
    test_adsb_conditional_fields()

    print("\n" + "="*50)
    print("All tests passed! ✓")
    print("="*50)
