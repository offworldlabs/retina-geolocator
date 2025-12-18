#!/usr/bin/env python3
"""Test dual-mode initial guess selection."""

import sys
from pathlib import Path

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent / 'lib'))

from lib.config_loader import Detection, Track
from lib.initial_guess_single import select_initial_guess
import numpy as np


class MockConfig:
    """Mock configuration object for testing."""
    def __init__(self, use_adsb=True, adsb_fallback=True):
        self.use_adsb_initial_guess = use_adsb
        self.adsb_fallback_to_geometric = adsb_fallback


def test_adsb_mode():
    """Test that ADS-B mode is selected when data available."""
    print("Testing ADS-B mode selection...")

    # Create track with ADS-B data
    adsb_data = {
        'lat': 37.85,
        'lon': -122.40,
        'alt_baro': 5000,
        'gs': 250,
        'track': 90
    }
    event_data = {
        'track_id': '250618-A12345',
        'adsb_hex': 'a12345',
        'adsb_initialized': True
    }
    det = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_data)
    track = Track("250618-A12345", [det], event_data)

    # Test parameters
    tx_enu = (50, 0, 0.783)  # TX position in km
    boresight_vector = np.array([0.8, 0.6, 0])
    frequency = 503e6
    config = MockConfig(use_adsb=True, adsb_fallback=True)
    rx_lla = (37.7644, -122.3954, 23)

    # Call router
    guess, source = select_initial_guess(
        track, tx_enu, boresight_vector, frequency, config, rx_lla
    )

    assert guess is not None, "Should return valid guess"
    assert source == "adsb", f"Should select ADS-B mode, got {source}"
    assert len(guess) == 6, "Should return 6-element state vector"

    x, y, z, vx, vy, vz = guess
    print(f"  ADS-B guess: pos=({x:.3f}, {y:.3f}, {z:.3f}) km, vel=({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s")
    print(f"  ✓ Selected source: {source}")


def test_geometric_mode_no_adsb():
    """Test that geometric mode is selected when no ADS-B data."""
    print("\nTesting geometric mode (no ADS-B)...")

    # Create track without ADS-B
    det = Detection(1718747745000, 16.1, 134.5, 18.2)
    track = Track("250618-000001", [det])

    # Test parameters
    tx_enu = (50, 0, 0.783)
    boresight_vector = np.array([0.8, 0.6, 0])
    frequency = 503e6
    config = MockConfig(use_adsb=True, adsb_fallback=True)
    rx_lla = (37.7644, -122.3954, 23)

    # Call router
    guess, source = select_initial_guess(
        track, tx_enu, boresight_vector, frequency, config, rx_lla
    )

    assert guess is not None, "Should return valid guess"
    assert source == "geometric", f"Should select geometric mode, got {source}"
    assert len(guess) == 6, "Should return 6-element state vector"

    print(f"  ✓ Selected source: {source}")


def test_adsb_disabled():
    """Test that geometric mode is selected when ADS-B disabled."""
    print("\nTesting ADS-B disabled in config...")

    # Create track with ADS-B data
    adsb_data = {
        'lat': 37.85,
        'lon': -122.40,
        'alt_baro': 5000,
        'gs': 250,
        'track': 90
    }
    event_data = {
        'track_id': '250618-A12345',
        'adsb_hex': 'a12345',
        'adsb_initialized': True
    }
    det = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_data)
    track = Track("250618-A12345", [det], event_data)

    # Test parameters
    tx_enu = (50, 0, 0.783)
    boresight_vector = np.array([0.8, 0.6, 0])
    frequency = 503e6
    config = MockConfig(use_adsb=False, adsb_fallback=True)  # ADS-B disabled
    rx_lla = (37.7644, -122.3954, 23)

    # Call router
    guess, source = select_initial_guess(
        track, tx_enu, boresight_vector, frequency, config, rx_lla
    )

    assert guess is not None, "Should return valid guess"
    assert source == "geometric", f"Should use geometric when disabled, got {source}"

    print(f"  ✓ Selected source: {source} (ADS-B disabled)")


def test_fallback_to_geometric():
    """Test fallback when ADS-B guess fails."""
    print("\nTesting fallback to geometric...")

    # Create track with incomplete ADS-B (missing lat, will cause generate_adsb_initial_guess to return None)
    adsb_invalid = {'lon': -122.40, 'alt_baro': 5000}  # Missing 'lat'
    event_data = {
        'track_id': '250618-000002',
        'adsb_hex': 'a00002',
        'adsb_initialized': True
    }
    det = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_invalid)
    track = Track("250618-000002", [det], event_data)

    # Test parameters
    tx_enu = (50, 0, 0.783)
    boresight_vector = np.array([0.8, 0.6, 0])
    frequency = 503e6
    config = MockConfig(use_adsb=True, adsb_fallback=True)
    rx_lla = (37.7644, -122.3954, 23)

    # Call router - should fallback to geometric
    guess, source = select_initial_guess(
        track, tx_enu, boresight_vector, frequency, config, rx_lla
    )

    assert guess is not None, "Should return valid guess"
    assert source == "geometric", f"Should fallback to geometric, got {source}"

    print(f"  ✓ Fell back to: {source}")


def test_fallback_disabled_raises():
    """Test that fallback disabled raises error when ADS-B fails."""
    print("\nTesting fallback disabled (should raise)...")

    # Create track with incomplete ADS-B (missing lat)
    adsb_invalid = {'lon': -122.40, 'alt_baro': 5000}  # Missing 'lat'
    event_data = {
        'track_id': '250618-000003',
        'adsb_hex': 'a00003',
        'adsb_initialized': True
    }
    det = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_invalid)
    track = Track("250618-000003", [det], event_data)

    # Test parameters
    tx_enu = (50, 0, 0.783)
    boresight_vector = np.array([0.8, 0.6, 0])
    frequency = 503e6
    config = MockConfig(use_adsb=True, adsb_fallback=False)  # Fallback disabled
    rx_lla = (37.7644, -122.3954, 23)

    # Call router - should raise ValueError
    try:
        guess, source = select_initial_guess(
            track, tx_enu, boresight_vector, frequency, config, rx_lla
        )
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "ADS-B initial guess failed" in str(e)
        print(f"  ✓ Raised ValueError as expected: {str(e)[:60]}...")


def test_backward_compatibility():
    """Test backward compatibility with config without ADS-B fields."""
    print("\nTesting backward compatibility (no ADS-B config fields)...")

    # Create mock config without ADS-B fields
    class OldConfig:
        pass

    # Create track without ADS-B
    det = Detection(1718747745000, 16.1, 134.5, 18.2)
    track = Track("250618-000004", [det])

    # Test parameters
    tx_enu = (50, 0, 0.783)
    boresight_vector = np.array([0.8, 0.6, 0])
    frequency = 503e6
    config = OldConfig()  # No ADS-B fields
    rx_lla = (37.7644, -122.3954, 23)

    # Call router - should use geometric (default behavior)
    guess, source = select_initial_guess(
        track, tx_enu, boresight_vector, frequency, config, rx_lla
    )

    assert guess is not None, "Should return valid guess"
    assert source == "geometric", f"Should use geometric with old config, got {source}"

    print(f"  ✓ Backward compatible: {source}")


def test_track_not_adsb_initialized():
    """Test that geometric is used when track not ADS-B initialized."""
    print("\nTesting track without adsb_initialized flag...")

    # Create track with ADS-B data but not initialized flag
    adsb_data = {
        'lat': 37.85,
        'lon': -122.40,
        'alt_baro': 5000,
        'gs': 250,
        'track': 90
    }
    det = Detection(1718747745000, 16.1, 134.5, 18.2, adsb=adsb_data)
    track = Track("250618-000005", [det])  # No event_data, adsb_initialized=False

    # Test parameters
    tx_enu = (50, 0, 0.783)
    boresight_vector = np.array([0.8, 0.6, 0])
    frequency = 503e6
    config = MockConfig(use_adsb=True, adsb_fallback=True)
    rx_lla = (37.7644, -122.3954, 23)

    # Call router - should use geometric
    guess, source = select_initial_guess(
        track, tx_enu, boresight_vector, frequency, config, rx_lla
    )

    assert guess is not None, "Should return valid guess"
    assert source == "geometric", f"Should use geometric when not initialized, got {source}"

    print(f"  ✓ Used geometric: track not ADS-B initialized")


if __name__ == '__main__':
    test_adsb_mode()
    test_geometric_mode_no_adsb()
    test_adsb_disabled()
    test_fallback_to_geometric()
    test_fallback_disabled_raises()
    test_backward_compatibility()
    test_track_not_adsb_initialized()

    print("\n" + "="*50)
    print("All tests passed! ✓")
    print("="*50)
