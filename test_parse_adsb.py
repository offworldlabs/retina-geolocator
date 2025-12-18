#!/usr/bin/env python3
"""Test ADS-B metadata parsing."""

import sys
import json
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'lib'))

from config_loader import Detection, Track, validate_adsb_data, load_tracks

def test_validate_adsb_data():
    """Test ADS-B validation function."""
    print("Testing validate_adsb_data()...")

    # Valid ADS-B
    valid = {
        'lat': 37.7749,
        'lon': -122.4194,
        'alt_baro': 5000,
        'gs': 250,
        'track': 45
    }
    assert validate_adsb_data(valid) == True, "Valid ADS-B should pass"

    # Invalid: latitude out of range
    invalid_lat = valid.copy()
    invalid_lat['lat'] = 95
    assert validate_adsb_data(invalid_lat) == False, "Lat > 90 should fail"

    # Invalid: NaN
    import math
    invalid_nan = valid.copy()
    invalid_nan['lat'] = math.nan
    assert validate_adsb_data(invalid_nan) == False, "NaN should fail"

    # Missing required field
    invalid_missing = {'lat': 37.0}
    assert validate_adsb_data(invalid_missing) == False, "Missing lon should fail"

    # Not a dict
    assert validate_adsb_data("not a dict") == False, "Non-dict should fail"

    # Valid with minimal fields
    minimal = {'lat': 37.0, 'lon': -122.0}
    assert validate_adsb_data(minimal) == True, "Minimal valid should pass"

    print("  ✓ All validation tests passed")

def test_detection_with_adsb():
    """Test Detection class with ADS-B."""
    print("\nTesting Detection class...")

    # Without ADS-B
    det1 = Detection(1234567890000, 16.1, 134.5, 18.2)
    assert det1.adsb is None, "Detection without ADS-B should have adsb=None"
    print(f"  Without ADS-B: {det1}")

    # With ADS-B
    adsb = {'lat': 37.7749, 'lon': -122.4194, 'alt_baro': 5000}
    det2 = Detection(1234567890000, 16.1, 134.5, 18.2, adsb=adsb)
    assert det2.adsb == adsb, "Detection with ADS-B should store it"
    print(f"  With ADS-B: {det2}")

    print("  ✓ Detection tests passed")

def test_track_with_adsb():
    """Test Track class with ADS-B metadata."""
    print("\nTesting Track class...")

    # Without ADS-B
    det = Detection(1234567890000, 16.1, 134.5, 18.2)
    track1 = Track("250618-000000", [det])
    assert track1.adsb_hex is None, "Track without ADS-B should have adsb_hex=None"
    assert track1.adsb_initialized == False, "Track without ADS-B should have adsb_initialized=False"
    print(f"  Without ADS-B: {track1}")

    # With ADS-B
    event_data = {
        'track_id': '250618-A12345',
        'adsb_hex': 'a12345',
        'adsb_initialized': True
    }
    track2 = Track("250618-A12345", [det], event_data)
    assert track2.adsb_hex == 'a12345', "Track should parse adsb_hex"
    assert track2.adsb_initialized == True, "Track should parse adsb_initialized"
    print(f"  With ADS-B: {track2}")

    print("  ✓ Track tests passed")

def test_load_tracks_with_adsb():
    """Test loading tracks with ADS-B from JSONL."""
    print("\nTesting load_tracks() with ADS-B data...")

    # Create test JSONL
    test_data = [
        {
            "track_id": "250618-A12345",
            "adsb_hex": "a12345",
            "adsb_initialized": True,
            "n_total": 3,
            "detections": [
                {
                    "timestamp": 1718747745000,
                    "delay": 16.1,
                    "doppler": 134.5,
                    "snr": 18.2,
                    "adsb": {
                        "lat": 37.7749,
                        "lon": -122.4194,
                        "alt_baro": 5000,
                        "gs": 250,
                        "track": 45
                    }
                },
                {
                    "timestamp": 1718747746000,
                    "delay": 16.2,
                    "doppler": 135.0,
                    "snr": 18.5,
                    "adsb": None
                },
                {
                    "timestamp": 1718747747000,
                    "delay": 16.3,
                    "doppler": 135.5,
                    "snr": 18.8
                }
            ]
        }
    ]

    # Write test file
    test_file = Path('/tmp/test_adsb_tracks.jsonl')
    with open(test_file, 'w') as f:
        for event in test_data:
            f.write(json.dumps(event) + '\n')

    # Load tracks
    tracks = load_tracks(str(test_file), min_detections=1)
    assert len(tracks) == 1, "Should load 1 track"

    track = tracks[0]
    assert track.adsb_hex == 'a12345', "Should parse adsb_hex"
    assert track.adsb_initialized == True, "Should parse adsb_initialized"

    # Check detections
    assert len(track.detections) == 3, "Should have 3 detections"
    assert track.detections[0].adsb is not None, "First detection should have ADS-B"
    assert track.detections[1].adsb is None, "Second detection should not have ADS-B"
    assert track.detections[2].adsb is None, "Third detection should not have ADS-B"

    print(f"  Loaded: {track}")
    print(f"  First detection: {track.detections[0]}")

    print("  ✓ load_tracks tests passed")

def test_backward_compatibility():
    """Test that tracks without ADS-B still work."""
    print("\nTesting backward compatibility (no ADS-B)...")

    # Create test JSONL without ADS-B
    test_data = [
        {
            "track_id": "250618-000001",
            "n_total": 2,
            "detections": [
                {
                    "timestamp": 1718747745000,
                    "delay": 20.5,
                    "doppler": -50.2,
                    "snr": 12.1
                },
                {
                    "timestamp": 1718747746000,
                    "delay": 20.8,
                    "doppler": -49.5,
                    "snr": 12.5
                }
            ]
        }
    ]

    # Write test file
    test_file = Path('/tmp/test_no_adsb_tracks.jsonl')
    with open(test_file, 'w') as f:
        for event in test_data:
            f.write(json.dumps(event) + '\n')

    # Load tracks
    tracks = load_tracks(str(test_file), min_detections=1)
    assert len(tracks) == 1, "Should load 1 track"

    track = tracks[0]
    assert track.adsb_hex is None, "Track without ADS-B should have adsb_hex=None"
    assert track.adsb_initialized == False, "Track without ADS-B should have adsb_initialized=False"
    assert all(det.adsb is None for det in track.detections), "All detections should have adsb=None"

    print(f"  Loaded: {track}")

    print("  ✓ Backward compatibility verified")

def test_invalid_adsb_handling():
    """Test that invalid ADS-B data is silently ignored."""
    print("\nTesting invalid ADS-B handling...")

    # Create test JSONL with invalid ADS-B
    test_data = [
        {
            "track_id": "250618-000002",
            "n_total": 2,
            "detections": [
                {
                    "timestamp": 1718747745000,
                    "delay": 25.1,
                    "doppler": 100.5,
                    "snr": 15.2,
                    "adsb": {
                        "lat": 95,  # Invalid: out of range
                        "lon": -122.0,
                        "alt_baro": 3000
                    }
                },
                {
                    "timestamp": 1718747746000,
                    "delay": 25.3,
                    "doppler": 101.0,
                    "snr": 15.5,
                    "adsb": "not a dict"  # Invalid: wrong type
                }
            ]
        }
    ]

    # Write test file
    test_file = Path('/tmp/test_invalid_adsb_tracks.jsonl')
    with open(test_file, 'w') as f:
        for event in test_data:
            f.write(json.dumps(event) + '\n')

    # Load tracks - should not fail, just ignore invalid ADS-B
    tracks = load_tracks(str(test_file), min_detections=1)
    assert len(tracks) == 1, "Should load 1 track"

    track = tracks[0]
    assert all(det.adsb is None for det in track.detections), "Invalid ADS-B should be ignored"

    print(f"  Loaded: {track}")
    print("  ✓ Invalid ADS-B handling verified")

if __name__ == '__main__':
    test_validate_adsb_data()
    test_detection_with_adsb()
    test_track_with_adsb()
    test_load_tracks_with_adsb()
    test_backward_compatibility()
    test_invalid_adsb_handling()

    print("\n" + "="*50)
    print("All tests passed! ✓")
    print("="*50)
