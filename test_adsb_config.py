#!/usr/bin/env python3
"""Test ADS-B configuration options."""

import sys
import yaml
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'lib'))

from config_loader import GeolocatorConfig, load_geolocator_config


def test_default_config():
    """Test that default configuration includes ADS-B options."""
    print("Testing default configuration...")

    config = load_geolocator_config('geolocator_config.yml')

    assert hasattr(config, 'use_adsb_initial_guess'), "Missing use_adsb_initial_guess"
    assert hasattr(config, 'adsb_fallback_to_geometric'), "Missing adsb_fallback_to_geometric"
    assert hasattr(config, 'validate_against_adsb'), "Missing validate_against_adsb"

    assert config.use_adsb_initial_guess == True, "Default use_adsb should be True"
    assert config.adsb_fallback_to_geometric == True, "Default fallback should be True"
    assert config.validate_against_adsb == True, "Default validate should be True"

    print(f"  {config}")
    print("  ✓ Default config loaded correctly")


def test_custom_config():
    """Test custom ADS-B configuration values."""
    print("\nTesting custom configuration...")

    config_dict = {
        'solver': {
            'use_adsb_initial_guess': False,
            'adsb_fallback_to_geometric': False,
            'validate_against_adsb': False
        }
    }

    config = GeolocatorConfig(config_dict)

    assert config.use_adsb_initial_guess == False
    assert config.adsb_fallback_to_geometric == False
    assert config.validate_against_adsb == False

    print(f"  {config}")
    print("  ✓ Custom values parsed correctly")


def test_backward_compatibility():
    """Test that configs without ADS-B fields use defaults."""
    print("\nTesting backward compatibility...")

    config_dict = {
        'solver': {
            'beamwidth_deg': 48,
            'min_detections': 3
        }
    }

    config = GeolocatorConfig(config_dict)

    assert config.use_adsb_initial_guess == True, "Should default to True"
    assert config.adsb_fallback_to_geometric == True, "Should default to True"
    assert config.validate_against_adsb == True, "Should default to True"

    print(f"  {config}")
    print("  ✓ Backward compatibility verified")


def test_invalid_type_use_adsb():
    """Test validation rejects invalid use_adsb_initial_guess type."""
    print("\nTesting invalid type for use_adsb_initial_guess...")

    config_dict = {
        'solver': {
            'use_adsb_initial_guess': 'yes'
        }
    }

    try:
        config = GeolocatorConfig(config_dict)
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "use_adsb_initial_guess must be boolean" in str(e)
        print(f"  ✓ Raised ValueError: {str(e)}")


def test_invalid_type_fallback():
    """Test validation rejects invalid adsb_fallback_to_geometric type."""
    print("\nTesting invalid type for adsb_fallback_to_geometric...")

    config_dict = {
        'solver': {
            'adsb_fallback_to_geometric': 1
        }
    }

    try:
        config = GeolocatorConfig(config_dict)
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "adsb_fallback_to_geometric must be boolean" in str(e)
        print(f"  ✓ Raised ValueError: {str(e)}")


def test_invalid_type_validate():
    """Test validation rejects invalid validate_against_adsb type."""
    print("\nTesting invalid type for validate_against_adsb...")

    config_dict = {
        'solver': {
            'validate_against_adsb': None
        }
    }

    try:
        config = GeolocatorConfig(config_dict)
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "validate_against_adsb must be boolean" in str(e)
        print(f"  ✓ Raised ValueError: {str(e)}")


def test_config_combinations():
    """Test various valid combinations of ADS-B settings."""
    print("\nTesting configuration combinations...")

    combinations = [
        (True, True, True, "Full ADS-B enabled"),
        (True, False, True, "ADS-B no fallback"),
        (False, True, False, "ADS-B disabled"),
        (False, False, False, "All disabled")
    ]

    for use_adsb, fallback, validate, desc in combinations:
        config_dict = {
            'solver': {
                'use_adsb_initial_guess': use_adsb,
                'adsb_fallback_to_geometric': fallback,
                'validate_against_adsb': validate
            }
        }

        config = GeolocatorConfig(config_dict)

        assert config.use_adsb_initial_guess == use_adsb
        assert config.adsb_fallback_to_geometric == fallback
        assert config.validate_against_adsb == validate

        print(f"  ✓ {desc}: adsb={use_adsb}, fallback={fallback}, validate={validate}")


def test_yaml_round_trip():
    """Test loading from actual YAML file."""
    print("\nTesting YAML round trip...")

    yaml_content = """
solver:
  beamwidth_deg: 48
  min_detections: 3
  use_adsb_initial_guess: true
  adsb_fallback_to_geometric: false
  validate_against_adsb: true
"""

    with tempfile.NamedTemporaryFile(mode='w', suffix='.yml', delete=False) as f:
        f.write(yaml_content)
        temp_path = f.name

    try:
        config = load_geolocator_config(temp_path)

        assert config.use_adsb_initial_guess == True
        assert config.adsb_fallback_to_geometric == False
        assert config.validate_against_adsb == True

        print(f"  {config}")
        print("  ✓ YAML loading verified")
    finally:
        Path(temp_path).unlink()


def test_repr_includes_adsb():
    """Test that __repr__ includes ADS-B status."""
    print("\nTesting __repr__ includes ADS-B...")

    config_dict = {'solver': {'use_adsb_initial_guess': True}}
    config = GeolocatorConfig(config_dict)

    repr_str = repr(config)
    assert 'adsb=True' in repr_str, "Should include adsb status in repr"

    print(f"  {repr_str}")
    print("  ✓ __repr__ includes ADS-B status")


if __name__ == '__main__':
    test_default_config()
    test_custom_config()
    test_backward_compatibility()
    test_invalid_type_use_adsb()
    test_invalid_type_fallback()
    test_invalid_type_validate()
    test_config_combinations()
    test_yaml_round_trip()
    test_repr_includes_adsb()

    print("\n" + "="*50)
    print("All tests passed! ✓")
    print("="*50)
