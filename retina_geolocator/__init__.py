"""
Retina Geolocator Library

Single-node passive radar geolocation using bistatic delay/Doppler measurements
with antenna beam constraints and temporal diversity.
"""

from .baseline_geometry import antenna_gain_pattern, calculate_baseline_geometry, calculate_target_azimuth
from .bistatic_models import bistatic_delay, bistatic_doppler
from .config_loader import Config, Detection, GeolocatorConfig, Track, load_config, load_geolocator_config, load_tracks
from .Geometry import Geometry
from .initial_guess_2d import generate_initial_guess_2d, generate_initial_guess_2d_from_previous
from .initial_guess_single import generate_initial_guess, select_initial_guess
from .lm_solver_track import solve_track
from .lm_solver_track_2d import solve_track_2d, state_to_cartesian_enu, velocities_to_cartesian
from .multinode_solver import solve_multinode

__all__ = [
    "Geometry",
    "calculate_baseline_geometry",
    "antenna_gain_pattern",
    "calculate_target_azimuth",
    "bistatic_delay",
    "bistatic_doppler",
    "Config",
    "GeolocatorConfig",
    "Detection",
    "Track",
    "load_config",
    "load_geolocator_config",
    "load_tracks",
    "generate_initial_guess",
    "select_initial_guess",
    "generate_initial_guess_2d",
    "generate_initial_guess_2d_from_previous",
    "solve_track",
    "solve_track_2d",
    "state_to_cartesian_enu",
    "velocities_to_cartesian",
    "solve_multinode",
]

__version__ = "1.0.0"
