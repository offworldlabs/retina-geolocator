#!/usr/bin/env python3
"""
3D Track Processing Script

Processes passive radar track data using 3D Levenberg-Marquardt solver.
State: [x, y, z, vx, vy, vz] in ENU coordinates with full 3D optimization.

Usage:
    python process_tracks_3d.py input_tracks.jsonl
    python process_tracks_3d.py input_tracks.jsonl --output results.jsonl
"""

import sys
import json
import argparse
from pathlib import Path
import numpy as np

# Add lib to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from lib import (
    Geometry,
    load_geolocator_config,
    load_config,
    load_tracks,
    Detection,
    Track,
    calculate_baseline_geometry,
    generate_initial_guess,
    solve_track
)


def enu_to_lla(enu_km, rx_lat, rx_lon, rx_alt):
    """Convert ENU (km) to LLA coordinates."""
    enu_m = tuple(x * 1000 for x in enu_km)
    ecef = Geometry.enu2ecef(enu_m[0], enu_m[1], enu_m[2], rx_lat, rx_lon, rx_alt)
    lla = Geometry.ecef2lla(ecef[0], ecef[1], ecef[2])
    return lla


def process_event(event, geo_config, radar_config, geometry, tx_enu, previous_solutions):
    """Process a single track event."""
    track_id = event.get('track_id')
    track_number = event.get('track_number')
    total_length = event.get('length', 0)

    # Parse detections
    detections = []
    for det_dict in event.get('detections', []):
        det = Detection(
            timestamp=det_dict['timestamp'],
            delay=det_dict['delay'],
            doppler=det_dict['doppler'],
            snr=det_dict.get('snr', 0)
        )
        detections.append(det)

    n_det = len(detections)

    # Skip if insufficient detections
    if n_det < geo_config.min_detections:
        return None

    # Create track
    track = Track(track_id, detections, event)

    # Generate initial guess
    use_previous = False
    if geo_config.temporal_continuity and track_number in previous_solutions:
        # Use previous solution
        prev_state = previous_solutions[track_number]
        initial_guess = prev_state
        use_previous = True
    else:
        # Generate new initial guess
        initial_guess = generate_initial_guess(
            track,
            tx_enu,
            geometry['antenna_boresight'],
            radar_config.frequency,
            altitude_km=geo_config.initial_altitude_m / 1000
        )

    # Solve track
    result = solve_track(
        track,
        initial_guess,
        tx_enu,
        (0, 0, 0),  # RX at origin
        radar_config.frequency,
        geometry['antenna_boresight'],
        radar_config.rx_alt,
        beamwidth_deg=geo_config.beamwidth_deg
    )

    # Store solution for next event with this track number
    if result['success']:
        previous_solutions[track_number] = result['final_state']

    # Convert final position to LLA
    final_enu = result['final_state'][:3]
    final_lla = enu_to_lla(final_enu, radar_config.rx_lat, radar_config.rx_lon, radar_config.rx_alt)

    # Quality flag based on track length
    if n_det < 5:
        quality = "very_short"
    elif n_det < 10:
        quality = "short"
    elif n_det < 20:
        quality = "medium"
    else:
        quality = "full"

    # Calculate duration
    t_start = detections[0].timestamp
    t_end = detections[-1].timestamp
    duration_s = (t_end - t_start) / 1000.0

    # Build output
    output = {
        'track_id': track_id,
        'track_number': track_number,
        'track_length': total_length,
        'n_detections': n_det,
        'quality_flag': quality,
        'duration_s': duration_s,
        'timestamp_start': t_start,
        'timestamp_end': t_end,
        'latitude': final_lla[0],
        'longitude': final_lla[1],
        'altitude': final_lla[2],
        'velocity_east': result['final_state'][3],
        'velocity_north': result['final_state'][4],
        'velocity_up': result['final_state'][5],
        'rms_delay_us': result['rms_delay'],
        'rms_doppler_hz': result['rms_doppler'],
        'cost': result['cost'],
        'success': result['success'],
        'used_previous_solution': use_previous,
        'message': result['message'],
        'nfev': result['nfev']
    }

    return output


def main():
    parser = argparse.ArgumentParser(description='Process passive radar tracks with 3D solver')
    parser.add_argument('input', help='Input JSONL file with track data')
    parser.add_argument('--output', help='Output JSONL file (default: data/output/results_3d.jsonl)')
    parser.add_argument('--config', default='geolocator_config.yml', help='Geolocator config file')
    args = parser.parse_args()

    # Load configurations
    print("Loading configurations...")
    geo_config = load_geolocator_config(args.config)

    radar_config = load_config(
        primary_path=geo_config.primary_config_path,
        fallback_path=geo_config.fallback_config_path
    )

    print(f"  {radar_config}")
    print(f"  {geo_config}")
    print()

    # Calculate baseline geometry
    geometry = calculate_baseline_geometry(radar_config.rx_lla, radar_config.tx_lla)
    print(f"Baseline: {geometry['baseline_distance']:.2f} km at {geometry['baseline_azimuth']:.2f}°")
    print(f"Antenna boresight: {geometry['antenna_boresight']:.2f}° (beamwidth {geo_config.beamwidth_deg}°)")
    print()

    # Convert TX to ENU
    tx_ecef = Geometry.lla2ecef(*radar_config.tx_lla)
    tx_enu_m = Geometry.ecef2enu(tx_ecef[0], tx_ecef[1], tx_ecef[2], *radar_config.rx_lla)
    tx_enu = tuple(x / 1000 for x in tx_enu_m)

    # Setup output
    output_path = args.output if args.output else Path(geo_config.output_directory) / 'results_3d.jsonl'
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)

    # Process tracks
    print(f"Processing tracks from {args.input}...")
    print(f"Output: {output_path}")
    print()

    previous_solutions = {}
    n_processed = 0
    n_success = 0

    with open(args.input, 'r') as f_in, open(output_path, 'w') as f_out:
        for line in f_in:
            event = json.loads(line)
            result = process_event(event, geo_config, radar_config, geometry, tx_enu, previous_solutions)

            if result is not None:
                n_processed += 1
                if result['success']:
                    n_success += 1

                # Write immediately (streaming output)
                f_out.write(json.dumps(result) + '\n')
                f_out.flush()

                if geo_config.verbose and n_processed % 10 == 0:
                    print(f"Processed {n_processed} tracks ({n_success} successful)", end='\r')

    print()
    print(f"\nDone! Processed {n_processed} tracks, {n_success} successful ({100*n_success/n_processed if n_processed > 0 else 0:.1f}%)")


if __name__ == '__main__':
    main()
