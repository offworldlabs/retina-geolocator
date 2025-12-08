# Retina Geolocator

Single-node passive radar geolocation using bistatic delay/Doppler measurements with antenna beam constraints and temporal diversity.

## Overview

This package implements geolocation for a **single-node passive radar system**. Unlike multi-sensor systems that require 3+ simultaneous detections from different sensors, this solver uses:
- **Temporal diversity**: Multiple detections over time from one sensor
- **Bistatic measurements**: Delay (range) and Doppler (velocity)
- **Antenna constraints**: Directional Yagi antenna beam pattern
- **Motion model**: Constant velocity trajectory fitting

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Copy your radar system config (if not in /opt/blah2/config/)
cp /path/to/your/config.yml data/config/radar_config.yml

# Process tracks with 3D solver
python scripts/process_tracks_3d.py data/input/tracks.jsonl

# Output written to data/output/results_3d.jsonl
```

## System Requirements

### Hardware Configuration
- **Single receiver node** with:
  - Reference antenna (receives direct TX→RX signal)
  - Surveillance antenna (12-element Yagi)
  - Coherent dual-channel SDR (e.g., RSPduo)
- **Illuminator of Opportunity (IoO)**: FM broadcast tower
- **Baseline**: TX-RX separation (typically 10-50 km)

### Software Dependencies
- Python 3.7+
- NumPy >= 1.20.0
- SciPy >= 1.7.0
- PyYAML >= 5.4.0
- Matplotlib >= 3.3.0 (for plotting)

## Configuration

### Geolocator Configuration (`geolocator_config.yml`)

Controls solver behavior:
```yaml
solver:
  beamwidth_deg: 48            # Yagi antenna beamwidth
  altitude_bounds: [50, 15000] # Min/max altitude (meters ASL)
  velocity_bounds: [-400, 400] # Min/max velocity (m/s)
  min_detections: 3            # Minimum detections per track
  temporal_continuity: true    # Use previous solution as initial guess

radar_config:
  primary: /opt/blah2/config/config.yml      # Production path
  fallback: data/config/radar_config.yml     # Local fallback
```

### Radar System Configuration

The solver needs radar system parameters (RX/TX positions, frequency). It will try:
1. **Primary path**: `/opt/blah2/config/config.yml` (production system)
2. **Fallback path**: `data/config/radar_config.yml` (local copy)

If neither exists, copy your system config:
```bash
cp /opt/blah2/config/config.yml data/config/radar_config.yml
```

## Usage

### Process Tracks (3D Solver)

```bash
python scripts/process_tracks_3d.py input_tracks.jsonl --output results.jsonl
```

**Input format** (JSONL - JSON Lines):
```jsonl
{"track_id": "250605-000000", "track_number": "000000", "length": 20, "detections": [...]}
```

Each detection:
```json
{
  "timestamp": 1749190334687,
  "delay": 45.2,
  "doppler": -12.5,
  "snr": 18.3
}
```

**Output format** (JSONL):
```jsonl
{
  "track_id": "250605-000000",
  "n_detections": 20,
  "latitude": 33.92382,
  "longitude": -84.68569,
  "altitude": 1085.5,
  "velocity_east": 17.00,
  "velocity_north": 60.04,
  "velocity_up": -6.23,
  "rms_delay_us": 0.217,
  "rms_doppler_hz": 0.356,
  "success": true
}
```

## Algorithm

### 3D Solver

**State**: `[x, y, z, vx, vy, vz]` in ENU coordinates
- Full 3D position and velocity optimization
- Constant velocity motion model
- Antenna beam constraint (soft)
- Altitude bounds

**Process**:
1. Generate initial guess from bistatic ellipsoid and antenna boresight
2. Levenberg-Marquardt optimization to fit delay/Doppler residuals
3. Output position at last detection time

### Physics Models

**Bistatic Delay**:
```
delay = (|TX→Target| + |Target→RX| - |TX→RX|) / c
```

**Bistatic Doppler**:
```
f_doppler = (f_tx / c) × [v·û_TX + v·û_RX]
```

where v is target velocity, û are unit vectors toward TX/RX.

## Directory Structure

```
retina-geolocator/
├── README.md                      # This file
├── geolocator_config.yml          # Geolocator parameters
├── requirements.txt               # Python dependencies
├── lib/                           # Core library
│   ├── Geometry.py               # Coordinate transforms
│   ├── baseline_geometry.py      # TX-RX baseline calculations
│   ├── bistatic_models.py        # Physics models
│   ├── config_loader.py          # Configuration loading
│   ├── initial_guess_single.py   # Initial guess (3D)
│   ├── initial_guess_2d.py       # Initial guess (2D)
│   ├── lm_solver_track.py        # LM solver (3D)
│   └── lm_solver_track_2d.py     # LM solver (2D)
├── scripts/                       # Processing scripts
│   ├── process_tracks_3d.py      # Main 3D processing
│   └── process_tracks_2d.py      # Main 2D processing (TBD)
├── tools/                         # Analysis utilities (TBD)
└── data/                          # Git-ignored data directory
    ├── input/                     # Input track files
    ├── output/                    # Output solutions
    ├── plots/                     # Generated plots
    └── config/                    # Local radar config copy
```

## Performance

Based on validation with 267 real-world tracks:
- **Success rate**: 99.6% (266/267 tracks)
- **RMS delay error**: 0.32 μs (median)
- **RMS Doppler error**: 0.32 Hz (median)

### Success Criteria
A solution is successful if:
- Solver converges (LM ftol satisfied)
- RMS delay error < 2 μs
- RMS Doppler error < 2 Hz

## Known Limitations

1. **Altitude ambiguity**: Single-node geometry poorly constrains altitude
   - Median altitude: ~1080m, but range 51m - 6000m
   - Use 2D solver with fixed altitude for better horizontal accuracy

2. **Constant velocity assumption**: Fails for maneuvering targets

3. **Linear clustering**: Solutions cluster along antenna boresight due to beam constraint

## Development

### Running Tests
```bash
# Test configuration loading
python -m lib.config_loader

# Process sample data
python scripts/process_tracks_3d.py data/input/sample.jsonl
```

### Adding Custom Scripts
Place analysis scripts in `tools/` directory. Import library:
```python
from lib import load_config, solve_track, Geometry
```

## License

MIT License - See LICENSE file for details

## References

- Passive bistatic radar geolocation
- Levenberg-Marquardt least squares optimization
- WGS-84 geodetic coordinate transformations

## Author

Generated for RETINAsolver single-node passive radar project.
