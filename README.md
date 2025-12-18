# Retina Geolocator

Single-node passive radar geolocation using bistatic delay/Doppler measurements with antenna beam constraints and temporal diversity.

## Overview

This package implements geolocation for a **single-node passive radar system**. Unlike multi-sensor systems that require 3+ simultaneous detections from different sensors, this solver uses:
- **Temporal diversity**: Multiple detections over time from one sensor
- **Bistatic measurements**: Delay (range) and Doppler (velocity)
- **Antenna constraints**: Directional Yagi antenna beam pattern
- **Motion model**: Constant velocity trajectory fitting

## ADS-B Features

retina-geolocator supports **ADS-B-assisted initial guess generation** when used with retina-tracker output containing ADS-B metadata. This provides significant performance improvements:

- **Faster convergence**: 3-5 iterations vs 10-20 with geometric guess
- **Better altitude initialization**: Start at ADS-B altitude (±100m) vs fixed 2km assumption
- **Velocity initialization**: Use ground truth velocity vs zero initial velocity
- **Validation data**: Compare solved position vs ADS-B truth for quality assessment

### When to Use ADS-B Features

**Enable ADS-B** when:
- Your radar system includes ADS-B receiver (e.g., dump1090, readsb)
- Using retina-tracker with ADS-B integration enabled
- Processing aircraft with Mode S transponders

**Use geometric fallback** when:
- No ADS-B data available (enabled by default via `adsb_fallback_to_geometric`)
- Processing non-cooperative targets
- ADS-B data quality is poor

### Configuration

Enable in `geolocator_config.yml`:
```yaml
solver:
  use_adsb_initial_guess: true        # Enable ADS-B-assisted initial guess
  adsb_fallback_to_geometric: true    # Use geometric guess if no ADS-B
  validate_against_adsb: true         # Include ADS-B comparison in output
```

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

### Quick Start with ADS-B

If using retina-tracker with ADS-B integration:
```bash
# Process tracks with ADS-B assistance (config default)
python scripts/process_tracks_3d.py data/input/tracks_with_adsb.jsonl

# Force geometric-only (disable ADS-B)
# Edit geolocator_config.yml: use_adsb_initial_guess: false
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
  # Antenna configuration
  beamwidth_deg: 48            # Yagi antenna beamwidth

  # Optimization bounds
  altitude_bounds: [50, 15000] # Min/max altitude (meters ASL)
  velocity_bounds: [-400, 400] # Min/max velocity (m/s)

  # Track filtering
  min_detections: 3            # Minimum detections per track
  temporal_continuity: true    # Use previous solution as initial guess

  # ADS-B assisted initial guess (requires retina-tracker with ADS-B)
  use_adsb_initial_guess: true        # Enable ADS-B-based initial guess
  adsb_fallback_to_geometric: true    # Fallback to geometric if unavailable
  validate_against_adsb: true         # Include ADS-B comparison metrics

radar_config:
  primary: /opt/blah2/config/config.yml      # Production path
  fallback: data/config/radar_config.yml     # Local fallback
```

**ADS-B Configuration Options**:
- `use_adsb_initial_guess`: Enable ADS-B-assisted initial guess when ADS-B data is available in track
- `adsb_fallback_to_geometric`: Automatically use geometric guess if ADS-B unavailable or invalid
- `validate_against_adsb`: Add comparison fields to output (distance error, altitude difference, etc.)

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

Each detection (basic format):
```json
{
  "timestamp": 1749190334687,
  "delay": 45.2,
  "doppler": -12.5,
  "snr": 18.3
}
```

**Input format with ADS-B** (from retina-tracker):
```jsonl
{
  "track_id": "250618-A12345",
  "track_number": "A12345",
  "adsb_hex": "a12345",
  "adsb_initialized": true,
  "length": 20,
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
        "track": 45,
        "geom_rate": 0
      }
    }
  ]
}
```

**ADS-B field descriptions**:
- `adsb_hex`: Aircraft ICAO 24-bit address (hex)
- `adsb_initialized`: Track has valid ADS-B data
- `adsb` (per detection, optional):
  - `lat`, `lon`: Position (degrees)
  - `alt_baro`: Barometric altitude (feet)
  - `gs`: Ground speed (knots)
  - `track`: Track angle (degrees, 0=North, 90=East)
  - `geom_rate`: Vertical rate (feet/min, optional)

**Output format** (JSONL - basic):
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

**Output format with ADS-B** (when `validate_against_adsb: true`):
```jsonl
{
  "track_id": "250618-A12345",
  "adsb_hex": "a12345",
  "adsb_initialized": true,
  "n_detections": 20,
  "latitude": 37.77491,
  "longitude": -122.41938,
  "altitude": 1524.3,
  "velocity_east": 90.8,
  "velocity_north": 90.6,
  "velocity_up": -0.2,
  "rms_delay_us": 0.183,
  "rms_doppler_hz": 0.241,
  "initial_guess": "adsb",
  "convergence": {
    "iterations": 4,
    "ftol_satisfied": true
  },
  "adsb_comparison": {
    "distance_error_m": 23.5,
    "horizontal_error_m": 18.2,
    "altitude_error_m": 15.7,
    "velocity_error_ms": 2.1
  },
  "success": true
}
```

**Additional fields with ADS-B**:
- `adsb_hex`: Aircraft identifier
- `adsb_initialized`: Track had ADS-B data available
- `initial_guess`: Source of initial guess (`"adsb"` or `"geometric"`)
- `convergence`: Solver convergence details
  - `iterations`: Number of LM iterations
  - `ftol_satisfied`: Function tolerance convergence
- `adsb_comparison`: Comparison with ADS-B truth (when `validate_against_adsb: true`)
  - `distance_error_m`: 3D distance between solved and ADS-B position
  - `horizontal_error_m`: 2D horizontal distance error
  - `altitude_error_m`: Altitude difference (absolute)
  - `velocity_error_ms`: 3D velocity magnitude difference

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

### ADS-B vs Geometric Initial Guess

Performance comparison with ADS-B data available (based on test suite):

| Metric | ADS-B Initial Guess | Geometric Initial Guess |
|--------|---------------------|-------------------------|
| **Convergence iterations** | 3-5 iterations | 10-20 iterations |
| **Initial altitude error** | ±100m (from ADS-B) | ~1000m (fixed 2km guess) |
| **Initial velocity error** | ±5 m/s (from ADS-B) | 100+ m/s (zero guess) |
| **Solver success rate** | 99.6%+ | 99.6% |
| **Final accuracy** | Same (limited by geometry) | Same (limited by geometry) |

**Key findings**:
- **ADS-B initial guess reduces iterations by 50-70%**, speeding up processing
- **Final accuracy is identical** - both converge to the same solution (limited by single-node geometry)
- **ADS-B provides validation** - compare solved vs ADS-B for quality assessment
- **Geometric fallback ensures robustness** - works with or without ADS-B

### Success Criteria
A solution is successful if:
- Solver converges (LM ftol satisfied)
- RMS delay error < 2 μs
- RMS Doppler error < 2 Hz

## Troubleshooting

### ADS-B Issues

**Problem**: "No ADS-B data available" or `initial_guess: "geometric"` in output

**Solutions**:
- Verify retina-tracker has ADS-B integration enabled
- Check that tracks include `adsb_hex` and `adsb_initialized` fields
- Ensure at least one detection has `adsb` metadata
- If intentional (non-cooperative targets), this is expected behavior

**Problem**: "ADS-B initial guess failed, using geometric fallback"

**Causes**:
- Missing required fields (`lat`, `lon` in ADS-B data)
- Invalid coordinates (NaN, Inf values)
- Coordinate transformation failure

**Solutions**:
- Check ADS-B data quality from retina-tracker
- Verify receiver location (`rx_lla`) in radar config is correct
- Enable `adsb_fallback_to_geometric: true` (default) for automatic fallback

**Problem**: Large `adsb_comparison` errors (>1000m)

**Causes**:
- Poor radar geometry (low SNR, limited baseline)
- Maneuvering aircraft (violates constant velocity assumption)
- Timing mismatch between radar and ADS-B

**Solutions**:
- Check RMS delay/Doppler errors - high values indicate poor radar data quality
- Filter tracks with high comparison errors for manual review
- Consider using 2D solver with ADS-B altitude as constraint

### General Issues

**Problem**: "Could not find radar config at any path"

**Solution**:
```bash
# Copy radar system config to fallback location
cp /opt/blah2/config/config.yml data/config/radar_config.yml
```

**Problem**: Low success rate (<95%)

**Causes**:
- Poor SNR tracks
- Very short tracks (< 5 detections)
- Highly maneuvering targets

**Solutions**:
- Increase `min_detections` threshold
- Filter input tracks by SNR or track length
- Review failed tracks manually

**Problem**: Solutions far from expected position

**Causes**:
- Incorrect TX/RX positions in config
- Wrong frequency in config
- Coordinate system mismatch

**Solutions**:
- Verify radar config matches actual system
- Check coordinate system (WGS-84 lat/lon/alt expected)
- Compare with ADS-B truth if available

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
