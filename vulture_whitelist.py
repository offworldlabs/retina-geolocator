"""Vulture dead-code whitelist.

Read by tools/check-dead-code.sh. Every name here is one vulture reports as
dead but which must not be deleted — or which nobody has decided about yet.
The distinction matters, so the two live in separate sections.

Add to CONTRACTS only when the name is genuinely referenced by something
vulture cannot see: a framework calling in, a wire format, a config key. Real
dead code should be deleted, not whitelisted.

The UNREVIEWED section is a backlog, not an exemption. Each entry is code that
appears genuinely unreachable and needs a decision — delete it, or wire up
whatever was left unfinished. The gate is green with these listed so that it
starts catching NEW dead code immediately; working through them is separate.
"""
# ruff: noqa: B018, F821
# B018 — bare-name expressions are how vulture whitelists work.
# F821 — these names are defined in other modules; only vulture reads this file.

_ = type("_", (), {})()

# ── Contracts: referenced by something vulture cannot see ─────────────────────
# retina-server's backend/tests/test_wgs84_equivalence.py imports all three
# speed-of-light constants to assert they agree (C_KM_S == C_M_S / 1000.0).
# Nothing inside this repo reads the m/s form, but deleting it breaks that
# cross-repo consistency check — which is the whole point of constants.py.
C_M_S

# ── UNREVIEWED: appears dead, needs a decision (delete, or finish wiring) ──────
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:68  (unused attribute)
_.altitude_bounds
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:223  (unused attribute)
_.cpi
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:89  (unused attribute)
_.fallback_config_path
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:81  (unused attribute)
_.ftol
# TODO: no reference found anywhere in the estate
#   retina_geolocator/initial_guess_single.py:310  (unused function)
generate_multi_start_guesses
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:73  (unused attribute)
_.initial_altitude_m
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:71  (unused attribute)
_.max_detections
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:83  (unused attribute)
_.max_iterations
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:84  (unused attribute)
_.max_rms_delay_us
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:85  (unused attribute)
_.max_rms_doppler_hz
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:93  (unused attribute)
_.output_directory
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:92  (unused attribute)
_.output_format
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:99  (unused attribute)
_.plot_dpi
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:98  (unused attribute)
_.plot_output_dir
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:97  (unused attribute)
_.plotting_enabled
# TODO: no reference found anywhere in the estate
#   retina_geolocator/bistatic_models.py:98  (unused function)
predict_detection
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:88  (unused attribute)
_.primary_config_path
# TODO: no reference found anywhere in the estate
#   retina_geolocator/lm_solver_track.py:91  (unused function)
residual_function
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:202  (unused attribute)
_.rx_name
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:69  (unused attribute)
_.velocity_bounds
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:94  (unused attribute)
_.verbose
# TODO: no reference found anywhere in the estate
#   retina_geolocator/lm_solver_track.py:124  (unused variable)
vx0
# TODO: no reference found anywhere in the estate
#   retina_geolocator/lm_solver_track.py:124  (unused variable)
vy0
# TODO: no reference found anywhere in the estate
#   retina_geolocator/lm_solver_track.py:124  (unused variable)
vz0
# TODO: config field: parsed but no reader found — wire it up or drop it
#   retina_geolocator/config_loader.py:82  (unused attribute)
_.xtol
