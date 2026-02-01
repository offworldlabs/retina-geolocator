"""
@file Geometry.py
@brief Re-exports Geometry class from RETINAsolver (canonical implementation).
"""

import os
import sys

_solver_path = os.environ.get(
    "RETINA_SOLVER_PATH",
    os.path.join(os.path.dirname(__file__), "..", "..", "RETINAsolver"),
)
_solver_path = os.path.abspath(_solver_path)
if _solver_path not in sys.path:
    sys.path.insert(0, _solver_path)

from Geometry import Geometry  # noqa: E402, F401
