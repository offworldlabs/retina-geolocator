"""Physical constants — one home per value.

The speed of light was hand-typed at eight call sites across this library in
three different unit systems; the values all agreed, but nothing enforced it.
Reference these instead.
"""

C_M_S = 299_792_458.0  # m/s
C_KM_S = 299_792.458  # km/s
C_KM_US = 0.299792458  # km/µs
