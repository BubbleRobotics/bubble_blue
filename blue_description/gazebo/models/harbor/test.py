"""
Convert up to 10 geodetic endpoints (A..J) to ENU and print them
in the same style as the current "# A_face: e n u" lines.

Outputs (per point) look like:
# A_face: <E> <N> <U>
"""

import numpy as np
import pymap3d as pm

# ---------- World origin (ENU reference) in geodetic ----------
lat0, lon0, h0 = 41.358389, 2.185278, 0.0

# ---------- Inputs: up to 10 endpoints in geodetic ----------
# Fill the ones you need; leave others as None
latA, lonA, hA  = 41.358508333, 2.185419444, 0.0
latB, lonB, hB  = 41.358491667, 2.185413889, 0.0
latC, lonC, hC  = 41.358475, 2.185408333, 0.0
latD, lonD, hD  = 41.358455556, 2.1854, 0.0
latE, lonE, hE  = 41.358438889, 2.185394444, 0.0
latF, lonF, hF  = 41.358413889, 2.185383333, 0.0
latG, lonG, hG  = 41.358397222, 2.185377778, 0.0
latH, lonH, hH  = 41.358377778, 2.185369444, 0.0
latI, lonI, hI  = None, None, None
latJ, lonJ, hJ  = None, None, None

# ---------- Helpers ----------
def geodetic_to_enu(lat, lon, h, lat0, lon0, h0):
    e, n, u = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
    return np.array([e, n, u], dtype=float)

# Bundle provided points into a dict: label -> (lat, lon, h) or None
points_geo = {
    "A": (latA, lonA, hA),
    "B": (latB, lonB, hB),
    "C": (latC, lonC, hC),
    "D": (latD, lonD, hD),
    "E": (latE, lonE, hE),
    "F": (latF, lonF, hF),
    "G": (latG, lonG, hG),
    "H": (latH, lonH, hH),
    "I": (latI, lonI, hI),
    "J": (latJ, lonJ, hJ),
}

print("# Face endpoints in ENU (should match what you see in Gazebo if model is at world origin):")
for label, triple in points_geo.items():
    if triple is None or any(v is None for v in triple):
        continue  # skip unset points
    lat, lon, h = triple
    E, N, U = geodetic_to_enu(lat, lon, h, lat0, lon0, h0)
    print(f"# {label}_face: {E:.6f} {N:.6f} {U:.6f}")
