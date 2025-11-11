"""
Wall & STL placement with explicit STL axis/origin handling.
- Endpoints (A,B) lie on the wall FACE that looks toward the world origin.
- Wall is a box of given width (thickness).
- STL's BACK is placed flush on that face and yaw-aligned.
- Prints SDF-ready <pose>x y z rx ry rz</pose> for the wall and STL.
"""

import math
import numpy as np
import pymap3d as pm

# ---------------- Inputs ----------------
# ENU reference (Gazebo world origin)
lat0, lon0, h0 = 41.358389, 2.185278, 0.0

# Two geodetic points ON the front face of the wall
latA, lonA, hA  = 41.358508333, 2.185419444, 0.0
latB, lonB, hB  = 41.358491667, 2.185413889, 0.0

# STL origin somewhere near that face (we’ll snap it)
lat1A, lon1A, h1A  = 41.358508333, 2.185419444, -5.0

# Wall geometry
wall_width  = 3.0     # thickness (face-to-face)
wall_height = 10.0
bottom_z    = -10.0

# ====== STL assumptions you MUST set correctly ======
# Which axis of the STL points "forward" out of its front face?
stl_front_axis = "+X"   # change to "+Y", "-Y", "+Z", "-Z" if needed

# How far is the STL origin IN FRONT of its back face, along the front axis? (m)
# If origin at center and STL thickness along the front axis is 0.30 m -> use 0.15
stl_back_offset_from_origin = 0.0

# Small nudge to make it visibly flush (positive -> into the wall a hair)
flush_epsilon = 0.0

# ---------------- Helpers ----------------
def geodetic_to_enu(lat, lon, h):
    e, n, u = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
    return np.array([e, n, u], dtype=float)

def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v

def yaw_from_vec_xy(vxy):
    return math.atan2(vxy[1], vxy[0])

# Map STL front axis to a unit vector in the STL link frame before yaw is applied.
# We only control yaw here, so we assume STL's local Z (up) and Y follow a right-handed frame.
axis_map = {
    "+X": np.array([1, 0, 0], float),
    "-X": np.array([-1, 0, 0], float),
    "+Y": np.array([0, 1, 0], float),
    "-Y": np.array([0, -1, 0], float),
    "+Z": np.array([0, 0, 1], float),
    "-Z": np.array([0, 0, -1], float),
}
if stl_front_axis not in axis_map:
    raise ValueError("stl_front_axis must be one of +X,-X,+Y,-Y,+Z,-Z")
front_axis_local = axis_map[stl_front_axis]

# ---------------- Convert inputs ----------------
A = geodetic_to_enu(latA, lonA, hA)
B = geodetic_to_enu(latB, lonB, hB)
Axy, Bxy = A[:2], B[:2]
M_face_xy = 0.5 * (Axy + Bxy)

t_hat = unit(Bxy - Axy)                       # tangent along the wall (A->B)
n_hat = np.array([-t_hat[1], t_hat[0]])       # +90° in XY

# Make +n_hat point AWAY from origin so the face looks toward origin
if np.dot(n_hat, M_face_xy) < 0.0:
    n_hat = -n_hat

length = float(np.linalg.norm(Bxy - Axy))
center_xy = M_face_xy + (wall_width/2.0) * n_hat
wall_yaw = yaw_from_vec_xy(t_hat)
wall_z   = bottom_z + wall_height/2.0

# ---------------- STL placement ----------------
P = geodetic_to_enu(lat1A, lon1A, h1A)
Pxy = P[:2]

# Project origin onto the A->B line, then onto the face plane (the side closer to origin)
AB = Bxy - Axy
ab_len2 = float(np.dot(AB, AB))
tau = 0.0 if ab_len2 == 0 else float(np.dot(Pxy - Axy, AB) / ab_len2)
tau = max(0.0, min(1.0, tau))
on_line_xy = Axy + tau * AB

# Put exactly on the face plane:
# Keep along-tangent coordinate relative to M_face_xy; set normal coordinate to face plane
def scalar_proj(p, d): return float(np.dot(p, d))  # d must be unit
s_t = scalar_proj(on_line_xy - M_face_xy, t_hat)
face_xy = M_face_xy + s_t * t_hat - (wall_width/2.0) * n_hat

# Determine yaw so that the STL's **back** faces the wall:
# Back direction = -front. We want BACK parallel to +n_hat (toward wall).
# So FRONT must be along -n_hat projected in XY.
front_dir_xy = -n_hat
stl_yaw = yaw_from_vec_xy(front_dir_xy)

# Apply the offset: move the STL origin TOWARD the wall by the distance its origin sits in front of the back face.
# If your origin is at center, use half-thickness; if already at back face, use 0.0.
stl_xy = face_xy + (stl_back_offset_from_origin - flush_epsilon) * n_hat
stl_z  = P[2]

# ---------------- Diagnostics ----------------
# Remaining signed gap along wall normal (should be near -flush_epsilon)
gap = float(np.dot(stl_xy - face_xy, n_hat))  # >0 means still away from wall
print(f"# DIAG: gap along wall normal (m): {gap:.4f} (target ≈ {-flush_epsilon:.4f})")
print(f"# DIAG: wall_yaw={wall_yaw:.6f}, stl_yaw={stl_yaw:.6f}")

# ---------------- SDF-ready outputs ----------------
print("\n# --- WALL (box) ---")
print(f"<size>{length:.3f} {wall_width:.3f} {wall_height:.3f}</size>")
print(f"<pose>{center_xy[0]:.6f} {center_xy[1]:.6f} {wall_z:.6f} 0 0 {wall_yaw:.6f}</pose>")

print("\n# --- STL ELEMENT ORIGIN (back flush on wall face) ---")
print(f"<pose>{stl_xy[0]:.6f} {stl_xy[1]:.6f} {stl_z:.6f} 0 0 {stl_yaw:.6f}</pose>")