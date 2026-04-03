"""
Fit collision spheres to the spatula mesh.
Uses numpy + struct to read the STL directly (no trimesh dependency).

Usage:
    /home/shyam/workspace/isaacsim-5.1.0/python.sh scripts/fit_spatula_spheres.py

Outputs YAML-ready sphere definitions for the spatula_link.
"""

import struct
import numpy as np
import os

# ---------------------------------------------------------------------------
# STL reader (binary format — no external deps)
# ---------------------------------------------------------------------------
def read_binary_stl(filepath):
    """Read a binary STL file, return (N, 3, 3) array of triangle vertices."""
    with open(filepath, "rb") as f:
        header = f.read(80)
        n_triangles = struct.unpack("<I", f.read(4))[0]
        vertices = []
        for _ in range(n_triangles):
            data = struct.unpack("<12fH", f.read(50))
            # data[0:3] = normal, data[3:12] = 3 vertices (x,y,z each)
            v1 = data[3:6]
            v2 = data[6:9]
            v3 = data[9:12]
            vertices.extend([v1, v2, v3])
    return np.array(vertices).reshape(-1, 3)


# ---------------------------------------------------------------------------
# Load mesh
# ---------------------------------------------------------------------------
_script_dir = os.path.dirname(os.path.abspath(__file__))
_project_dir = os.path.dirname(_script_dir)
STL_PATH = os.path.join(_project_dir, "robots", "meshes", "hloder_with_spatula.stl")
SCALE = 0.001  # STL is in millimeters, robot uses meters

raw_verts = read_binary_stl(STL_PATH)
verts = raw_verts * SCALE  # scale to meters

print(f"Loaded {len(verts)} vertices from {os.path.basename(STL_PATH)}")
print(f"  Raw bounds (mm):  X=[{raw_verts[:,0].min():.1f}, {raw_verts[:,0].max():.1f}]  "
      f"Y=[{raw_verts[:,1].min():.1f}, {raw_verts[:,1].max():.1f}]  "
      f"Z=[{raw_verts[:,2].min():.1f}, {raw_verts[:,2].max():.1f}]")
print(f"  Scaled bounds (m): X=[{verts[:,0].min():.4f}, {verts[:,0].max():.4f}]  "
      f"Y=[{verts[:,1].min():.4f}, {verts[:,1].max():.4f}]  "
      f"Z=[{verts[:,2].min():.4f}, {verts[:,2].max():.4f}]")
print()

# ---------------------------------------------------------------------------
# Deduplicate vertices (STL has 3 copies per triangle)
# ---------------------------------------------------------------------------
verts_unique = np.unique(verts.round(6), axis=0)
print(f"  Unique vertices: {len(verts_unique)}")

# ---------------------------------------------------------------------------
# Fit spheres along X axis (primary axis of the spatula)
# ---------------------------------------------------------------------------
x_min, x_max = verts_unique[:, 0].min(), verts_unique[:, 0].max()
x_length = x_max - x_min
print(f"  Spatula length along X: {x_length:.4f} m ({x_length*1000:.1f} mm)")
print()

# Divide into slices along X
# Handle section: fewer, smaller spheres
# Head section: more, wider spheres
n_slices = 16  # enough resolution
x_positions = np.linspace(x_min + 0.005, x_max - 0.005, n_slices)
slice_width = (x_max - x_min) / n_slices * 1.2  # slight overlap

print("=" * 65)
print(f"  {'X (m)':>8s}  {'Y_range (mm)':>14s}  {'Z_range (mm)':>14s}  "
      f"{'center':>20s}  {'radius':>8s}")
print("=" * 65)

spheres = []
for x in x_positions:
    # Find vertices near this X slice
    mask = np.abs(verts_unique[:, 0] - x) < slice_width
    if mask.sum() < 3:
        continue

    local_pts = verts_unique[mask]

    # Center of the slice
    cy = local_pts[:, 1].mean()
    cz = local_pts[:, 2].mean()

    # Radius = max distance from center in Y-Z plane + small padding
    dists = np.sqrt((local_pts[:, 1] - cy) ** 2 + (local_pts[:, 2] - cz) ** 2)
    radius = dists.max() + 0.003  # 3mm padding

    y_range = (local_pts[:, 1].max() - local_pts[:, 1].min()) * 1000
    z_range = (local_pts[:, 2].max() - local_pts[:, 2].min()) * 1000

    print(f"  {x:8.4f}  {y_range:14.1f}  {z_range:14.1f}  "
          f"[{x:.4f}, {cy:.4f}, {cz:.4f}]  {radius:.4f}")

    spheres.append({
        "center": [round(float(x), 4), round(float(cy), 4), round(float(cz), 4)],
        "radius": round(float(radius), 4),
    })

    # If this slice is wide (>30mm in Y), add side spheres
    if y_range > 30:
        y_min_local = local_pts[:, 1].min()
        y_max_local = local_pts[:, 1].max()
        side_r = y_range / 4 / 1000 + 0.003
        spheres.append({
            "center": [round(float(x), 4), round(float(y_max_local - side_r), 4), round(float(cz), 4)],
            "radius": round(float(side_r), 4),
        })
        spheres.append({
            "center": [round(float(x), 4), round(float(y_min_local + side_r), 4), round(float(cz), 4)],
            "radius": round(float(side_r), 4),
        })

print()
print("=" * 65)
print(f"  Total spheres computed: {len(spheres)}")
print("=" * 65)

# ---------------------------------------------------------------------------
# Output YAML
# ---------------------------------------------------------------------------
print()
print("  Copy this into your collision_spheres YAML under 'spatula_link':")
print()
print("  spatula_link:")
for s in spheres:
    c = s["center"]
    r = s["radius"]
    print(f'    - "center": [{c[0]}, {c[1]}, {c[2]}]')
    print(f'      "radius": {r}')
print()
