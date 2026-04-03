"""
Spatula Orientation Visual Check
==================================
Opens Isaac Sim with the wxai spatula robot at the retract config so you can
visually confirm whether the spatula blade is face-up or face-down.

Usage:
    /home/shyam/workspace/isaacsim-5.1.0/python.sh scripts/verify_spatula_orientation.py

What to look for:
  - If the spatula blade faces UP  → flat_orientation_wxyz = [1, 0, 0, 0]  (already set)
  - If the spatula blade faces DOWN → flat_orientation_wxyz = [0, 1, 0, 0]  (revert)
"""

import os
import sys
import numpy as np

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

import omni.timeline
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SPATULA_USD = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "robots/assets/wxai/wxai_follower_spatula.usd",
)
ROBOT_SCENE_PATH = "/World/robot"

# Retract config — same as eval_config.yaml retract_config
# joint_0..5 = arm joints, then carriage joints (gripper open = 0.022)
RETRACT_JOINTS = [0.0, 0.5, 0.5, 0.0, 0.0, 0.0]

# ---------------------------------------------------------------------------
# Build scene
# ---------------------------------------------------------------------------
stage_utils.create_new_stage(template="sunlight")

stage_utils.add_reference_to_stage(usd_path=SPATULA_USD, path=ROBOT_SCENE_PATH)

# Ground plane
stage_utils.add_reference_to_stage(
    usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
    path="/World/ground",
)

robot = Articulation(ROBOT_SCENE_PATH)

# ---------------------------------------------------------------------------
# Play
# ---------------------------------------------------------------------------
omni.timeline.get_timeline_interface().play()
simulation_app.update()
simulation_app.update()

# ---------------------------------------------------------------------------
# Print DOF names so we know the joint ordering in this USD
# ---------------------------------------------------------------------------
try:
    dof_names = robot.get_dof_names()
    print("\n" + "="*60)
    print("  DOF names in this USD (index : name)")
    print("="*60)
    for i, name in enumerate(dof_names):
        print(f"    {i}: {name}")
    print()

    # Build position array: set arm joints to retract, leave rest at 0
    n_dofs = len(dof_names)
    positions = np.zeros(n_dofs)
    for i, name in enumerate(dof_names):
        joint_map = {
            "joint_0": 0.0,
            "joint_1": 0.5,
            "joint_2": 0.5,
            "joint_3": 0.0,
            "joint_4": 0.0,
            "joint_5": 0.0,
            "left_carriage_joint":  0.022,
            "right_carriage_joint": 0.022,
        }
        if name in joint_map:
            positions[i] = joint_map[name]

    robot.set_joint_positions(positions.reshape(1, -1))
    print(f"  Set joints: {dict(zip(dof_names, positions.round(3)))}")

except Exception as e:
    print(f"  [warn] Could not set joint positions via names: {e}")
    print("  Trying by index (assuming joint_0..5 = DOF 0..5)...")
    try:
        positions = np.zeros(8)
        positions[:6] = RETRACT_JOINTS
        positions[6:] = 0.022
        robot.set_joint_positions(positions.reshape(1, -1))
    except Exception as e2:
        print(f"  [warn] Also failed: {e2}")
        print("  Joint positions may not be set — check visually.")

# Let physics settle
for _ in range(30):
    simulation_app.update()

print()
print("="*60)
print("  LOOK AT THE SPATULA BLADE in the Isaac Sim window.")
print()
print("  Face UP   → keep flat_orientation_wxyz = [1, 0, 0, 0]")
print("  Face DOWN → change it to              = [0, 1, 0, 0]")
print()
print("  Close the Isaac Sim window when done.")
print("="*60)

# ---------------------------------------------------------------------------
# Keep window open
# ---------------------------------------------------------------------------
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
