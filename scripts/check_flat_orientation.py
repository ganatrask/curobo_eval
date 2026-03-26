"""
Check what quaternion spatula_tip reports for "level spatula" configurations.

This runs cuRobo FK only — no Isaac Sim GUI needed.
Run with: omni_python scripts/check_flat_orientation.py

Purpose: Find the correct flat_orientation_wxyz to put in eval_config.yaml.
A "level" spatula means the blade is horizontal (spatula pointing forward,
roughly parallel to the ground plane).
"""

import os
import sys
import torch
import numpy as np

# --- bootstrap (same pattern as other scripts) ---
try:
    import isaacsim  # noqa: F401
except ImportError:
    pass

script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
sys.path.insert(0, script_dir)

import yaml
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.util_file import get_robot_configs_path, join_path, load_yaml

tensor_args = TensorDeviceType()

with open(os.path.join(project_dir, "config/eval_config.yaml"), "r") as f:
    cfg = yaml.safe_load(f)

robot_file = cfg["robot"]["config_file"]
robot_cfg_dict = load_yaml(robot_file)
robot_cfg = RobotConfig.from_dict(robot_cfg_dict["robot_cfg"])
kin_model = CudaRobotModel(robot_cfg.kinematics)

ee_link = robot_cfg_dict["robot_cfg"]["kinematics"]["ee_link"]
print(f"\nRobot:   {os.path.basename(robot_file)}")
print(f"EE link: {ee_link}")
print()


def fk(joints, label=""):
    """Run FK and print the ee position + quaternion (wxyz)."""
    q = torch.tensor([joints], dtype=torch.float32, device="cuda:0")
    state = kin_model.get_state(q)
    pos  = state.ee_position.cpu().numpy()[0]
    quat = state.ee_quaternion.cpu().numpy()[0]  # wxyz

    # Convert to roll/pitch/yaw for easier interpretation
    w, x, y, z = quat
    roll  = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1)))
    yaw   = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

    print(f"  [{label}]")
    print(f"    joints:  {[f'{v:.3f}' for v in joints]}")
    print(f"    pos:     x={pos[0]:.3f}  y={pos[1]:.3f}  z={pos[2]:.3f}  m")
    print(f"    quat:    [{w:.4f}, {x:.4f}, {y:.4f}, {z:.4f}]  (wxyz)")
    print(f"    rpy:     roll={roll:.1f}°  pitch={pitch:.1f}°  yaw={yaw:.1f}°")
    print(f"    => level if |roll|≈180° or 0° AND |pitch|≈0°")
    print()
    return quat


# -----------------------------------------------------------------------
# Candidate configs to test.
#
# For spatula to be level (horizontal), the arm must reach forward-and-down
# so the spatula tip points roughly horizontally.  The wxai joint axes are:
#   joint_0: Z (base yaw)
#   joint_1: Y (shoulder pitch, 0=up, π=down)
#   joint_2: -Y (elbow, 0=straight, 2.35=max bend)
#   joint_3: -Y (forearm pitch)
#   joint_4: -Z (wrist yaw)
#   joint_5: X  (wrist roll)
# -----------------------------------------------------------------------
print("=" * 60)
print("  FK check — candidate 'level spatula' configurations")
print("=" * 60)
print()

candidates = {
    "retract (home)":          [0.0, 0.5, 0.5, 0.0,  0.0, 0.0],
    "reach-forward-flat":      [0.0, 1.0, 1.0, 0.0,  0.0, 0.0],
    "reach-forward-lower":     [0.0, 1.2, 1.5, 0.0,  0.0, 0.0],
    "extended-low":            [0.0, 1.4, 1.8, 0.0,  0.0, 0.0],
    "j3-tilt +0.5":            [0.0, 1.0, 1.0, 0.5,  0.0, 0.0],
    "j3-tilt -0.5":            [0.0, 1.0, 1.0,-0.5,  0.0, 0.0],
    "j5-roll 90°":             [0.0, 1.0, 1.0, 0.0,  0.0, 1.571],
    "j5-roll 180°":            [0.0, 1.0, 1.0, 0.0,  0.0, 3.14159],
}

results = {}
for label, joints in candidates.items():
    results[label] = fk(joints, label)

print("=" * 60)
print("  Interpretation guide")
print("=" * 60)
print("""
  A 'level spatula' means the spatula blade is horizontal.
  Look for configs where:
    - pitch ≈ 0°  (blade not tilted fore/aft)
    - roll ≈ 0° or ±180°  (blade not rolled sideways)

  The quaternion from the best-looking config is your flat_orientation_wxyz.

  Round to 2 decimal places and set in eval_config.yaml:
    flat_orientation_wxyz: [qw, qx, qy, qz]

  TIP: If no candidate is perfectly level, adjust joint_3 (forearm pitch)
  until pitch ≈ 0° — it's the most direct lever for spatula tilt.
""")
