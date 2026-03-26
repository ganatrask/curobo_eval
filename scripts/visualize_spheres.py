"""
Collision Sphere Visualization
================================
Opens Isaac Sim with the wxai spatula robot and overlays cuRobo's
collision spheres at the retract configuration so you can verify coverage.

What to check:
  - Red spheres should hug all arm links (no large gaps)
  - Gripper and spatula area should have adequate coverage
  - No spheres should float far away from the robot mesh

Usage:
    /home/shyam/workspace/isaacsim-5.1.0/python.sh scripts/visualize_spheres.py
"""

import os
import sys
import numpy as np
import yaml
import torch

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

import omni.usd
import omni.timeline
from pxr import UsdGeom, UsdShade, Gf, Sdf
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from eval_utils import load_robot_config

# ---------------------------------------------------------------------------
# Load config
# ---------------------------------------------------------------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)

with open(os.path.join(project_dir, "config/eval_config.yaml")) as f:
    cfg = yaml.safe_load(f)

ROBOT_FILE = cfg["robot"]["config_file"]
if not os.path.isabs(ROBOT_FILE):
    ROBOT_FILE = os.path.join(project_dir, ROBOT_FILE)
USD_PATH   = cfg["robot"]["usd_path"]
RETRACT    = cfg["robot"]["retract_config"]
ROBOT_PATH = "/World/robot"

# ---------------------------------------------------------------------------
# Build stage
# ---------------------------------------------------------------------------
stage_utils.create_new_stage(template="sunlight")
stage_utils.add_reference_to_stage(usd_path=USD_PATH, path=ROBOT_PATH)
stage_utils.add_reference_to_stage(
    usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
    path="/World/ground",
)
robot = Articulation(ROBOT_PATH)

# ---------------------------------------------------------------------------
# Compute sphere positions via cuRobo FK
# ---------------------------------------------------------------------------
tensor_args = TensorDeviceType()
robot_cfg_dict = load_robot_config(ROBOT_FILE)

robot_cfg = RobotConfig.from_dict(robot_cfg_dict["robot_cfg"])
kin_model = CudaRobotModel(robot_cfg.kinematics)

q = torch.tensor([RETRACT], dtype=torch.float32, device="cuda:0")
spheres_list = kin_model.get_robot_as_spheres(q)
all_spheres = spheres_list[0]

print(f"\n{'='*55}")
print(f"  Drawing {len(all_spheres)} collision spheres")
print(f"{'='*55}")
for s in all_spheres:
    print(f"    {s.name:30s}  r={s.radius:.4f}m  "
          f"pos=[{s.position[0]:.3f}, {s.position[1]:.3f}, {s.position[2]:.3f}]")
print()

# ---------------------------------------------------------------------------
# Draw spheres in USD (semi-transparent red)
# ---------------------------------------------------------------------------
stage = omni.usd.get_context().get_stage()
scope_path = "/World/collision_spheres"
stage.DefinePrim(scope_path, "Scope")

# Shared material
mat_path = scope_path + "/sphere_mat"
material = UsdShade.Material.Define(stage, mat_path)
shader   = UsdShade.Shader.Define(stage, mat_path + "/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor",    Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.15, 0.15))
shader.CreateInput("opacity",         Sdf.ValueTypeNames.Float).Set(0.35)
shader.CreateInput("opacityThreshold",Sdf.ValueTypeNames.Float).Set(0.0)
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

for i, s in enumerate(all_spheres):
    x, y, z = float(s.position[0]), float(s.position[1]), float(s.position[2])
    r       = float(s.radius)

    sph_path = f"{scope_path}/sphere_{i}"
    sph_geom = UsdGeom.Sphere.Define(stage, sph_path)
    sph_geom.GetRadiusAttr().Set(r)

    xf = UsdGeom.Xformable(sph_geom.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

    UsdShade.MaterialBindingAPI(sph_geom.GetPrim()).Bind(material)

# ---------------------------------------------------------------------------
# Play and set robot to retract config
# ---------------------------------------------------------------------------
omni.timeline.get_timeline_interface().play()
simulation_app.update()
simulation_app.update()

try:
    dof_names = robot.get_dof_names()
    positions = np.zeros(len(dof_names))
    joint_map = {
        "joint_0": RETRACT[0], "joint_1": RETRACT[1], "joint_2": RETRACT[2],
        "joint_3": RETRACT[3], "joint_4": RETRACT[4], "joint_5": RETRACT[5],
        "left_carriage_joint":  0.022,
        "right_carriage_joint": 0.022,
    }
    for idx, name in enumerate(dof_names):
        if name in joint_map:
            positions[idx] = joint_map[name]
    robot.set_joint_positions(positions.reshape(1, -1))
except Exception as e:
    print(f"  [warn] Could not set joint positions: {e}")

for _ in range(30):
    simulation_app.update()

# ---------------------------------------------------------------------------
# Keep window open
# ---------------------------------------------------------------------------
print("="*55)
print("  VERIFY in the Isaac Sim window:")
print()
print("  GOOD: red spheres hug all arm links tightly")
print("  BAD:  large gaps between spheres and mesh")
print("  BAD:  spheres floating far from the robot")
print()
print("  Close the window when done.")
print("="*55)

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
