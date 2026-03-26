"""
Visualize Failed Planning Problems in Isaac Sim
=================================================
For each failure:
  INFEASIBLE  → shows robot at start config + RED goal marker (no motion)
  SOLVER LIM  → re-solves with more seeds, animates the trajectory + YELLOW goal

Usage:
    /home/shyam/workspace/isaacsim-5.1.0/python.sh scripts/visualize_failures.py --phase phase1_baseline
    /home/shyam/workspace/isaacsim-5.1.0/python.sh scripts/visualize_failures.py --phase phase1_baseline --category solver
    /home/shyam/workspace/isaacsim-5.1.0/python.sh scripts/visualize_failures.py --phase phase1_baseline --category infeasible
"""

import os
import sys
import json
import yaml
import time
import numpy as np
import torch
import argparse

parser = argparse.ArgumentParser(description="Visualize failed problems")
parser.add_argument("--phase", type=str, required=True)
parser.add_argument("--category", type=str, default=None,
                    choices=["infeasible", "solver", "all"],
                    help="Filter by category (default: all)")
parser.add_argument("--config", type=str, default="config/eval_config.yaml")
parser.add_argument("--pause", type=float, default=3.0,
                    help="Seconds to pause on infeasible failures (default: 3)")
args = parser.parse_args()

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
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.util_file import load_yaml

# ---------------------------------------------------------------------------
# Load config & diagnosis data
# ---------------------------------------------------------------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
tensor_args = TensorDeviceType()

with open(os.path.join(project_dir, args.config)) as f:
    cfg = yaml.safe_load(f)

# Find diagnosis file
diag_file = None
for d in [os.path.join(project_dir, "results"),
          os.path.join(script_dir, "results")]:
    candidate = os.path.join(d, f"{args.phase}_failure_diagnosis.json")
    if os.path.exists(candidate):
        diag_file = candidate
        break

if not diag_file:
    print(f"ERROR: No diagnosis file found for phase '{args.phase}'.")
    print(f"Run failure_diagnosis.py first.")
    simulation_app.close()
    sys.exit(1)

with open(diag_file) as f:
    diag_data = json.load(f)

failures = diag_data["diagnoses"]

# Filter by category
if args.category == "infeasible":
    failures = [f for f in failures if f["root_cause"].startswith("INFEASIBLE")]
elif args.category == "solver":
    failures = [f for f in failures if f["root_cause"].startswith("SOLVER")]

if not failures:
    print(f"No failures matching category '{args.category}'.")
    simulation_app.close()
    sys.exit(0)

# Check if any solver failures exist (need MotionGen)
has_solver_failures = any(f["root_cause"].startswith("SOLVER") for f in failures)

print(f"\nLoaded {len(failures)} failures to visualize")

# ---------------------------------------------------------------------------
# Build cuRobo MotionGen (only if we have solver failures to re-solve)
# ---------------------------------------------------------------------------
motion_gen = None
if has_solver_failures:
    print("  Setting up cuRobo MotionGen for trajectory re-solving...")
    robot_file = cfg["robot"]["config_file"]
    robot_cfg_dict = load_yaml(robot_file)
    _yaml_dir = os.path.dirname(os.path.abspath(robot_file))
    _kin = robot_cfg_dict.get("robot_cfg", {}).get("kinematics", {})
    _kin["external_asset_path"] = _yaml_dir

    world_cfg = {
        "cuboid": {"_dummy": {"dims": [0.01, 0.01, 0.01],
                               "pose": [100.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]}}
    }

    pcfg = cfg["planning"]
    mg_config = MotionGenConfig.load_from_robot_config(
        robot_cfg_dict, world_cfg, tensor_args,
        interpolation_dt=pcfg["interpolation_dt"],
        num_trajopt_seeds=48,      # extra seeds to re-solve
        num_graph_seeds=24,
        collision_activation_distance=pcfg["collision_activation_distance"],
        use_cuda_graph=False,      # flexible batch sizes
    )
    motion_gen = MotionGen(mg_config)
    motion_gen.warmup(parallel_finetune=True)
    print("  MotionGen ready.")

# ---------------------------------------------------------------------------
# Build scene
# ---------------------------------------------------------------------------
USD_PATH = cfg["robot"]["usd_path"]
ROBOT_PATH = "/World/robot"

stage_utils.create_new_stage(template="sunlight")
stage_utils.add_reference_to_stage(usd_path=USD_PATH, path=ROBOT_PATH)
stage_utils.add_reference_to_stage(
    usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
    path="/World/ground",
)
robot = Articulation(ROBOT_PATH)

# Set camera to look at the robot from a good angle
from pxr import UsdGeom as _UsdGeom, Gf as _Gf
_cam_prim = stage_utils.get_current_stage().GetPrimAtPath("/OmniverseKit_Persp")
if _cam_prim.IsValid():
    _cam_xform = _UsdGeom.Xformable(_cam_prim)
    _cam_xform.ClearXformOpOrder()
    _cam_xform.AddTransformOp().Set(_Gf.Matrix4d().SetLookAt(
        _Gf.Vec3d(0.8, 0.6, 0.5),   # camera position
        _Gf.Vec3d(0.25, 0.0, 0.15),  # look-at target
        _Gf.Vec3d(0, 0, 1),          # up vector
    ).GetInverse())

# Create goal marker
stage = omni.usd.get_context().get_stage()
marker_path = "/World/goal_marker"
marker = UsdGeom.Sphere.Define(stage, marker_path)
marker.GetRadiusAttr().Set(0.02)

# Red material (infeasible)
mat_red_path = "/World/mat_red"
mat_red = UsdShade.Material.Define(stage, mat_red_path)
sh_red = UsdShade.Shader.Define(stage, mat_red_path + "/Shader")
sh_red.CreateIdAttr("UsdPreviewSurface")
sh_red.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.1, 0.1))
sh_red.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.8)
mat_red.CreateSurfaceOutput().ConnectToSource(sh_red.ConnectableAPI(), "surface")

# Yellow material (solver limitation)
mat_yel_path = "/World/mat_yellow"
mat_yel = UsdShade.Material.Define(stage, mat_yel_path)
sh_yel = UsdShade.Shader.Define(stage, mat_yel_path + "/Shader")
sh_yel.CreateIdAttr("UsdPreviewSurface")
sh_yel.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.9, 0.1))
sh_yel.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.8)
mat_yel.CreateSurfaceOutput().ConnectToSource(sh_yel.ConnectableAPI(), "surface")

# Green material (re-solved successfully)
mat_grn_path = "/World/mat_green"
mat_grn = UsdShade.Material.Define(stage, mat_grn_path)
sh_grn = UsdShade.Shader.Define(stage, mat_grn_path + "/Shader")
sh_grn.CreateIdAttr("UsdPreviewSurface")
sh_grn.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.1, 0.9, 0.2))
sh_grn.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.8)
mat_grn.CreateSurfaceOutput().ConnectToSource(sh_grn.ConnectableAPI(), "surface")

# ---------------------------------------------------------------------------
# Play
# ---------------------------------------------------------------------------
omni.timeline.get_timeline_interface().play()
simulation_app.update()
simulation_app.update()

try:
    dof_names = robot.get_dof_names()
except Exception:
    dof_names = None

ARM_JOINTS = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def set_robot_joints(joint_values_6dof):
    """Set the robot to a 6-DOF joint config."""
    if dof_names is None:
        return
    positions = np.zeros(len(dof_names))
    for idx, name in enumerate(dof_names):
        if name in ARM_JOINTS:
            j_idx = ARM_JOINTS.index(name)
            if j_idx < len(joint_values_6dof):
                positions[idx] = joint_values_6dof[j_idx]
        elif name in ("left_carriage_joint", "right_carriage_joint"):
            positions[idx] = 0.022
    robot.set_joint_positions(positions.reshape(1, -1))


def set_goal_marker(pos, material):
    """Move the goal marker and set its material."""
    xf = UsdGeom.Xformable(marker.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    UsdShade.MaterialBindingAPI(marker.GetPrim()).Bind(material)


def animate_trajectory(waypoints, dt=0.05):
    """Animate the robot along a list of joint configs."""
    # Subsample if too many waypoints (keep animation smooth, not too slow)
    if len(waypoints) > 100:
        step = len(waypoints) // 100
        waypoints = waypoints[::step]
    for wp in waypoints:
        if not simulation_app.is_running():
            return
        set_robot_joints(wp)
        for _ in range(3):
            simulation_app.update()
        time.sleep(dt)  # throttle to keep UI responsive


def try_solve_and_animate(fail):
    """Try to re-solve a solver-limitation failure and animate the result."""
    if motion_gen is None:
        return False

    start_q = torch.tensor(
        [fail["start_config"]], dtype=torch.float32, device="cuda:0"
    )
    start_state = JointState.from_position(
        start_q, joint_names=cfg["robot"]["joint_names"]
    )
    goal_pose = Pose.from_list(
        fail["goal_position"] + fail["goal_orientation_wxyz"]
    )

    plan_config = MotionGenPlanConfig(
        max_attempts=50,
        enable_graph=True,
        enable_finetune_trajopt=True,
        parallel_finetune=True,
    )

    result = motion_gen.plan_single(start_state, goal_pose, plan_config)

    # Let renderer recover after GPU-heavy planning
    torch.cuda.synchronize()
    for _ in range(10):
        simulation_app.update()

    if not result.success.item():
        print("    Could not re-solve — showing static pose only")
        return False

    # Extract trajectory waypoints
    traj = result.get_interpolated_plan()
    waypoints = traj.position.cpu().numpy()  # (T, n_joints)

    n_pts = len(waypoints)
    print(f"    Re-solved! Animating {n_pts} waypoints...")

    # Change marker to green (solved)
    set_goal_marker(fail["goal_position"], mat_grn)

    # Animate forward
    animate_trajectory(waypoints, dt=0.04)

    # Brief pause at goal
    t0 = time.time()
    while simulation_app.is_running() and (time.time() - t0) < 1.0:
        simulation_app.update()

    # Animate back (reverse)
    animate_trajectory(waypoints[::-1], dt=0.04)

    return True


# ---------------------------------------------------------------------------
# Step through failures
# ---------------------------------------------------------------------------
print("\n" + "=" * 60)
print(f"  Visualizing {len(failures)} failures")
print(f"  RED    = INFEASIBLE (no valid path, static view)")
print(f"  YELLOW = SOLVER LIMITATION (re-solving + animating)")
print(f"  GREEN  = re-solved successfully (trajectory shown)")
print(f"  Close the Isaac Sim window to stop early")
print("=" * 60)

for i, fail in enumerate(failures):
    if not simulation_app.is_running():
        break

    is_infeasible = fail["root_cause"].startswith("INFEASIBLE")
    tag = "INFEASIBLE" if is_infeasible else "SOLVER"

    print(f"\n  [{i+1}/{len(failures)}] Problem #{fail['problem_id']}")
    print(f"    Category: {tag}")
    print(f"    Cause:    {fail['root_cause']}")
    print(f"    Goal:     {[f'{v:.3f}' for v in fail['goal_position']]}")

    # Set robot to start config and let renderer settle
    set_robot_joints(fail["start_config"])
    for _ in range(30):
        simulation_app.update()
    time.sleep(0.1)  # give UI time to process events

    if is_infeasible:
        # Static view — red marker, pause
        set_goal_marker(fail["goal_position"], mat_red)
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < args.pause:
            simulation_app.update()
    else:
        # Solver limitation — try to re-solve and animate
        set_goal_marker(fail["goal_position"], mat_yel)
        for _ in range(5):
            simulation_app.update()

        if not try_solve_and_animate(fail):
            # Couldn't re-solve — just show static for a bit
            t0 = time.time()
            while simulation_app.is_running() and (time.time() - t0) < args.pause:
                simulation_app.update()

if simulation_app.is_running():
    print("\n  Done! Close the Isaac Sim window.")
    while simulation_app.is_running():
        simulation_app.update()

simulation_app.close()
