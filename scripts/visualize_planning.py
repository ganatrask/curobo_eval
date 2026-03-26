"""
Visualize Motion Planning in Isaac Sim
========================================
Shows the robot planning and executing trajectories with obstacles.
For each problem: start config → plan → animate trajectory → next.

Usage:
    # No obstacles (Phase 1 problems), first 20
    .../python.sh scripts/visualize_planning.py --num 20

    # With obstacles
    .../python.sh scripts/visualize_planning.py --layout few_boxes --num 20

    # With orientation constraint
    .../python.sh scripts/visualize_planning.py --layout scattered --constrained --num 10

    # Faster/slower animation
    .../python.sh scripts/visualize_planning.py --layout cluttered --speed 2.0
"""

import os
import sys
import json
import yaml
import time
import numpy as np
import torch
import argparse

parser = argparse.ArgumentParser(description="Visualize motion planning")
parser.add_argument("--layout", type=str, default=None,
                    help="Obstacle layout (e.g., few_boxes, scattered, cluttered)")
parser.add_argument("--constrained", action="store_true",
                    help="Enable orientation constraint")
parser.add_argument("--num", type=int, default=20,
                    help="Number of problems to visualize (default: 20)")
parser.add_argument("--speed", type=float, default=1.0,
                    help="Animation speed multiplier (default: 1.0)")
parser.add_argument("--pause", type=float, default=1.0,
                    help="Pause between problems in seconds")
parser.add_argument("--config", type=str, default="config/eval_config.yaml")
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
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.rollout.cost.pose_cost import PoseCostMetric
from curobo.util_file import load_yaml

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
tensor_args = TensorDeviceType()

with open(os.path.join(project_dir, args.config)) as f:
    cfg = yaml.safe_load(f)

# ---------------------------------------------------------------------------
# Load problems
# ---------------------------------------------------------------------------
if args.layout:
    problems_file = os.path.join(project_dir, "results", f"problems_{args.layout}.json")
else:
    problems_file = os.path.join(project_dir, "results", "problems_baseline.json")

if not os.path.exists(problems_file):
    print(f"ERROR: {problems_file} not found. Run the benchmark phase first.")
    simulation_app.close()
    sys.exit(1)

with open(problems_file) as f:
    problems = json.load(f)["problems"][:args.num]

print(f"\nLoaded {len(problems)} problems")

# ---------------------------------------------------------------------------
# Build world config
# ---------------------------------------------------------------------------
world_cfg = {
    "cuboid": {"_dummy": {"dims": [0.01, 0.01, 0.01],
                           "pose": [100.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]}}
}
layout_cfg = None

if args.layout:
    layouts = cfg.get("obstacle_layouts", {})
    if args.layout not in layouts:
        print(f"Layout '{args.layout}' not found. Available: {list(layouts.keys())}")
        simulation_app.close()
        sys.exit(1)
    layout_cfg = layouts[args.layout]
    world_cfg = {"cuboid": {}, "mesh": {}}
    for name, obs in layout_cfg.get("cuboid", {}).items():
        world_cfg["cuboid"][name] = {"dims": obs["dims"], "pose": obs["pose"]}

# ---------------------------------------------------------------------------
# Build cuRobo MotionGen
# ---------------------------------------------------------------------------
print("Setting up cuRobo MotionGen...")
robot_file = cfg["robot"]["config_file"]
robot_cfg_dict = load_yaml(robot_file)
_yaml_dir = os.path.dirname(os.path.abspath(robot_file))
_kin = robot_cfg_dict.get("robot_cfg", {}).get("kinematics", {})
_kin["external_asset_path"] = _yaml_dir

pcfg = cfg["planning"]
mg_config = MotionGenConfig.load_from_robot_config(
    robot_cfg_dict, world_cfg, tensor_args,
    interpolation_dt=pcfg["interpolation_dt"],
    num_trajopt_seeds=pcfg["num_trajopt_seeds"],
    num_graph_seeds=pcfg["num_graph_seeds"],
    collision_activation_distance=pcfg["collision_activation_distance"],
    use_cuda_graph=False,
)
motion_gen = MotionGen(mg_config)
motion_gen.warmup(parallel_finetune=True)
print("MotionGen ready.")

# Plan config
if args.constrained:
    pose_cost = PoseCostMetric(
        hold_partial_pose=True,
        hold_vec_weight=tensor_args.to_device([1, 1, 1, 0, 0, 0]),
    )
else:
    pose_cost = None

plan_config = MotionGenPlanConfig(
    max_attempts=pcfg["max_attempts"],
    enable_graph=pcfg["enable_graph"],
    enable_finetune_trajopt=True,
    parallel_finetune=True,
    pose_cost_metric=pose_cost,
)

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

stage = omni.usd.get_context().get_stage()

# Camera
_cam = stage.GetPrimAtPath("/OmniverseKit_Persp")
if _cam.IsValid():
    _xf = UsdGeom.Xformable(_cam)
    _xf.ClearXformOpOrder()
    _xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(0.9, 0.7, 0.6),
        Gf.Vec3d(0.3, 0.0, 0.1),
        Gf.Vec3d(0, 0, 1),
    ).GetInverse())

# Draw obstacles
if layout_cfg:
    obs_scope = "/World/obstacles"
    stage.DefinePrim(obs_scope, "Scope")

    obs_colors = {
        "table": Gf.Vec3f(0.4, 0.3, 0.2),
        "shelf": Gf.Vec3f(0.6, 0.5, 0.3),
        "wall": Gf.Vec3f(0.3, 0.5, 0.7),
    }
    default_color = Gf.Vec3f(0.7, 0.35, 0.35)

    from pxr import UsdPhysics

    for obs_name, obs in layout_cfg.get("cuboid", {}).items():
        dims, pose = obs["dims"], obs["pose"]
        cube_path = f"{obs_scope}/{obs_name}"
        cube = UsdGeom.Cube.Define(stage, cube_path)
        cube.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(pose[0], pose[1], pose[2]))
        xf.AddScaleOp().Set(Gf.Vec3f(dims[0], dims[1], dims[2]))

        # Add physics collision — robot will physically collide with these
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        # Make it a static rigid body (doesn't move when hit)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        cube.GetPrim().GetAttribute("physics:kinematicEnabled").Set(True)

        color = default_color
        for key, c in obs_colors.items():
            if key in obs_name.lower():
                color = c
                break

        mat_path = f"{obs_scope}/mat_{obs_name}"
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.5)
        sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)

# Goal marker
marker = UsdGeom.Sphere.Define(stage, "/World/goal_marker")
marker.GetRadiusAttr().Set(0.02)

# Marker materials
def make_mat(path, color, opacity=0.8):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

mat_green = make_mat("/World/mat_green", Gf.Vec3f(0.1, 0.9, 0.2))
mat_red = make_mat("/World/mat_red", Gf.Vec3f(1.0, 0.1, 0.1))
mat_blue = make_mat("/World/mat_blue", Gf.Vec3f(0.2, 0.4, 1.0))

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
omni.timeline.get_timeline_interface().play()
simulation_app.update()
simulation_app.update()

try:
    dof_names = robot.get_dof_names()
except Exception:
    dof_names = None

ARM_JOINTS = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def build_full_positions(joint_values):
    """Map 6-DOF arm joints to full DOF array."""
    positions = np.zeros(len(dof_names))
    for idx, name in enumerate(dof_names):
        if name in ARM_JOINTS:
            j = ARM_JOINTS.index(name)
            if j < len(joint_values):
                positions[idx] = joint_values[j]
        elif name in ("left_carriage_joint", "right_carriage_joint"):
            positions[idx] = 0.022
    return positions


def set_joints(joint_values, use_targets=False):
    if dof_names is None:
        return
    positions = build_full_positions(joint_values)
    if use_targets:
        # Use position targets — works during physics simulation
        robot.set_joint_position_targets(positions.reshape(1, -1))
    else:
        # Direct set — use when snapping to a pose (e.g., start config)
        robot.set_joint_positions(positions.reshape(1, -1))
        robot.set_joint_position_targets(positions.reshape(1, -1))


def set_marker(pos, mat):
    xf = UsdGeom.Xformable(marker.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    UsdShade.MaterialBindingAPI(marker.GetPrim()).Bind(mat)


def animate(waypoints, speed=1.0):
    """Animate by setting joint positions while timeline is playing.

    Uses the same pattern as verify_spatula_orientation.py (which works):
    set_joint_positions → multiple update() calls to let physics apply it.
    """
    # Subsample for reasonable animation length
    if len(waypoints) > 100:
        step = max(1, len(waypoints) // 100)
        waypoints = waypoints[::step]

    for wp in waypoints:
        if not simulation_app.is_running():
            break
        positions = build_full_positions(wp)
        robot.set_joint_positions(positions.reshape(1, -1))
        # Multiple updates so physics applies the position before next waypoint
        for _ in range(5):
            simulation_app.update()
        time.sleep(0.02 / speed)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
label = args.layout or "baseline"
constraint_label = "constrained" if args.constrained else "unconstrained"
success_count = 0
fail_count = 0

print(f"\n{'='*60}")
print(f"  Visualizing: {label} | {constraint_label}")
print(f"  Problems: {len(problems)} | Speed: {args.speed}x")
print(f"  Close Isaac Sim window to stop")
print(f"{'='*60}")

for i, problem in enumerate(problems):
    if not simulation_app.is_running():
        break

    # Snap to start config — same pattern as verify_spatula_orientation.py
    positions = build_full_positions(problem["start_config"])
    robot.set_joint_positions(positions.reshape(1, -1))
    set_marker(problem["goal_position"], mat_blue)

    # Let robot settle at start position
    for _ in range(30):
        simulation_app.update()
    time.sleep(0.3)

    # Plan
    start_q = torch.tensor([problem["start_config"]], dtype=torch.float32, device="cuda:0")
    start_state = JointState.from_position(start_q, joint_names=cfg["robot"]["joint_names"])
    goal_pose = Pose.from_list(problem["goal_position"] + problem["goal_orientation_wxyz"])

    result = motion_gen.plan_single(start_state, goal_pose, plan_config)
    torch.cuda.synchronize()

    # Let renderer recover
    for _ in range(10):
        simulation_app.update()

    if result.success.item():
        success_count += 1
        traj = result.get_interpolated_plan()
        waypoints = traj.position.cpu().numpy()
        plan_ms = result.solve_time * 1000 if hasattr(result, 'solve_time') else 0

        # Change marker to green
        set_marker(problem["goal_position"], mat_green)

        gp = problem['goal_position']
        print(f"  [{i+1}/{len(problems)}] #{problem['id']} SUCCESS "
              f"({len(waypoints)} pts) goal=[{gp[0]:.2f}, {gp[1]:.2f}, {gp[2]:.2f}]")

        # Animate forward
        animate(waypoints, args.speed)

        # Brief pause at goal
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < args.pause:
            simulation_app.update()

        # Animate back to start
        animate(waypoints[::-1], args.speed * 2)

    else:
        fail_count += 1
        set_marker(problem["goal_position"], mat_red)
        status = str(result.status) if hasattr(result, 'status') else "FAIL"
        print(f"  [{i+1}/{len(problems)}] #{problem['id']} FAILED — {status}")

        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < args.pause * 2:
            simulation_app.update()

print(f"\n{'='*60}")
print(f"  Done: {success_count} success, {fail_count} failed")
print(f"{'='*60}")

if simulation_app.is_running():
    print("  Close the Isaac Sim window.")
    while simulation_app.is_running():
        simulation_app.update()

simulation_app.close()
