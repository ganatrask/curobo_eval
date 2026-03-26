"""
Visualize Obstacle Layouts in Isaac Sim
========================================
Shows the robot + obstacles for each layout defined in eval_config.yaml.

Usage:
    /home/shyam/workspace/isaacsim-5.1.0/python.sh scripts/visualize_obstacles.py
    /home/shyam/workspace/isaacsim-5.1.0/python.sh scripts/visualize_obstacles.py --layout easy
"""

import os
import sys
import yaml
import numpy as np
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--layout", type=str, default=None,
                    help="Show only this layout (e.g., easy, medium, hard)")
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

script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)

with open(os.path.join(project_dir, args.config)) as f:
    cfg = yaml.safe_load(f)

layouts = cfg.get("obstacle_layouts", {})
if args.layout:
    if args.layout not in layouts:
        print(f"Layout '{args.layout}' not found. Available: {list(layouts.keys())}")
        simulation_app.close()
        sys.exit(1)
    layouts = {args.layout: layouts[args.layout]}

USD_PATH = cfg["robot"]["usd_path"]
RETRACT = cfg["robot"]["retract_config"]
ROBOT_PATH = "/World/robot"


def build_scene(layout_name, layout_cfg):
    """Build a scene with robot + obstacles for one layout."""
    stage_utils.create_new_stage(template="sunlight")
    stage_utils.add_reference_to_stage(usd_path=USD_PATH, path=ROBOT_PATH)
    stage_utils.add_reference_to_stage(
        usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
        path="/World/ground",
    )

    stage = omni.usd.get_context().get_stage()

    # Camera
    _cam = stage.GetPrimAtPath("/OmniverseKit_Persp")
    if _cam.IsValid():
        _xf = UsdGeom.Xformable(_cam)
        _xf.ClearXformOpOrder()
        _xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(1.0, 0.8, 0.7),
            Gf.Vec3d(0.3, 0.0, 0.1),
            Gf.Vec3d(0, 0, 1),
        ).GetInverse())

    # Obstacle materials
    colors = {
        "table": Gf.Vec3f(0.4, 0.3, 0.2),    # brown
        "shelf": Gf.Vec3f(0.6, 0.5, 0.3),     # light brown
        "wall":  Gf.Vec3f(0.3, 0.5, 0.7),     # blue
        "box":   Gf.Vec3f(0.7, 0.3, 0.3),     # red
    }

    # Draw cuboid obstacles
    obs_scope = "/World/obstacles"
    stage.DefinePrim(obs_scope, "Scope")

    cuboids = layout_cfg.get("cuboid", {})
    for obs_name, obs in cuboids.items():
        dims = obs["dims"]   # [length, width, height]
        pose = obs["pose"]   # [x, y, z, qw, qx, qy, qz]

        cube_path = f"{obs_scope}/{obs_name}"
        cube = UsdGeom.Cube.Define(stage, cube_path)
        cube.GetSizeAttr().Set(1.0)  # unit cube, scale to dims

        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(pose[0], pose[1], pose[2]))
        xf.AddScaleOp().Set(Gf.Vec3f(dims[0], dims[1], dims[2]))

        if len(pose) == 7 and not (pose[3] == 1 and pose[4] == 0 and pose[5] == 0 and pose[6] == 0):
            # Non-identity rotation — apply quaternion
            qw, qx, qy, qz = pose[3], pose[4], pose[5], pose[6]
            xf.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))

        # Color based on obstacle type
        color = Gf.Vec3f(0.5, 0.5, 0.5)  # default gray
        for key, c in colors.items():
            if key in obs_name.lower():
                color = c
                break

        mat_path = f"{obs_scope}/mat_{obs_name}"
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.6)
        sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)

    # Draw workspace bounds as wireframe box
    ws = cfg["workspace"]
    x_min, x_max = ws["x_range"]
    y_min, y_max = ws["y_range"]
    z_min, z_max = ws["z_range"]
    cx = (x_min + x_max) / 2
    cy = (y_min + y_max) / 2
    cz = (z_min + z_max) / 2
    sx = x_max - x_min
    sy = y_max - y_min
    sz = z_max - z_min

    ws_path = "/World/workspace_bounds"
    ws_cube = UsdGeom.Cube.Define(stage, ws_path)
    ws_cube.GetSizeAttr().Set(1.0)
    ws_cube.GetPurposeAttr().Set("guide")  # wireframe
    ws_xf = UsdGeom.Xformable(ws_cube.GetPrim())
    ws_xf.ClearXformOpOrder()
    ws_xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    ws_xf.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))

    # Green wireframe material for workspace
    ws_mat_path = "/World/mat_workspace"
    ws_mat = UsdShade.Material.Define(stage, ws_mat_path)
    ws_sh = UsdShade.Shader.Define(stage, ws_mat_path + "/Shader")
    ws_sh.CreateIdAttr("UsdPreviewSurface")
    ws_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.2, 1.0, 0.3))
    ws_sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.15)
    ws_sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    ws_mat.CreateSurfaceOutput().ConnectToSource(ws_sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(ws_cube.GetPrim()).Bind(ws_mat)

    return Articulation(ROBOT_PATH)


# ---------------------------------------------------------------------------
# Show each layout
# ---------------------------------------------------------------------------
for layout_name, layout_cfg in layouts.items():
    print(f"\n{'='*55}")
    print(f"  Layout: {layout_name}")
    print(f"  Description: {layout_cfg.get('description', '')}")
    cuboids = layout_cfg.get("cuboid", {})
    print(f"  Obstacles: {len(cuboids)}")
    for name, obs in cuboids.items():
        print(f"    {name}: dims={obs['dims']} pos=[{obs['pose'][0]:.2f}, {obs['pose'][1]:.2f}, {obs['pose'][2]:.2f}]")
    print(f"{'='*55}")

    robot = build_scene(layout_name, layout_cfg)

    omni.timeline.get_timeline_interface().play()
    simulation_app.update()
    simulation_app.update()

    # Set robot to retract
    try:
        dof_names = robot.get_dof_names()
        positions = np.zeros(len(dof_names))
        arm_joints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        for idx, name in enumerate(dof_names):
            if name in arm_joints:
                j_idx = arm_joints.index(name)
                positions[idx] = RETRACT[j_idx]
            elif name in ("left_carriage_joint", "right_carriage_joint"):
                positions[idx] = 0.022
        robot.set_joint_positions(positions.reshape(1, -1))
    except Exception:
        pass

    for _ in range(30):
        simulation_app.update()

    if len(layouts) > 1:
        print(f"\n  Showing '{layout_name}' for 10 seconds...")
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < 10:
            simulation_app.update()
    else:
        print(f"\n  Close the Isaac Sim window when done.")
        while simulation_app.is_running():
            simulation_app.update()

    if not simulation_app.is_running():
        break

if simulation_app.is_running():
    print("\n  All layouts shown. Close the window.")
    while simulation_app.is_running():
        simulation_app.update()

simulation_app.close()
