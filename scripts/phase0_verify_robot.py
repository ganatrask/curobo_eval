"""
Phase 0: Robot Verification
============================
Run this FIRST before any benchmarking. It validates:
1. Collision sphere coverage (visual inspection in Isaac Sim)
2. FK/IK round-trip accuracy (numerical check)
3. Workspace reachability scan (generates the workspace map used by all phases)

Usage:
    omni_python scripts/phase0_verify_robot.py
    omni_python scripts/phase0_verify_robot.py --headless   # skip visualization
    omni_python scripts/phase0_verify_robot.py --robot /path/to/my_robot.yml
"""

# Handle Isaac Sim import
try:
    import isaacsim
except ImportError:
    pass

import torch
import numpy as np
import argparse
import os
import sys
import yaml

# Parse args before Isaac Sim launch
parser = argparse.ArgumentParser(description="Phase 0: Robot Verification")
parser.add_argument("--config", type=str, default="config/eval_config.yaml")
parser.add_argument("--robot", type=str, default=None, help="Override robot config file")
parser.add_argument("--headless", action="store_true", help="Skip GUI visualization")
parser.add_argument("--skip_workspace", action="store_true", help="Skip workspace scan")
args = parser.parse_args()

# Load eval config
script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)

# Must be added BEFORE Isaac Sim launches — it modifies sys.path on startup
# and can shadow local packages like 'utils'
sys.path.insert(0, script_dir)

config_path = os.path.join(project_dir, args.config)
with open(config_path, "r") as f:
    cfg = yaml.safe_load(f)

robot_file = args.robot or cfg["robot"]["config_file"]

# ---------------------------------------------------------------------------
# Launch Isaac Sim (must happen before other Isaac/cuRobo imports)
# ---------------------------------------------------------------------------
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": args.headless,
    "width": "1920",
    "height": "1080",
})

# Now safe to import cuRobo
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.util.usd_helper import UsdHelper

# Load local utils directly by file path — bypasses Isaac Sim's sys.path pollution
import importlib.util as _ilu
_pg_spec = _ilu.spec_from_file_location(
    "problem_generator",
    os.path.join(script_dir, "utils", "problem_generator.py"),
)
_pg_mod = _ilu.module_from_spec(_pg_spec)
_pg_spec.loader.exec_module(_pg_mod)
ProblemGenerator = _pg_mod.ProblemGenerator

tensor_args = TensorDeviceType()


def load_robot_config(robot_file: str) -> dict:
    """Load robot config dict from file path or cuRobo's built-in configs."""
    if os.path.isabs(robot_file):
        return load_yaml(robot_file)
    else:
        return load_yaml(join_path(get_robot_configs_path(), robot_file))


# =====================================================================
# TEST 1: Load robot and verify basic FK
# =====================================================================
def test_fk(robot_cfg_dict: dict):
    """Verify forward kinematics works correctly."""
    print("\n" + "="*60)
    print("  TEST 1: Forward Kinematics Verification")
    print("="*60)

    robot_cfg = RobotConfig.from_dict(robot_cfg_dict["robot_cfg"])
    kin_model = CudaRobotModel(robot_cfg.kinematics)

    # Test at retract config
    retract = cfg["robot"].get("retract_config")
    if retract:
        q = torch.tensor([retract], dtype=torch.float32, device="cuda:0")
    else:
        # Use cuRobo's built-in retract
        q = torch.zeros(1, len(cfg["robot"]["joint_names"]),
                        dtype=torch.float32, device="cuda:0")

    state = kin_model.get_state(q)
    ee_pos = state.ee_position.cpu().numpy()[0]
    ee_quat = state.ee_quaternion.cpu().numpy()[0]

    print(f"  Robot:     {robot_file}")
    print(f"  EE link:   {robot_cfg_dict['robot_cfg']['kinematics']['ee_link']}")
    print(f"  DOF:       {len(cfg['robot']['joint_names'])}")
    print(f"  Retract q: {[f'{v:.3f}' for v in q[0].cpu().numpy()]}")
    print(f"  EE pos:    [{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}] m")
    print(f"  EE quat:   [{ee_quat[0]:.4f}, {ee_quat[1]:.4f}, "
          f"{ee_quat[2]:.4f}, {ee_quat[3]:.4f}] (wxyz)")

    # Verify EE is in a sensible position (not at origin, not 10m away)
    ee_dist = np.linalg.norm(ee_pos)
    if ee_dist < 0.05:
        print("  WARNING: EE is very close to robot base — check URDF/config")
    elif ee_dist > 3.0:
        print("  WARNING: EE is very far from base — check units (should be meters)")
    else:
        print(f"  PASS: EE distance from base = {ee_dist:.3f}m (reasonable)")

    return kin_model


# =====================================================================
# TEST 2: Verify collision spheres
# =====================================================================
def test_spheres(robot_cfg_dict: dict, kin_model: CudaRobotModel):
    """Check collision sphere coverage."""
    print("\n" + "="*60)
    print("  TEST 2: Collision Sphere Verification")
    print("="*60)

    retract = cfg["robot"].get("retract_config")
    if retract:
        q = torch.tensor([retract], dtype=torch.float32, device="cuda:0")
    else:
        q = torch.zeros(1, len(cfg["robot"]["joint_names"]),
                        dtype=torch.float32, device="cuda:0")

    # Get spheres at retract config
    # Returns list[list[Sphere]] — each Sphere has .name, .position, .radius
    spheres = kin_model.get_robot_as_spheres(q)
    all_spheres = spheres[0]  # first batch item
    total_spheres = len(all_spheres)

    link_sphere_counts = {}
    for s in all_spheres:
        link_sphere_counts[s.name] = link_sphere_counts.get(s.name, 0) + 1

    print(f"  Total collision spheres: {total_spheres}")
    print(f"  Spheres per link:")
    for link, count in link_sphere_counts.items():
        marker = "OK" if count >= 3 else "LOW"
        print(f"    {link}: {count} [{marker}]")

    if total_spheres < 15:
        print("  WARNING: Very few spheres — collision checking may miss contacts")
    else:
        print(f"  PASS: {total_spheres} spheres (adequate coverage)")

    # Check for spatula coverage
    tool_cfg = cfg.get("tool", {})
    if tool_cfg.get("has_spatula"):
        ee_link = robot_cfg_dict["robot_cfg"]["kinematics"]["ee_link"]
        extra_links = robot_cfg_dict["robot_cfg"]["kinematics"].get("extra_links", {})
        if extra_links:
            print(f"  Spatula extra_links found: {list(extra_links.keys())}")
        else:
            print("  WARNING: has_spatula=true but no extra_links in robot config")
            print("  Make sure spatula is either in the URDF or added via extra_links")

    if not args.headless:
        print("\n  ==> VISUAL CHECK: Isaac Sim window is open.")
        print("  ==> Look at the robot and verify spheres cover all links,")
        print("  ==> especially the spatula/gripper area.")
        print("  ==> Run: omni_python scripts/phase0_verify_robot.py")
        print("  ==> with --visualize_spheres flag in motion_gen_reacher.py")


# =====================================================================
# TEST 3: IK round-trip accuracy
# =====================================================================
def test_ik_roundtrip(robot_cfg_dict: dict, kin_model: CudaRobotModel):
    """
    Test FK→IK→FK round-trip accuracy.
    1. Start at known joint config
    2. Compute FK → get EE pose
    3. Solve IK for that EE pose → get joint config
    4. Compute FK on IK result → get new EE pose
    5. Compare: original EE pose vs round-trip EE pose
    """
    print("\n" + "="*60)
    print("  TEST 3: IK Round-Trip Accuracy")
    print("="*60)

    robot_cfg = RobotConfig.from_dict(robot_cfg_dict["robot_cfg"])

    ik_config = IKSolverConfig.load_from_robot_config(
        robot_cfg,
        None,  # No world obstacles for this test
        rotation_threshold=0.05,
        position_threshold=0.005,
        num_seeds=20,
        self_collision_check=True,
        self_collision_opt=True,
        tensor_args=tensor_args,
        use_cuda_graph=True,
    )
    ik_solver = IKSolver(ik_config)

    # Sample 100 random configs, do FK→IK→FK
    n_test = 100
    q_samples = ik_solver.sample_configs(n_test)

    # FK on original configs
    state_orig = kin_model.get_state(q_samples)
    pos_orig = state_orig.ee_position
    quat_orig = state_orig.ee_quaternion

    # Solve IK for these poses
    goal_poses = Pose(position=pos_orig, quaternion=quat_orig)
    ik_result = ik_solver.solve_batch(goal_poses)

    n_solved = ik_result.success.sum().item()
    print(f"  IK solved: {n_solved}/{n_test} ({100*n_solved/n_test:.1f}%)")

    if n_solved == 0:
        print("  FAIL: IK could not solve any pose — check robot config")
        return

    # FK on IK solutions
    ik_solutions = ik_result.solution[:, 0]  # Take first solution per pose
    success_mask = ik_result.success.squeeze()

    solved_q = ik_solutions[success_mask]
    solved_pos_orig = pos_orig[success_mask]
    solved_quat_orig = quat_orig[success_mask]

    state_rt = kin_model.get_state(solved_q)
    pos_rt = state_rt.ee_position
    quat_rt = state_rt.ee_quaternion

    # Compute errors
    pos_errors = torch.norm(solved_pos_orig - pos_rt, dim=1) * 1000  # mm
    mean_pos_err = pos_errors.mean().item()
    max_pos_err = pos_errors.max().item()

    print(f"  Position error: mean={mean_pos_err:.4f}mm, max={max_pos_err:.4f}mm")

    if max_pos_err > 1.0:
        print("  WARNING: Max position error > 1mm — check IK precision settings")
    else:
        print("  PASS: Sub-millimeter IK accuracy")


# =====================================================================
# TEST 4: Workspace Discovery
# =====================================================================
def test_workspace():
    """Discover and save the reachable workspace."""
    print("\n" + "="*60)
    print("  TEST 4: Workspace Reachability Scan")
    print("="*60)

    ws = cfg["workspace"]
    tool_cfg = cfg.get("tool", {})

    gen = ProblemGenerator(
        robot_file=robot_file,
        world_config={},  # No obstacles for workspace scan
        workspace_bounds={
            "x_range": ws["x_range"],
            "y_range": ws["y_range"],
            "z_range": ws["z_range"],
        },
        tensor_args=tensor_args,
        orientation_wxyz=tool_cfg.get("flat_orientation_wxyz", [0, 1, 0, 0]),
        orientation_mode=ws.get("orientation_mode", "fixed"),
        reachability_resolution=ws.get("reachability_resolution", 0.05),
    )

    reachable_pos, reachable_q = gen.discover_workspace()

    # Save for use by Phase 1-3
    workspace_file = os.path.join(project_dir, "results", "workspace.npz")
    gen.save_workspace(workspace_file)

    # Print workspace stats
    if len(reachable_pos) > 0:
        print(f"\n  Reachable workspace bounds:")
        print(f"    X: [{reachable_pos[:,0].min():.3f}, {reachable_pos[:,0].max():.3f}] m")
        print(f"    Y: [{reachable_pos[:,1].min():.3f}, {reachable_pos[:,1].max():.3f}] m")
        print(f"    Z: [{reachable_pos[:,2].min():.3f}, {reachable_pos[:,2].max():.3f}] m")

        vol = ((reachable_pos[:,0].max() - reachable_pos[:,0].min()) *
               (reachable_pos[:,1].max() - reachable_pos[:,1].min()) *
               (reachable_pos[:,2].max() - reachable_pos[:,2].min()))
        print(f"    Approximate volume: {vol:.3f} m³")

    # Generate and save baseline problems
    problems = gen.generate(n=cfg["planning"]["num_problems"], seed=42)
    problems_file = os.path.join(project_dir, "results", "problems_baseline.json")
    gen.save_problems(problems, problems_file)

    return gen


# =====================================================================
# MAIN
# =====================================================================
def main():
    print("\n" + "#"*60)
    print("  cuRobo Evaluation — Phase 0: Robot Verification")
    print("#"*60)

    robot_cfg_dict = load_robot_config(robot_file)

    # Test 1: FK
    kin_model = test_fk(robot_cfg_dict)

    # Test 2: Spheres
    test_spheres(robot_cfg_dict, kin_model)

    # Test 3: IK round-trip
    test_ik_roundtrip(robot_cfg_dict, kin_model)

    # Test 4: Workspace (skip if requested)
    if not args.skip_workspace:
        test_workspace()
    else:
        print("\n  Skipping workspace scan (--skip_workspace)")

    print("\n" + "#"*60)
    print("  Phase 0 Complete")
    print("  Next step: omni_python scripts/phase1_baseline.py --headless")
    print("#"*60 + "\n")

    simulation_app.close()


if __name__ == "__main__":
    main()
