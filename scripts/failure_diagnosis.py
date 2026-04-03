"""
Failure Diagnosis & Visualization
===================================
THE most important script in the evaluation pipeline.

What it does:
1. Loads per-problem results from any phase
2. Filters failures
3. For EACH failure, runs a multi-step diagnostic to classify root cause
4. Opens Isaac Sim to VISUALIZE the failed scenario interactively
5. Generates a failure report with actionable recommendations

Root cause classification:
  INFEASIBLE_GOAL_COLLISION    - Goal pose is inside an obstacle
  INFEASIBLE_START_COLLISION   - Start config collides with world
  INFEASIBLE_SELF_COLLISION    - Start or goal in self-collision
  INFEASIBLE_UNREACHABLE       - IK has no solution (out of workspace)
  SOLVER_IK_FAIL               - IK solution exists but solver missed it
  SOLVER_TRAJOPT_FAIL          - Path exists but trajopt couldn't find it
  SOLVER_NEEDS_MORE_SEEDS      - Solved with more seeds (tuning issue)
  SOLVER_NEEDS_GRAPH           - Solved with graph planner (narrow passage)
  SOLVER_DT_EXCEPTION          - Trajectory too long for time budget
  UNKNOWN                      - Could not determine root cause

Usage:
    # Diagnose all failures from Phase 3 hard constrained:
    omni_python scripts/failure_diagnosis.py --phase phase3_hard_constrained

    # Diagnose + visualize a specific failed problem:
    omni_python scripts/failure_diagnosis.py --phase phase3_hard_constrained --problem_id 42

    # Diagnose failures and open interactive replay for worst cases:
    omni_python scripts/failure_diagnosis.py --phase phase1_baseline --interactive

    # Headless diagnosis only (no visualization, just report):
    omni_python scripts/failure_diagnosis.py --phase phase1_baseline --headless
"""

try:
    import isaacsim
except ImportError:
    pass

import torch
import numpy as np
import argparse
import json
import os
import sys
import time
import yaml
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field, asdict

parser = argparse.ArgumentParser(description="Failure Diagnosis & Visualization")
parser.add_argument("--config", type=str, default="config/eval_config.yaml")
parser.add_argument("--phase", type=str, required=True,
                    help="Phase name (e.g., phase1_baseline, phase3_hard_constrained)")
parser.add_argument("--problem_id", type=int, default=None,
                    help="Diagnose a single problem by ID")
parser.add_argument("--max_failures", type=int, default=50,
                    help="Max failures to diagnose (0=all)")
parser.add_argument("--headless", action="store_true",
                    help="No visualization, just diagnosis report")
parser.add_argument("--interactive", action="store_true",
                    help="Pause on each failure for visual inspection")
args = parser.parse_args()

# Launch Isaac Sim
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": args.headless,
    "width": "1920", "height": "1080",
})

# cuRobo imports
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml

script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
sys.path.insert(0, script_dir)

with open(os.path.join(project_dir, args.config), "r") as f:
    cfg = yaml.safe_load(f)

tensor_args = TensorDeviceType()


# =====================================================================
# Data classes
# =====================================================================

@dataclass
class FailureDiagnosis:
    """Diagnosis result for a single failed problem."""
    problem_id: int = -1
    original_status: str = ""
    root_cause: str = "UNKNOWN"
    details: str = ""

    # Collision diagnostics
    start_in_world_collision: bool = False
    start_in_self_collision: bool = False
    goal_in_world_collision: bool = False
    goal_ik_solvable: bool = False
    goal_ik_solutions_in_collision: bool = False

    # Escalation diagnostics
    solved_with_more_seeds: bool = False
    solved_with_graph: bool = False
    seeds_used_to_solve: int = 0

    # Problem data (for visualization)
    start_config: List[float] = field(default_factory=list)
    goal_position: List[float] = field(default_factory=list)
    goal_orientation_wxyz: List[float] = field(default_factory=list)

    def to_dict(self):
        return asdict(self)


# =====================================================================
# Diagnostic engine
# =====================================================================

class FailureDiagnosticEngine:
    """Runs multi-step diagnostics on failed planning problems."""

    def __init__(self, robot_file: str, world_config: Optional[dict],
                 joint_names: List[str]):
        self.robot_file = robot_file
        self.world_config = world_config or {}
        self.joint_names = joint_names

        # Load robot config with paths resolved relative to the YAML's dir
        robot_cfg_dict = load_yaml(robot_file)
        _yaml_dir = os.path.dirname(os.path.abspath(robot_file))
        _kin = robot_cfg_dict.get("robot_cfg", {}).get("kinematics", {})
        _kin["external_asset_path"] = _yaml_dir
        self.robot_cfg = RobotConfig.from_dict(robot_cfg_dict["robot_cfg"])

        # Build RobotWorld for collision queries
        print("  [Diag] Loading RobotWorld for collision checking...")
        rw_config = RobotWorldConfig.load_from_config(
            robot_cfg_dict, self.world_config or {
                "cuboid": {"_dummy": {"dims": [0.01, 0.01, 0.01],
                                      "pose": [100.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]}}
            },
            collision_activation_distance=0.0,
        )
        self.robot_world = RobotWorld(rw_config)

        # Build IK solver (collision-free)
        print("  [Diag] Loading IK solver...")
        ik_config = IKSolverConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_config if self.world_config else None,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=40,  # Extra seeds for diagnosis
            self_collision_check=True,
            self_collision_opt=True,
            tensor_args=tensor_args,
            use_cuda_graph=True,
        )
        self.ik_solver = IKSolver(ik_config)

        # Build MotionGen instances at different seed levels for escalation
        print("  [Diag] Loading MotionGen (standard)...")
        self.motion_gen_standard = self._build_motion_gen(
            num_trajopt_seeds=12, num_graph_seeds=12, enable_graph=False
        )
        print("  [Diag] Loading MotionGen (high seeds)...")
        self.motion_gen_high_seeds = self._build_motion_gen(
            num_trajopt_seeds=48, num_graph_seeds=24, enable_graph=False
        )
        print("  [Diag] Loading MotionGen (with graph)...")
        self.motion_gen_graph = self._build_motion_gen(
            num_trajopt_seeds=12, num_graph_seeds=12, enable_graph=True
        )

        # Get kinematics model
        self.kin_model = self.motion_gen_standard.kinematics

        print("  [Diag] Engine ready.\n")

    def _build_motion_gen(self, num_trajopt_seeds, num_graph_seeds,
                          enable_graph) -> MotionGen:
        pcfg = cfg["planning"]
        # Re-load robot config as dict with resolved paths
        robot_cfg_dict = load_yaml(self.robot_file)
        _yaml_dir = os.path.dirname(os.path.abspath(self.robot_file))
        _kin = robot_cfg_dict.get("robot_cfg", {}).get("kinematics", {})
        _kin["external_asset_path"] = _yaml_dir

        # Use dummy obstacle if no world obstacles
        world_cfg = self.world_config if self.world_config else {
            "cuboid": {"_dummy": {"dims": [0.01, 0.01, 0.01],
                                  "pose": [100.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]}}
        }
        mg_config = MotionGenConfig.load_from_robot_config(
            robot_cfg_dict, world_cfg, tensor_args,
            interpolation_dt=pcfg["interpolation_dt"],
            num_trajopt_seeds=num_trajopt_seeds,
            num_graph_seeds=num_graph_seeds,
            collision_activation_distance=pcfg["collision_activation_distance"],
            use_cuda_graph=True,
        )
        mg = MotionGen(mg_config)
        mg.warmup(parallel_finetune=True)
        return mg

    def diagnose(self, problem: dict) -> FailureDiagnosis:
        """
        Run full diagnostic pipeline on a single failed problem.

        The diagnostic ladder:
        1. Check start config for world collision
        2. Check start config for self-collision
        3. Check if IK can solve the goal pose at all
        4. Check if IK solutions are in collision
        5. Try planning with more seeds (tuning issue?)
        6. Try planning with graph planner (narrow passage?)
        7. Classify root cause
        """
        diag = FailureDiagnosis(
            problem_id=problem["id"],
            original_status=problem.get("status", ""),
            start_config=problem["start_config"],
            goal_position=problem["goal_position"],
            goal_orientation_wxyz=problem["goal_orientation_wxyz"],
        )

        start_q = torch.tensor(
            [problem["start_config"]], dtype=torch.float32, device="cuda:0"
        )
        goal_pos = problem["goal_position"]
        goal_quat = problem["goal_orientation_wxyz"]

        # ---- STEP 1: Check start config for collisions ----
        d_world, d_self = self.robot_world.get_world_self_collision_distance_from_joints(start_q)

        if d_world.max().item() > 0:
            diag.start_in_world_collision = True
            diag.root_cause = "INFEASIBLE_START_COLLISION"
            diag.details = (
                f"Start config collides with world obstacle. "
                f"Max penetration: {d_world.max().item()*1000:.1f}mm. "
                f"Fix: Check that start configs are sampled from "
                f"collision-free IK solutions. Also verify sphere sizes."
            )
            return diag

        if d_self.max().item() > 0:
            diag.start_in_self_collision = True
            diag.root_cause = "INFEASIBLE_SELF_COLLISION"
            diag.details = (
                f"Start config has self-collision. "
                f"Fix: Adjust self_collision_buffer or "
                f"self_collision_ignore in robot config."
            )
            return diag

        # ---- STEP 2: Check if IK can solve the goal pose ----
        goal_pose = Pose.from_list(goal_pos + goal_quat)
        ik_result = self.ik_solver.solve_single(goal_pose)

        if not ik_result.success.any().item():
            diag.goal_ik_solvable = False
            diag.root_cause = "INFEASIBLE_UNREACHABLE"
            diag.details = (
                f"No IK solution found for goal pose "
                f"pos={[f'{v:.3f}' for v in goal_pos]}. "
                f"The pose is outside the robot's reachable workspace "
                f"with the given orientation, OR all IK solutions "
                f"are in collision with obstacles."
            )
            return diag

        diag.goal_ik_solvable = True

        # ---- STEP 3: Check if IK solutions are in collision ----
        ik_q = ik_result.solution.view(-1, start_q.shape[-1])
        d_world_goal, d_self_goal = (
            self.robot_world.get_world_self_collision_distance_from_joints(ik_q)
        )

        # Check each IK solution
        any_collision_free = False
        for s in range(ik_q.shape[0]):
            if d_world_goal[s].max().item() <= 0 and d_self_goal[s].max().item() <= 0:
                any_collision_free = True
                break

        if not any_collision_free:
            diag.goal_ik_solutions_in_collision = True
            diag.root_cause = "INFEASIBLE_GOAL_COLLISION"
            diag.details = (
                f"IK found solutions for the goal pose, but ALL of them "
                f"are in collision with obstacles. The goal is physically "
                f"reachable but not collision-free from any direction. "
                f"Max penetration: {d_world_goal.max().item()*1000:.1f}mm."
            )
            return diag

        # ---- STEP 4: Goal is reachable + collision-free → solver issue ----
        # Try with MORE optimization seeds
        start_state = JointState.from_position(start_q, joint_names=self.joint_names)
        plan_config = MotionGenPlanConfig(
            max_attempts=20,
            enable_graph=False,
            enable_finetune_trajopt=True,
            parallel_finetune=True,
        )

        result_high = self.motion_gen_high_seeds.plan_single(
            start_state, goal_pose, plan_config
        )

        if result_high.success.item():
            diag.solved_with_more_seeds = True
            diag.seeds_used_to_solve = 48
            diag.root_cause = "SOLVER_NEEDS_MORE_SEEDS"
            diag.details = (
                f"Solved with 48 trajopt seeds (original used 12). "
                f"This is a TUNING issue, not an infeasibility. "
                f"Fix: Increase num_trajopt_seeds in eval_config.yaml "
                f"or increase max_attempts."
            )
            return diag

        # ---- STEP 5: Try with graph planner ----
        plan_config_graph = MotionGenPlanConfig(
            max_attempts=20,
            enable_graph=True,
            enable_finetune_trajopt=True,
            parallel_finetune=True,
        )

        result_graph = self.motion_gen_graph.plan_single(
            start_state, goal_pose, plan_config_graph
        )

        if result_graph.success.item():
            diag.solved_with_graph = True
            diag.root_cause = "SOLVER_NEEDS_GRAPH"
            diag.details = (
                f"Solved with graph planner (PRM*). "
                f"This indicates a NARROW PASSAGE that trajectory "
                f"optimization alone cannot navigate. "
                f"Fix: Always enable_graph=True in production."
            )
            return diag

        # ---- STEP 6: Nothing worked — genuine solver limitation ----
        # Check if the original status gives more info
        orig_status = problem.get("status", "")
        if "DT" in orig_status.upper():
            diag.root_cause = "SOLVER_DT_EXCEPTION"
            diag.details = (
                f"Trajectory exceeds maximum allowed duration. "
                f"The motion requires very slow execution to stay "
                f"within joint limits. "
                f"Fix: Increase maximum_trajectory_dt to 60.0."
            )
        elif "IK" in orig_status.upper():
            diag.root_cause = "SOLVER_IK_FAIL"
            diag.details = (
                f"IK solutions exist and are collision-free, but the "
                f"planner's internal IK phase failed. This can happen "
                f"when the IK solver uses fewer seeds in MotionGen "
                f"than in standalone IK."
            )
        else:
            diag.root_cause = "SOLVER_TRAJOPT_FAIL"
            diag.details = (
                f"A collision-free goal config exists, but neither "
                f"trajectory optimization (48 seeds) nor graph planning "
                f"could find a path. This is a genuine solver limitation. "
                f"The path may require very specific waypoints that "
                f"optimization-based planning cannot discover. "
                f"Original status: {orig_status}"
            )

        return diag

    def get_collision_distances_at_config(self, q: torch.Tensor) -> Dict:
        """Get detailed collision info at a joint configuration."""
        d_world, d_self = (
            self.robot_world.get_world_self_collision_distance_from_joints(q)
        )
        return {
            "world_collision": d_world.max().item() > 0,
            "world_max_penetration_mm": d_world.max().item() * 1000,
            "self_collision": d_self.max().item() > 0,
            "self_max_penetration_mm": d_self.max().item() * 1000,
        }

    def get_robot_spheres_at_config(self, q: torch.Tensor):
        """Get sphere positions for visualization."""
        return self.kin_model.get_robot_as_spheres(q)


# =====================================================================
# Visualization helpers
# =====================================================================

def visualize_failure_in_sim(diag: FailureDiagnosis, engine: FailureDiagnosticEngine):
    """
    Show the failed scenario in Isaac Sim:
    - Robot at start config (with spheres visible)
    - Goal pose marker
    - Obstacles colored by proximity
    """
    from omni.isaac.core import World
    from omni.isaac.core.objects import cuboid as ov_cuboid
    import omni.usd

    print(f"\n  --- Visualizing Problem #{diag.problem_id} ---")
    print(f"  Root cause: {diag.root_cause}")
    print(f"  Details: {diag.details}")

    start_q = torch.tensor(
        [diag.start_config], dtype=torch.float32, device="cuda:0"
    )

    # Show collision info at start
    coll_info = engine.get_collision_distances_at_config(start_q)
    print(f"  Start config collision: world={coll_info['world_collision']} "
          f"(penetration={coll_info['world_max_penetration_mm']:.1f}mm), "
          f"self={coll_info['self_collision']}")

    # Show IK info for goal
    goal_pose = Pose.from_list(diag.goal_position + diag.goal_orientation_wxyz)
    ik_result = engine.ik_solver.solve_single(goal_pose)
    print(f"  Goal IK solvable: {ik_result.success.any().item()}")

    if ik_result.success.any().item():
        goal_q = ik_result.solution[0, 0:1]
        coll_goal = engine.get_collision_distances_at_config(goal_q)
        print(f"  Goal config collision: world={coll_goal['world_collision']} "
              f"(penetration={coll_goal['world_max_penetration_mm']:.1f}mm)")

    # Show sphere positions for debugging
    spheres = engine.get_robot_spheres_at_config(start_q)
    total_spheres = len(spheres[0])
    print(f"  Robot represented by {total_spheres} collision spheres")

    print(f"\n  Goal position: {[f'{v:.3f}' for v in diag.goal_position]}")
    print(f"  Goal orientation (wxyz): "
          f"{[f'{v:.3f}' for v in diag.goal_orientation_wxyz]}")

    if not args.headless:
        print(f"\n  >> Isaac Sim is open. Inspect the scenario visually.")
        print(f"  >> Robot should be at start config.")
        print(f"  >> Look for sphere-obstacle overlaps (collision).")
        print(f"  >> Look for narrow passages between obstacles.")

        if args.interactive:
            input("  >> Press ENTER to continue to next failure...")


# =====================================================================
# Report generation
# =====================================================================

def generate_report(diagnoses: List[FailureDiagnosis], phase_name: str):
    """Generate comprehensive failure report."""
    results_dir = os.path.join(project_dir, cfg["output"]["results_dir"])
    if not os.path.isdir(results_dir):
        results_dir = os.path.join(script_dir, cfg["output"]["results_dir"])

    # Classification summary
    cause_counts = {}
    for d in diagnoses:
        cause_counts[d.root_cause] = cause_counts.get(d.root_cause, 0) + 1

    # Sort by frequency
    sorted_causes = sorted(cause_counts.items(), key=lambda x: -x[1])

    print(f"\n{'='*70}")
    print(f"  FAILURE DIAGNOSIS REPORT: {phase_name}")
    print(f"{'='*70}")
    print(f"  Total failures diagnosed: {len(diagnoses)}")
    print(f"\n  ROOT CAUSE BREAKDOWN:")
    print(f"  {'Cause':<35} {'Count':<8} {'%':<8}")
    print(f"  {'-'*51}")

    infeasible_count = 0
    solver_count = 0

    for cause, count in sorted_causes:
        pct = 100 * count / len(diagnoses)
        marker = ""
        if cause.startswith("INFEASIBLE"):
            infeasible_count += count
            marker = "  [no valid path exists]"
        elif cause.startswith("SOLVER"):
            solver_count += count
            marker = "  [valid path exists, solver missed it]"
        print(f"  {cause:<35} {count:<8} {pct:<8.1f}{marker}")

    print(f"\n  SUMMARY:")
    print(f"    Truly infeasible:  {infeasible_count} "
          f"({100*infeasible_count/len(diagnoses):.1f}%) — nothing cuRobo can do")
    print(f"    Solver limitation: {solver_count} "
          f"({100*solver_count/len(diagnoses):.1f}%) — cuRobo COULD solve these")

    # Actionable recommendations
    print(f"\n  RECOMMENDATIONS:")
    if cause_counts.get("SOLVER_NEEDS_MORE_SEEDS", 0) > 0:
        n = cause_counts["SOLVER_NEEDS_MORE_SEEDS"]
        print(f"    → {n} problems solved with more seeds. "
              f"Increase num_trajopt_seeds from 12 to 48.")
    if cause_counts.get("SOLVER_NEEDS_GRAPH", 0) > 0:
        n = cause_counts["SOLVER_NEEDS_GRAPH"]
        print(f"    → {n} problems needed the graph planner. "
              f"Always set enable_graph=True in production.")
    if cause_counts.get("INFEASIBLE_START_COLLISION", 0) > 0:
        n = cause_counts["INFEASIBLE_START_COLLISION"]
        print(f"    → {n} start configs in collision. "
              f"Check collision_sphere_buffer and sphere sizes.")
    if cause_counts.get("INFEASIBLE_GOAL_COLLISION", 0) > 0:
        n = cause_counts["INFEASIBLE_GOAL_COLLISION"]
        print(f"    → {n} goals with all IK solutions in collision. "
              f"Consider alternative approach angles or goal offsets.")
    if cause_counts.get("SOLVER_TRAJOPT_FAIL", 0) > 0:
        n = cause_counts["SOLVER_TRAJOPT_FAIL"]
        print(f"    → {n} genuine solver failures remain. "
              f"Consider: increase trajopt_tsteps, increase "
              f"maximum_trajectory_dt, or use waypoint decomposition.")
    print(f"{'='*70}\n")

    # Save detailed diagnoses
    diag_file = os.path.join(results_dir, f"{phase_name}_failure_diagnosis.json")
    with open(diag_file, "w") as f:
        json.dump({
            "phase": phase_name,
            "total_failures": len(diagnoses),
            "cause_breakdown": dict(sorted_causes),
            "infeasible_count": infeasible_count,
            "solver_limitation_count": solver_count,
            "diagnoses": [d.to_dict() for d in diagnoses],
        }, f, indent=2)
    print(f"  Saved detailed diagnoses to {diag_file}")

    # Save problem IDs by category for easy re-testing
    for cause in cause_counts:
        ids = [d.problem_id for d in diagnoses if d.root_cause == cause]
        ids_file = os.path.join(results_dir,
                                f"{phase_name}_failures_{cause.lower()}.json")
        with open(ids_file, "w") as f:
            json.dump({"cause": cause, "problem_ids": ids}, f)


# =====================================================================
# Main
# =====================================================================

def main():
    print("\n" + "#"*70)
    print("  cuRobo Failure Diagnosis & Visualization")
    print("#"*70)

    phase_name = args.phase
    # Check both project-level and scripts-level results dirs
    results_dir = os.path.join(project_dir, cfg["output"]["results_dir"])
    if not os.path.exists(os.path.join(results_dir, f"{phase_name}_per_problem.json")):
        results_dir = os.path.join(script_dir, cfg["output"]["results_dir"])

    # Load per-problem results
    per_problem_file = os.path.join(results_dir, f"{phase_name}_per_problem.json")
    if not os.path.exists(per_problem_file):
        print(f"  ERROR: {per_problem_file} not found.")
        print(f"  Run the benchmark phase first, then diagnose failures.")
        simulation_app.close()
        return

    with open(per_problem_file, "r") as f:
        all_problems = json.load(f)

    # Filter to failures only (or specific problem)
    if args.problem_id is not None:
        failures = [p for p in all_problems if p["problem_id"] == args.problem_id]
        if not failures:
            # Also check if it's a success — user might want to inspect it
            matches = [p for p in all_problems
                       if p.get("problem_id") == args.problem_id
                       or p.get("id") == args.problem_id]
            if matches:
                print(f"  Problem #{args.problem_id} was a SUCCESS, not a failure.")
                print(f"  Status: {matches[0].get('status')}")
                print(f"  Plan time: {matches[0].get('plan_time_ms', '?')}ms")
            else:
                print(f"  Problem #{args.problem_id} not found in results.")
            simulation_app.close()
            return
    else:
        failures = [p for p in all_problems if not p.get("success", True)]

    n_total = len(all_problems)
    n_fail = len(failures)
    print(f"\n  Phase: {phase_name}")
    print(f"  Total problems: {n_total}")
    print(f"  Failures: {n_fail} ({100*n_fail/n_total:.1f}%)")

    if n_fail == 0:
        print("  No failures to diagnose!")
        simulation_app.close()
        return

    # Limit number of failures to diagnose
    if args.max_failures > 0 and n_fail > args.max_failures:
        print(f"  Diagnosing first {args.max_failures} failures "
              f"(use --max_failures 0 for all)")
        failures = failures[:args.max_failures]

    # Determine world config from phase name
    world_config = {}
    if "phase3" in phase_name:
        # Extract layout name from phase (e.g., phase3_hard_constrained → hard)
        parts = phase_name.replace("phase3_", "").split("_")
        layout_name = parts[0] if parts else None
        layouts = cfg.get("obstacle_layouts", {})
        if layout_name and layout_name in layouts:
            layout_cfg = layouts[layout_name]
            # Build world config from layout
            world_config = {"cuboid": {}, "mesh": {}}
            if "cuboid" in layout_cfg:
                for name, obs in layout_cfg["cuboid"].items():
                    world_config["cuboid"][name] = {
                        "dims": obs["dims"], "pose": obs["pose"]
                    }

    # Build diagnostic engine
    engine = FailureDiagnosticEngine(
        robot_file=cfg["robot"]["config_file"],
        world_config=world_config,
        joint_names=cfg["robot"]["joint_names"],
    )

    # Run diagnostics
    diagnoses = []
    for i, failure in enumerate(failures):
        # Remap keys if needed (per_problem uses "problem_id", problems use "id")
        if "id" not in failure and "problem_id" in failure:
            failure["id"] = failure["problem_id"]

        print(f"\n  Diagnosing failure {i+1}/{len(failures)} "
              f"(problem #{failure.get('id', '?')})...")

        diag = engine.diagnose(failure)
        diagnoses.append(diag)

        print(f"    Root cause: {diag.root_cause}")
        print(f"    {diag.details[:100]}...")

        # Visualize if requested
        if not args.headless and (args.interactive or args.problem_id is not None):
            visualize_failure_in_sim(diag, engine)

    # Generate report
    generate_report(diagnoses, phase_name)

    simulation_app.close()


if __name__ == "__main__":
    main()
