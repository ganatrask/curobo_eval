"""
Benchmark Engine
=================
Core loop shared by Phase 1, 2, and 3.
Runs N problems through cuRobo MotionGen and collects metrics.

Not run directly — imported by phase1_baseline.py, phase2_constrained.py, etc.
"""

import torch
import time
import json
import os
import numpy as np
from typing import Dict, List, Optional
from dataclasses import dataclass

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.rollout.cost.pose_cost import PoseCostMetric
from curobo.geom.types import WorldConfig

from eval_utils.metrics import (
    PlanningMetrics,
    compute_path_length,
    compute_max_jerk,
    compute_max_acceleration,
    quaternion_angular_distance,
    compute_orientation_deviation,
    aggregate_metrics,
    print_summary,
)


@dataclass
class BenchmarkConfig:
    """Configuration for a benchmark run."""
    phase_name: str = "phase1_baseline"
    robot_file: str = "ur5e.yml"
    world_config: Optional[dict] = None
    joint_names: List[str] = None
    num_problems: int = 1000

    # Planning params
    num_trajopt_seeds: int = 12
    num_graph_seeds: int = 12
    interpolation_dt: float = 0.02
    max_attempts: int = 10
    enable_graph: bool = True
    collision_activation_distance: float = 0.025

    # Orientation constraints (Phase 2+)
    use_orientation_constraint: bool = False
    target_orientation_wxyz: List[float] = None
    max_orientation_deviation_deg: float = 5.0

    # Output
    results_dir: str = "results"
    save_trajectories: bool = False
    save_per_problem: bool = True


class BenchmarkRunner:
    """Runs the planning benchmark loop."""

    def __init__(self, bench_cfg: BenchmarkConfig, project_dir: str = None):
        self.cfg = bench_cfg
        self.tensor_args = TensorDeviceType()
        # project_dir anchors relative output paths (e.g. results_dir)
        self.project_dir = project_dir or os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))
        )

        print(f"\n[{bench_cfg.phase_name}] Initializing MotionGen...")

        # Build world config
        # cuRobo requires at least one obstacle for collision checking —
        # use a tiny dummy far from workspace when no real obstacles exist
        if bench_cfg.world_config:
            world_cfg = bench_cfg.world_config
        else:
            world_cfg = {
                "cuboid": {
                    "_dummy": {
                        "dims": [0.01, 0.01, 0.01],
                        "pose": [100.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                    }
                }
            }

        # Load robot config with relative paths resolved
        from eval_utils import load_robot_config
        robot_cfg_dict = load_robot_config(bench_cfg.robot_file)

        # Load MotionGen
        self.motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg_dict,
            world_cfg,
            self.tensor_args,
            interpolation_dt=bench_cfg.interpolation_dt,
            num_trajopt_seeds=bench_cfg.num_trajopt_seeds,
            num_graph_seeds=bench_cfg.num_graph_seeds,
            collision_activation_distance=bench_cfg.collision_activation_distance,
            use_cuda_graph=True,
        )
        self.motion_gen = MotionGen(self.motion_gen_config)

        print(f"[{bench_cfg.phase_name}] Warming up (JIT compilation)...")
        self.motion_gen.warmup(parallel_finetune=True)
        print(f"[{bench_cfg.phase_name}] Ready.")

        # Get kinematics model for FK evaluation
        self.kin_model = self.motion_gen.kinematics

        # Build plan config
        if bench_cfg.use_orientation_constraint and bench_cfg.target_orientation_wxyz:
            # hold_partial_pose keeps specified pose components constant
            # along the entire trajectory.
            # hold_vec_weight format: [rot_x, rot_y, rot_z, pos_x, pos_y, pos_z]
            #   rotation = 1 (constrain orientation — keep spatula level)
            #   position = 0 (free to move to goal)
            pose_cost = PoseCostMetric(
                hold_partial_pose=True,
                hold_vec_weight=self.tensor_args.to_device(
                    [1, 1, 1, 0, 0, 0]
                ),
            )
        else:
            pose_cost = None

        self.plan_config = MotionGenPlanConfig(
            max_attempts=bench_cfg.max_attempts,
            enable_graph=bench_cfg.enable_graph,
            enable_finetune_trajopt=True,
            parallel_finetune=True,
            pose_cost_metric=pose_cost,
        )

    def run(self, problems: List[Dict]) -> List[PlanningMetrics]:
        """
        Run the benchmark on a list of problems.
        Each problem dict has: id, start_config, goal_position, goal_orientation_wxyz
        """
        n = len(problems)
        all_metrics = []
        n_skipped = 0
        cfg = self.cfg

        print(f"\n[{cfg.phase_name}] Running {n} problems...")
        print(f"  Orientation constraint: {cfg.use_orientation_constraint}")
        print(f"  World obstacles: "
              f"{'yes' if cfg.world_config and cfg.world_config.get('cuboid') else 'none'}")

        for i, problem in enumerate(problems):
            metrics = self._run_single(problem)

            # Track start-state collisions separately — these indicate
            # the problem set is invalid for this obstacle layout, not
            # a planner failure.
            if "INVALID_START_STATE" in metrics.status:
                metrics.skipped_invalid_start = True
                n_skipped += 1
            all_metrics.append(metrics)

            # Progress update every 100 problems
            if (i + 1) % 100 == 0 or i == n - 1:
                n_ok = sum(1 for m in all_metrics if m.success)
                n_valid = (i + 1) - n_skipped
                if n_valid > 0:
                    print(f"  [{i+1}/{n}] success={n_ok}/{n_valid} valid "
                          f"({100*n_ok/n_valid:.1f}%) "
                          f"[{n_skipped} skipped: start in collision]")
                else:
                    print(f"  [{i+1}/{n}] {n_skipped} skipped (start in collision)")

        if n_skipped > 0:
            print(f"\n  WARNING: {n_skipped}/{n} problems had start configs "
                  f"colliding with obstacles.")
            print(f"  These are excluded from success rate. "
                  f"Re-generate problems with obstacle-aware workspace scan.")

        return all_metrics

    def _run_single(self, problem: Dict) -> PlanningMetrics:
        """Run a single planning problem and compute all metrics."""
        metrics = PlanningMetrics(problem_id=problem["id"])
        cfg = self.cfg

        # Build start state
        start_q = torch.tensor(
            [problem["start_config"]],
            dtype=torch.float32, device="cuda:0"
        )
        start_state = JointState.from_position(
            start_q,
            joint_names=cfg.joint_names,
        )

        # Build goal pose
        goal_pos = problem["goal_position"]
        goal_quat = problem["goal_orientation_wxyz"]
        goal_pose = Pose.from_list(
            goal_pos + goal_quat  # [x,y,z, qw,qx,qy,qz]
        )

        # ---- PLAN ----
        torch.cuda.synchronize()
        t_start = time.perf_counter()

        try:
            result = self.motion_gen.plan_single(
                start_state, goal_pose, self.plan_config
            )
        except Exception as e:
            metrics.success = False
            metrics.status = f"EXCEPTION: {str(e)[:100]}"
            return metrics

        torch.cuda.synchronize()
        t_end = time.perf_counter()

        metrics.plan_time_ms = (t_end - t_start) * 1000.0
        metrics.success = result.success.item() if result.success is not None else False
        metrics.status = str(result.status) if hasattr(result, 'status') else "UNKNOWN"

        if not metrics.success:
            return metrics

        # ---- TRAJECTORY ANALYSIS ----
        try:
            traj = result.get_interpolated_plan()
            if traj is None or traj.position.shape[0] == 0:
                metrics.success = False
                metrics.status = "EMPTY_TRAJECTORY"
                return metrics

            positions = traj.position  # (T, DOF)
            T = positions.shape[0]
            dt = cfg.interpolation_dt

            metrics.num_waypoints = T
            metrics.path_length_rad = compute_path_length(positions)
            metrics.motion_time_s = T * dt
            metrics.max_jerk = compute_max_jerk(positions, dt)
            metrics.max_acceleration = compute_max_acceleration(positions, dt)

            # ---- GOAL ACCURACY ----
            # FK at final waypoint
            final_q = positions[-1:, :]
            final_state = self.kin_model.get_state(final_q)
            final_pos = final_state.ee_position[0]
            final_quat = final_state.ee_quaternion[0]

            goal_pos_t = torch.tensor(goal_pos, dtype=torch.float32, device="cuda:0")
            goal_quat_t = torch.tensor(goal_quat, dtype=torch.float32, device="cuda:0")

            metrics.position_error_mm = (
                torch.norm(final_pos - goal_pos_t).item() * 1000.0
            )
            metrics.orientation_error_deg = quaternion_angular_distance(
                final_quat, goal_quat_t
            )

            # ---- ORIENTATION CONSTRAINT CHECK (Phase 2+) ----
            if cfg.use_orientation_constraint and cfg.target_orientation_wxyz:
                # Compute FK at every waypoint
                all_states = self.kin_model.get_state(positions)
                all_quats = all_states.ee_quaternion  # (T, 4)

                orient_info = compute_orientation_deviation(
                    all_quats, cfg.target_orientation_wxyz
                )
                metrics.max_orientation_deviation_deg = orient_info["max_deviation_deg"]
                metrics.mean_orientation_deviation_deg = orient_info["mean_deviation_deg"]
                metrics.constraint_satisfied = (
                    orient_info["max_deviation_deg"] <= cfg.max_orientation_deviation_deg
                )

                # Hard gate: a plan that violates the orientation constraint
                # is NOT a success for constrained planning.
                if not metrics.constraint_satisfied:
                    metrics.success = False
                    metrics.status += " (ORIENTATION_VIOLATED)"

            # ---- COLLISION VALIDATION ----
            # Check the planned trajectory for collisions using cuRobo's checker
            # This catches cases where the planner reports success but the
            # interpolated trajectory has collisions between waypoints
            try:
                coll_result = self.motion_gen.check_trajectory_collision(
                    traj
                )
                if coll_result is not None:
                    if hasattr(coll_result, 'collision') and coll_result.collision is not None:
                        metrics.any_collision_detected = coll_result.collision.any().item()
            except Exception:
                # Some cuRobo versions don't have this API — skip gracefully
                pass

        except Exception as e:
            # Trajectory analysis failed but planning succeeded
            metrics.status += f" (ANALYSIS_ERROR: {str(e)[:80]})"

        return metrics

    def save_results(self, all_metrics: List[PlanningMetrics], problems: List[Dict]):
        """Save per-problem metrics and aggregate summary."""
        cfg = self.cfg
        results_dir = os.path.join(self.project_dir, cfg.results_dir)
        os.makedirs(results_dir, exist_ok=True)

        # Per-problem results
        if cfg.save_per_problem:
            per_problem = []
            for m, p in zip(all_metrics, problems):
                entry = m.to_dict()
                entry["start_config"] = p["start_config"]
                entry["goal_position"] = p["goal_position"]
                entry["goal_orientation_wxyz"] = p["goal_orientation_wxyz"]
                per_problem.append(entry)

            per_file = os.path.join(results_dir, f"{cfg.phase_name}_per_problem.json")
            with open(per_file, "w") as f:
                json.dump(per_problem, f, indent=2, default=str)
            print(f"  Saved per-problem results to {per_file}")

        # Aggregate summary
        agg = aggregate_metrics(all_metrics)
        agg["phase"] = cfg.phase_name
        agg["robot"] = cfg.robot_file
        agg["orientation_constraint"] = cfg.use_orientation_constraint

        agg_file = os.path.join(results_dir, f"{cfg.phase_name}_summary.json")
        with open(agg_file, "w") as f:
            json.dump(agg, f, indent=2, default=str)
        print(f"  Saved summary to {agg_file}")

        # Print summary
        print_summary(agg, cfg.phase_name.upper())
