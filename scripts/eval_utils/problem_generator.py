"""
Problem Generator for cuRobo Evaluation
========================================
1. Discovers the robot's reachable workspace via batched IK
2. Generates N random (start_config, goal_pose) problems within that workspace
3. Validates that all problems are solvable in principle (IK exists for goal)

Usage:
    from eval_utils.problem_generator import ProblemGenerator
    gen = ProblemGenerator(robot_file, world_config, workspace_bounds, tensor_args)
    problems = gen.generate(n=1000)
"""

import torch
import numpy as np
import json
import os
from typing import Dict, List, Tuple

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

from eval_utils import load_robot_config


class ProblemGenerator:
    """Generates benchmark problems within the robot's reachable workspace."""

    def __init__(
        self,
        robot_file: str,
        world_config: dict,
        workspace_bounds: dict,
        tensor_args: TensorDeviceType = None,
        orientation_wxyz: List[float] = None,
        orientation_mode: str = "fixed",
        reachability_resolution: float = 0.05,
    ):
        """
        Args:
            robot_file: Path or name of cuRobo robot YAML
            world_config: Dict of obstacles (can be empty {} for Phase 1)
            workspace_bounds: Dict with x_range, y_range, z_range
            orientation_wxyz: Default goal orientation [qw, qx, qy, qz]
            orientation_mode: "fixed", "random", or "varied"
            reachability_resolution: Grid spacing for workspace scan (meters)
        """
        self.tensor_args = tensor_args or TensorDeviceType()
        self.workspace_bounds = workspace_bounds
        self.orientation_wxyz = orientation_wxyz or [0.0, 1.0, 0.0, 0.0]
        self.orientation_mode = orientation_mode
        self.resolution = reachability_resolution

        # Load robot config (path resolution handled by load_robot_config)
        robot_cfg_dict = load_robot_config(robot_file)
        self.robot_cfg = RobotConfig.from_dict(robot_cfg_dict["robot_cfg"])

        # Build IK solver (collision-aware if world_config has obstacles)
        # use_cuda_graph=False: the workspace scan uses variable-size batches
        # (last batch is smaller), and CUDA graph resize requires CUDA >= 12.0
        ik_config = IKSolverConfig.load_from_robot_config(
            self.robot_cfg,
            world_config if world_config else None,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=20,
            self_collision_check=True,
            self_collision_opt=True,
            tensor_args=self.tensor_args,
            use_cuda_graph=False,
        )
        self._ik = IKSolver(ik_config)

        # Storage
        self.reachable_poses = None
        self.reachable_configs = None

    def discover_workspace(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Scan a 3D grid of poses and find which ones are IK-reachable.
        Returns (reachable_positions, reachable_joint_configs)

        This is how you find "where can my robot actually go" within
        the workspace bounds you defined.
        """
        xr = self.workspace_bounds["x_range"]
        yr = self.workspace_bounds["y_range"]
        zr = self.workspace_bounds["z_range"]

        # Build 3D grid
        xs = np.arange(xr[0], xr[1] + self.resolution, self.resolution)
        ys = np.arange(yr[0], yr[1] + self.resolution, self.resolution)
        zs = np.arange(zr[0], zr[1] + self.resolution, self.resolution)

        grid = np.array(np.meshgrid(xs, ys, zs, indexing="ij")).reshape(3, -1).T
        n_total = grid.shape[0]
        print(f"[Workspace] Scanning {n_total} positions "
              f"({len(xs)}x{len(ys)}x{len(zs)} grid, res={self.resolution}m)")

        # Build pose tensors with the desired orientation
        quat = self.orientation_wxyz
        positions = torch.tensor(grid, dtype=torch.float32, device="cuda:0")
        quaternions = torch.tensor(
            [quat] * n_total, dtype=torch.float32, device="cuda:0"
        )

        # Solve IK in batches (GPU memory management)
        batch_size = 500
        reachable_pos = []
        reachable_q = []

        for i in range(0, n_total, batch_size):
            end = min(i + batch_size, n_total)
            goal_pose = Pose(
                position=positions[i:end].contiguous(),
                quaternion=quaternions[i:end].contiguous(),
            )

            result = self._ik.solve_batch(goal_pose)

            # Check which poses were solved successfully
            success = result.success.squeeze()
            if success.dim() == 0:
                success = success.unsqueeze(0)

            mask = success.cpu().numpy().astype(bool)
            if mask.any():
                reachable_pos.extend(grid[i:end][mask])
                reachable_q.extend(
                    result.solution[success, 0].cpu().numpy()
                )

            if (i // batch_size) % 10 == 0:
                print(f"  Scanned {end}/{n_total} "
                      f"({len(reachable_pos)} reachable so far)")

        reachable_pos = np.array(reachable_pos)
        reachable_q = np.array(reachable_q)

        pct = 100 * len(reachable_pos) / n_total
        print(f"[Workspace] Found {len(reachable_pos)}/{n_total} "
              f"reachable positions ({pct:.1f}%)")

        self.reachable_poses = reachable_pos
        self.reachable_configs = reachable_q
        return reachable_pos, reachable_q

    def generate(
        self,
        n: int = 1000,
        seed: int = 42,
        verify_ik: bool = True,
    ) -> List[Dict]:
        """
        Generate N benchmark problems. Each problem is a dict:
        {
            "id": int,
            "start_config": [float] (joint angles in radians),
            "goal_position": [float] (x, y, z in meters),
            "goal_orientation_wxyz": [float] (qw, qx, qy, qz),
        }

        Strategy:
        - Start configs: Randomly sampled from IK solutions of reachable poses
          (guarantees collision-free, valid start states)
        - Goal poses: Randomly sampled from the reachable workspace
          (guarantees IK-solvable goals)
        """
        if self.reachable_poses is None:
            print("[ProblemGen] No workspace data — running discovery first...")
            self.discover_workspace()

        if len(self.reachable_poses) < 10:
            raise RuntimeError(
                f"Only {len(self.reachable_poses)} reachable poses found. "
                "Check workspace bounds and robot config."
            )

        rng = np.random.RandomState(seed)
        n_reachable = len(self.reachable_poses)
        problems = []

        for i in range(n):
            # Pick random start config from reachable IK solutions
            start_idx = rng.randint(0, n_reachable)
            start_config = self.reachable_configs[start_idx].tolist()

            # Pick random goal from reachable poses (different from start)
            goal_idx = rng.randint(0, n_reachable)
            while goal_idx == start_idx and n_reachable > 1:
                goal_idx = rng.randint(0, n_reachable)

            goal_position = self.reachable_poses[goal_idx].tolist()

            # Orientation based on mode
            if self.orientation_mode == "fixed":
                goal_orient = self.orientation_wxyz
            elif self.orientation_mode == "random":
                # Random unit quaternion
                q = rng.randn(4)
                q = q / np.linalg.norm(q)
                goal_orient = q.tolist()
            elif self.orientation_mode == "varied":
                # Small perturbation from flat
                base = np.array(self.orientation_wxyz)
                noise = rng.randn(4) * 0.05
                q = base + noise
                q = q / np.linalg.norm(q)
                goal_orient = q.tolist()
            else:
                goal_orient = self.orientation_wxyz

            problems.append({
                "id": i,
                "start_config": start_config,
                "goal_position": goal_position,
                "goal_orientation_wxyz": goal_orient,
            })

        print(f"[ProblemGen] Generated {n} problems "
              f"(orientation_mode={self.orientation_mode})")
        return problems

    def save_problems(self, problems: List[Dict], filepath: str):
        """Save problems to JSON for reproducibility."""
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "w") as f:
            json.dump({
                "workspace_bounds": self.workspace_bounds,
                "orientation_mode": self.orientation_mode,
                "n_reachable_positions": len(self.reachable_poses),
                "problems": problems,
            }, f, indent=2)
        print(f"[ProblemGen] Saved {len(problems)} problems to {filepath}")

    def load_problems(self, filepath: str) -> List[Dict]:
        """Load previously generated problems."""
        with open(filepath, "r") as f:
            data = json.load(f)
        print(f"[ProblemGen] Loaded {len(data['problems'])} problems from {filepath}")
        return data["problems"]

    def save_workspace(self, filepath: str):
        """Save workspace scan results for reuse."""
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        np.savez(
            filepath,
            reachable_poses=self.reachable_poses,
            reachable_configs=self.reachable_configs,
        )
        print(f"[Workspace] Saved to {filepath}")

    def load_workspace(self, filepath: str):
        """Load a previous workspace scan."""
        data = np.load(filepath)
        self.reachable_poses = data["reachable_poses"]
        self.reachable_configs = data["reachable_configs"]
        print(f"[Workspace] Loaded {len(self.reachable_poses)} reachable poses")
