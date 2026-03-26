"""
Phase 1: Baseline Planning Benchmark
======================================
Tests A→B motion planning with NO obstacles and NO constraints.
This establishes the performance baseline for your robot.

Usage:
    omni_python scripts/phase1_baseline.py --headless
    omni_python scripts/phase1_baseline.py --headless --num_problems 100  # quick test
"""

try:
    import isaacsim
except ImportError:
    pass

import argparse
import os
import sys
import yaml

parser = argparse.ArgumentParser(description="Phase 1: Baseline Benchmark")
parser.add_argument("--config", type=str, default="config/eval_config.yaml")
parser.add_argument("--num_problems", type=int, default=None)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--problems_file", type=str, default=None,
                    help="Load pre-generated problems (from Phase 0)")
args = parser.parse_args()

# Launch Isaac Sim
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": args.headless, "width": "1920", "height": "1080"})

# Load config
script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
sys.path.insert(0, script_dir)

with open(os.path.join(project_dir, args.config), "r") as f:
    cfg = yaml.safe_load(f)

from eval_utils.benchmark_engine import BenchmarkRunner, BenchmarkConfig
from eval_utils.problem_generator import ProblemGenerator
from curobo.types.base import TensorDeviceType
import json

tensor_args = TensorDeviceType()


def main():
    print("\n" + "#"*60)
    print("  cuRobo Evaluation — Phase 1: Baseline (no obstacles)")
    print("#"*60)

    n_problems = args.num_problems or cfg["planning"]["num_problems"]
    pcfg = cfg["planning"]

    # Build benchmark config — NO world obstacles, NO constraints
    bench_cfg = BenchmarkConfig(
        phase_name="phase1_baseline",
        robot_file=cfg["robot"]["config_file"],
        world_config=None,  # <-- No obstacles
        joint_names=cfg["robot"]["joint_names"],
        num_problems=n_problems,
        num_trajopt_seeds=pcfg["num_trajopt_seeds"],
        num_graph_seeds=pcfg["num_graph_seeds"],
        interpolation_dt=pcfg["interpolation_dt"],
        max_attempts=pcfg["max_attempts"],
        enable_graph=pcfg["enable_graph"],
        collision_activation_distance=pcfg["collision_activation_distance"],
        use_orientation_constraint=False,  # <-- No constraint
        results_dir=cfg["output"]["results_dir"],
        save_per_problem=cfg["output"]["save_per_problem"],
    )

    # Load or generate problems
    problems_file = args.problems_file or os.path.join(
        project_dir, "results", "problems_baseline.json"
    )

    if os.path.exists(problems_file):
        print(f"\n  Loading problems from {problems_file}")
        with open(problems_file, "r") as f:
            data = json.load(f)
        problems = data["problems"][:n_problems]
    else:
        print(f"\n  No problems file found — generating {n_problems} problems...")
        ws = cfg["workspace"]
        tool_cfg = cfg.get("tool", {})
        gen = ProblemGenerator(
            robot_file=cfg["robot"]["config_file"],
            world_config={},
            workspace_bounds={
                "x_range": ws["x_range"],
                "y_range": ws["y_range"],
                "z_range": ws["z_range"],
            },
            tensor_args=tensor_args,
            orientation_wxyz=tool_cfg.get("flat_orientation_wxyz", [0, 1, 0, 0]),
            orientation_mode=ws.get("orientation_mode", "fixed"),
        )
        problems = gen.generate(n=n_problems, seed=42)
        gen.save_problems(problems, problems_file)

    # Run benchmark
    runner = BenchmarkRunner(bench_cfg)
    all_metrics = runner.run(problems)
    runner.save_results(all_metrics, problems)

    simulation_app.close()


if __name__ == "__main__":
    main()
