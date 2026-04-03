"""
Phase 2: Constrained Planning Benchmark
=========================================
Same problems as Phase 1, but with orientation constraints active.
The spatula must stay level (within max_orientation_deviation_deg).

This phase directly answers: "Can cuRobo keep the spatula flat during motion?"

Usage:
    omni_python scripts/phase2_constrained.py --headless
"""

try:
    import isaacsim
except ImportError:
    pass

import argparse
import os
import sys
import yaml

parser = argparse.ArgumentParser(description="Phase 2: Constrained Benchmark")
parser.add_argument("--config", type=str, default="config/eval_config.yaml")
parser.add_argument("--num_problems", type=int, default=None)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--problems_file", type=str, default=None)
args = parser.parse_args()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": args.headless, "width": "1920", "height": "1080"})

script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
sys.path.insert(0, script_dir)

with open(os.path.join(project_dir, args.config), "r") as f:
    cfg = yaml.safe_load(f)

from eval_utils.benchmark_engine import BenchmarkRunner, BenchmarkConfig
import json


def main():
    print("\n" + "#"*60)
    print("  cuRobo Evaluation — Phase 2: Orientation Constraints")
    print("#"*60)

    n_problems = args.num_problems or cfg["planning"]["num_problems"]
    pcfg = cfg["planning"]
    tool_cfg = cfg.get("tool", {})

    # SAME as Phase 1 but WITH orientation constraint
    bench_cfg = BenchmarkConfig(
        phase_name="phase2_constrained",
        robot_file=cfg["robot"]["config_file"],
        world_config=None,  # Still no obstacles
        joint_names=cfg["robot"]["joint_names"],
        num_problems=n_problems,
        num_trajopt_seeds=pcfg["num_trajopt_seeds"],
        num_graph_seeds=pcfg["num_graph_seeds"],
        interpolation_dt=pcfg["interpolation_dt"],
        max_attempts=pcfg["max_attempts"],
        enable_graph=pcfg["enable_graph"],
        collision_activation_distance=pcfg["collision_activation_distance"],
        # ---- THE KEY DIFFERENCE ----
        use_orientation_constraint=True,
        target_orientation_wxyz=tool_cfg.get("flat_orientation_wxyz", [0, 1, 0, 0]),
        max_orientation_deviation_deg=tool_cfg.get("max_orientation_deviation_deg", 5.0),
        # ----------------------------
        results_dir=cfg["output"]["results_dir"],
        save_per_problem=cfg["output"]["save_per_problem"],
    )

    # Load same problems as Phase 1 for direct comparison
    problems_file = args.problems_file or os.path.join(
        project_dir, "results", "problems_baseline.json"
    )
    if not os.path.exists(problems_file):
        print(f"  ERROR: {problems_file} not found.")
        print(f"  Run Phase 0 or Phase 1 first to generate problems.")
        simulation_app.close()
        return

    with open(problems_file, "r") as f:
        data = json.load(f)
    problems = data["problems"][:n_problems]

    # Run benchmark
    runner = BenchmarkRunner(bench_cfg)
    all_metrics = runner.run(problems)
    runner.save_results(all_metrics, problems)

    # Compare with Phase 1 if available
    p1_summary = os.path.join(project_dir, "results", "phase1_baseline_summary.json")
    if os.path.exists(p1_summary):
        with open(p1_summary, "r") as f:
            p1 = json.load(f)
        print("\n  --- Phase 1 vs Phase 2 Comparison ---")
        print(f"  Success rate:  P1={p1['success_rate_pct']:.1f}%  vs  "
              f"P2={100*sum(1 for m in all_metrics if m.success)/len(all_metrics):.1f}%")
        if p1.get("plan_time_ms"):
            print(f"  Plan time:     P1={p1['plan_time_ms']['median']:.1f}ms  vs  "
                  f"P2 (see summary above)")
        print(f"  The difference shows the cost of enforcing orientation constraints.")

    simulation_app.close()


if __name__ == "__main__":
    main()
