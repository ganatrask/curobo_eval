"""
Phase 3: Static Obstacle Benchmark
====================================
Tests motion planning with fixed obstacles AND orientation constraints.
Runs every obstacle layout defined in eval_config.yaml (easy, medium, hard).

For each layout, runs both constrained and unconstrained for comparison,
giving you a 2x3 matrix of results.

Usage:
    omni_python scripts/phase3_obstacles.py --headless
    omni_python scripts/phase3_obstacles.py --headless --layout easy  # single layout
"""

try:
    import isaacsim
except ImportError:
    pass

import argparse
import os
import sys
import yaml
import json

parser = argparse.ArgumentParser(description="Phase 3: Obstacle Benchmark")
parser.add_argument("--config", type=str, default="config/eval_config.yaml")
parser.add_argument("--num_problems", type=int, default=None)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--layout", type=str, default=None,
                    help="Run only this layout (e.g., 'easy'). Default: run all.")
parser.add_argument("--skip_unconstrained", action="store_true",
                    help="Only run constrained tests")
args = parser.parse_args()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": args.headless, "width": "1920", "height": "1080"})

script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
sys.path.insert(0, script_dir)

with open(os.path.join(project_dir, args.config), "r") as f:
    cfg = yaml.safe_load(f)

from eval_utils.benchmark_engine import BenchmarkRunner, BenchmarkConfig
from eval_utils.problem_generator import ProblemGenerator
from curobo.types.base import TensorDeviceType

tensor_args = TensorDeviceType()


def build_world_config(layout_cfg: dict) -> dict:
    """
    Convert obstacle layout from eval_config.yaml format
    to cuRobo's WorldConfig dict format.

    eval_config format:
        cuboid:
          table:
            dims: [1.2, 0.8, 0.05]
            pose: [0.45, 0.0, -0.025, 1, 0, 0, 0]

    cuRobo format:
        {"cuboid": {"table": {"dims": [...], "pose": [...]}}}
    """
    world = {"cuboid": {}, "mesh": {}}

    if "cuboid" in layout_cfg:
        for name, obs in layout_cfg["cuboid"].items():
            world["cuboid"][name] = {
                "dims": obs["dims"],
                "pose": obs["pose"],
            }

    if "mesh" in layout_cfg:
        for name, obs in layout_cfg["mesh"].items():
            world["mesh"][name] = {
                "pose": obs["pose"],
                "file_path": obs["file_path"],
            }

    return world


def generate_problems_for_layout(layout_name: str, world_config: dict, n: int):
    """
    Generate problems that are valid for a specific obstacle layout.
    This re-runs workspace discovery WITH obstacles, so goals are
    guaranteed to be reachable even with obstacles present.
    """
    ws = cfg["workspace"]
    tool_cfg = cfg.get("tool", {})

    problems_file = os.path.join(
        project_dir, "results", f"problems_{layout_name}.json"
    )

    if os.path.exists(problems_file):
        print(f"  Loading existing problems for '{layout_name}'")
        with open(problems_file, "r") as f:
            data = json.load(f)
        return data["problems"][:n]

    print(f"  Generating {n} problems for '{layout_name}' layout...")
    print(f"  (This includes IK reachability scan WITH obstacles — may take a few minutes)")

    gen = ProblemGenerator(
        robot_file=cfg["robot"]["config_file"],
        world_config=world_config,  # <-- obstacles included in IK scan
        workspace_bounds={
            "x_range": ws["x_range"],
            "y_range": ws["y_range"],
            "z_range": ws["z_range"],
        },
        tensor_args=tensor_args,
        orientation_wxyz=tool_cfg.get("flat_orientation_wxyz", [0, 1, 0, 0]),
        orientation_mode=ws.get("orientation_mode", "fixed"),
        reachability_resolution=0.06,  # Slightly coarser for speed
    )

    problems = gen.generate(n=n, seed=42)
    gen.save_problems(problems, problems_file)
    return problems


def run_layout(
    layout_name: str,
    layout_cfg: dict,
    problems: list,
    use_constraint: bool,
):
    """Run benchmark for one layout + constraint combination."""
    pcfg = cfg["planning"]
    tool_cfg = cfg.get("tool", {})
    constraint_tag = "constrained" if use_constraint else "unconstrained"
    phase_name = f"phase3_{layout_name}_{constraint_tag}"

    print(f"\n{'='*60}")
    print(f"  Layout: {layout_name} | Constraint: {constraint_tag}")
    print(f"  Description: {layout_cfg.get('description', 'N/A')}")
    print(f"{'='*60}")

    world_config = build_world_config(layout_cfg)

    # Count obstacles
    n_obs = len(world_config.get("cuboid", {})) + len(world_config.get("mesh", {}))
    print(f"  Obstacles: {n_obs} "
          f"(cuboids: {len(world_config.get('cuboid', {}))}, "
          f"meshes: {len(world_config.get('mesh', {}))})")

    bench_cfg = BenchmarkConfig(
        phase_name=phase_name,
        robot_file=cfg["robot"]["config_file"],
        world_config=world_config,
        joint_names=cfg["robot"]["joint_names"],
        num_problems=len(problems),
        num_trajopt_seeds=pcfg["num_trajopt_seeds"],
        num_graph_seeds=pcfg["num_graph_seeds"],
        interpolation_dt=pcfg["interpolation_dt"],
        max_attempts=pcfg["max_attempts"],
        enable_graph=pcfg["enable_graph"],
        collision_activation_distance=pcfg["collision_activation_distance"],
        use_orientation_constraint=use_constraint,
        target_orientation_wxyz=tool_cfg.get("flat_orientation_wxyz", [0, 1, 0, 0]),
        max_orientation_deviation_deg=tool_cfg.get("max_orientation_deviation_deg", 5.0),
        results_dir=cfg["output"]["results_dir"],
        save_per_problem=cfg["output"]["save_per_problem"],
    )

    runner = BenchmarkRunner(bench_cfg)
    all_metrics = runner.run(problems)
    runner.save_results(all_metrics, problems)

    return all_metrics


def main():
    print("\n" + "#"*60)
    print("  cuRobo Evaluation — Phase 3: Static Obstacles")
    print("#"*60)

    obstacle_layouts = cfg.get("obstacle_layouts", {})
    if not obstacle_layouts:
        print("  ERROR: No obstacle_layouts defined in eval_config.yaml")
        simulation_app.close()
        return

    n_problems = args.num_problems or cfg["planning"]["num_problems"]

    # Filter to single layout if specified
    if args.layout:
        if args.layout not in obstacle_layouts:
            print(f"  ERROR: Layout '{args.layout}' not found. "
                  f"Available: {list(obstacle_layouts.keys())}")
            simulation_app.close()
            return
        layouts_to_run = {args.layout: obstacle_layouts[args.layout]}
    else:
        layouts_to_run = obstacle_layouts

    # Run all layouts
    all_results = {}
    for layout_name, layout_cfg in layouts_to_run.items():
        world_config = build_world_config(layout_cfg)

        # Generate problems valid for this obstacle layout
        problems = generate_problems_for_layout(layout_name, world_config, n_problems)

        # Run unconstrained (unless skipped)
        if not args.skip_unconstrained:
            metrics_uc = run_layout(layout_name, layout_cfg, problems, use_constraint=False)
            all_results[f"{layout_name}_unconstrained"] = metrics_uc

        # Run constrained
        metrics_c = run_layout(layout_name, layout_cfg, problems, use_constraint=True)
        all_results[f"{layout_name}_constrained"] = metrics_c

    # Print comparison table
    print("\n" + "="*70)
    print("  PHASE 3 COMPARISON MATRIX")
    print("="*70)
    print(f"  {'Layout':<12} {'Constraint':<15} {'Success%':<10} "
          f"{'PlanTime(ms)':<14} {'MaxOrient(°)':<14}")
    print("-"*70)

    for key, metrics in all_results.items():
        n_ok = sum(1 for m in metrics if m.success)
        n_tot = len(metrics)
        sr = 100 * n_ok / n_tot
        pt = sum(m.plan_time_ms for m in metrics if m.success) / max(n_ok, 1)
        orient = sum(m.max_orientation_deviation_deg for m in metrics if m.success) / max(n_ok, 1)

        parts = key.rsplit("_", 1)
        layout = parts[0]
        constraint = parts[1] if len(parts) > 1 else ""

        print(f"  {layout:<12} {constraint:<15} {sr:<10.1f} "
              f"{pt:<14.1f} {orient:<14.2f}")

    print("="*70)
    print("\n  All Phase 3 results saved to results/ directory.")
    print("  Next: run scripts/analyze_results.py for cross-phase analysis.\n")

    simulation_app.close()


if __name__ == "__main__":
    main()
