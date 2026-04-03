"""
Results Analyzer
=================
Loads all phase results and produces a cross-phase comparison.
Run after any/all phases complete.

Usage:
    omni_python scripts/analyze_results.py
    python scripts/analyze_results.py   # No Isaac Sim needed
"""

import json
import os
import glob
import sys
from typing import Dict, List


def load_summaries(results_dir: str) -> Dict[str, dict]:
    """Load all *_summary.json files from results directory."""
    summaries = {}
    pattern = os.path.join(results_dir, "*_summary.json")
    for filepath in sorted(glob.glob(pattern)):
        name = os.path.basename(filepath).replace("_summary.json", "")
        with open(filepath, "r") as f:
            summaries[name] = json.load(f)
    return summaries


def print_cross_phase_report(summaries: Dict[str, dict]):
    """Print a comprehensive comparison across all phases."""

    print("\n" + "="*80)
    print("  cuRobo EVALUATION REPORT — Cross-Phase Comparison")
    print("="*80)

    if not summaries:
        print("  No results found. Run Phase 0-3 first.")
        return

    # Header
    print(f"\n  {'Phase':<35} {'Success%':<10} {'PlanTime':<12} "
          f"{'PathLen':<10} {'MaxJerk':<10} {'OrientDev':<12}")
    print(f"  {'':35} {'':10} {'median(ms)':<12} "
          f"{'mean(rad)':<10} {'p98(r/s³)':<10} {'mean(°)':<12}")
    print("-"*80)

    for name, s in summaries.items():
        sr = f"{s['success_rate_pct']:.1f}%"

        if s["successes"] == 0:
            print(f"  {name:<35} {sr:<10} {'—':<12} {'—':<10} {'—':<10} {'—':<12}")
            continue

        pt = f"{s['plan_time_ms']['median']:.1f}" if 'plan_time_ms' in s else "—"
        pl = f"{s['path_length_rad']['mean']:.2f}" if 'path_length_rad' in s else "—"
        jk = f"{s['max_jerk']['p98']:.1f}" if 'max_jerk' in s else "—"
        od = f"{s['max_orientation_deviation_deg']['mean']:.2f}" if 'max_orientation_deviation_deg' in s else "—"

        print(f"  {name:<35} {sr:<10} {pt:<12} {pl:<10} {jk:<10} {od:<12}")

    print("-"*80)

    # Failure analysis
    print(f"\n  FAILURE BREAKDOWN:")
    print(f"  {'Phase':<35} {'Failures':<10} {'Types'}")
    print("-"*80)
    for name, s in summaries.items():
        fb = s.get("failure_breakdown", {})
        if fb:
            types_str = ", ".join(f"{k}:{v}" for k, v in fb.items())
            print(f"  {name:<35} {s['failures']:<10} {types_str}")
        elif s["failures"] > 0:
            print(f"  {name:<35} {s['failures']:<10} (no breakdown available)")

    # Collision check
    print(f"\n  COLLISION VALIDATION:")
    for name, s in summaries.items():
        cr = s.get("collision_rate_in_successes_pct", 0)
        if cr > 0:
            print(f"  WARNING: {name}: {cr:.1f}% of successful plans had collisions!")
        elif s["successes"] > 0:
            print(f"  {name}: No collisions detected in successful plans")

    # Constraint satisfaction
    print(f"\n  ORIENTATION CONSTRAINT SATISFACTION:")
    for name, s in summaries.items():
        cvr = s.get("constraint_violation_rate_pct")
        if cvr is not None:
            if cvr > 0:
                od = s.get("max_orientation_deviation_deg", {})
                print(f"  {name}: {cvr:.1f}% violated constraint "
                      f"(max deviation p98={od.get('p98', '?'):.2f}°)")
            else:
                print(f"  {name}: All trajectories within constraint")

    # Key insights
    print(f"\n  KEY INSIGHTS:")
    # Compare Phase 1 vs Phase 2
    p1 = summaries.get("phase1_baseline")
    p2 = summaries.get("phase2_constrained")
    if p1 and p2 and p1["successes"] > 0 and p2["successes"] > 0:
        sr_drop = p1["success_rate_pct"] - p2["success_rate_pct"]
        pt_increase = p2["plan_time_ms"]["median"] - p1["plan_time_ms"]["median"]
        print(f"  → Adding orientation constraints: "
              f"{sr_drop:+.1f}% success rate, "
              f"{pt_increase:+.1f}ms planning time")

    # Compare obstacle layouts
    for layout in ["easy", "medium", "hard"]:
        uc = summaries.get(f"phase3_{layout}_unconstrained")
        c = summaries.get(f"phase3_{layout}_constrained")
        if uc and c and uc["successes"] > 0 and c["successes"] > 0:
            sr_drop = uc["success_rate_pct"] - c["success_rate_pct"]
            print(f"  → {layout} obstacles + constraints vs without: "
                  f"{sr_drop:+.1f}% success rate change")

    print("\n" + "="*80)


def analyze_failures(results_dir: str):
    """Deep-dive into failures from per-problem data."""
    print(f"\n  DETAILED FAILURE ANALYSIS:")
    print("-"*60)

    for filepath in sorted(glob.glob(os.path.join(results_dir, "*_per_problem.json"))):
        name = os.path.basename(filepath).replace("_per_problem.json", "")
        with open(filepath, "r") as f:
            per_problem = json.load(f)

        failures = [p for p in per_problem if not p["success"]]
        if not failures:
            continue

        print(f"\n  {name}: {len(failures)} failures")

        # Categorize
        ik_fails = [f for f in failures if "IK" in f.get("status", "")]
        trajopt_fails = [f for f in failures if "TRAJOPT" in f.get("status", "")]
        other_fails = [f for f in failures
                       if "IK" not in f.get("status", "")
                       and "TRAJOPT" not in f.get("status", "")]

        if ik_fails:
            print(f"    IK failures: {len(ik_fails)} — "
                  "goal may be unreachable or in collision")
        if trajopt_fails:
            print(f"    TrajOpt failures: {len(trajopt_fails)} — "
                  "path exists but optimizer couldn't find it")
            print(f"    → Try increasing num_trajopt_seeds or max_attempts")
        if other_fails:
            statuses = set(f.get("status", "?") for f in other_fails)
            print(f"    Other: {len(other_fails)} — statuses: {statuses}")


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    results_dir = os.path.join(project_dir, "results")

    if not os.path.exists(results_dir):
        print("  No results directory found. Run Phase 0-3 first.")
        return

    summaries = load_summaries(results_dir)
    print_cross_phase_report(summaries)
    analyze_failures(results_dir)

    print(f"\n  All result files are in: {results_dir}/")
    print(f"  Summary JSONs can be loaded for custom analysis.\n")


if __name__ == "__main__":
    main()
