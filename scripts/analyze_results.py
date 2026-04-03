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
    print(f"\n  {'Phase':<35} {'Success%':<10} {'Valid':<8} {'PlanTime':<12} "
          f"{'PathLen':<10} {'OrientDev':<12}")
    print(f"  {'':35} {'':10} {'':8} {'median(ms)':<12} "
          f"{'mean(rad)':<10} {'mean(°)':<12}")
    print("-"*88)

    for name, s in summaries.items():
        sr = f"{s['success_rate_pct']:.1f}%"
        n_valid = s.get('valid_problems', s['total_problems'])
        valid_str = f"{n_valid}/{s['total_problems']}"

        if s["successes"] == 0:
            print(f"  {name:<35} {sr:<10} {valid_str:<8} {'—':<12} {'—':<10} {'—':<12}")
            continue

        pt = f"{s['plan_time_ms']['median']:.1f}" if 'plan_time_ms' in s else "—"
        pl = f"{s['path_length_rad']['mean']:.2f}" if 'path_length_rad' in s else "—"
        od = f"{s['max_orientation_deviation_deg']['mean']:.2f}" if 'max_orientation_deviation_deg' in s else "—"

        print(f"  {name:<35} {sr:<10} {valid_str:<8} {pt:<12} {pl:<10} {od:<12}")

    print("-"*88)

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
    print(f"\n  ORIENTATION CONSTRAINT ENFORCEMENT:")
    for name, s in summaries.items():
        n_orient_fail = s.get("orientation_constraint_failures", 0)
        if n_orient_fail > 0:
            planning_sr = s.get("planning_success_rate_pct", s["success_rate_pct"])
            final_sr = s["success_rate_pct"]
            od = s.get("max_orientation_deviation_deg", {})
            print(f"  {name}:")
            print(f"    Planning success: {planning_sr:.1f}% -> "
                  f"After constraint enforcement: {final_sr:.1f}%")
            print(f"    {n_orient_fail} plans rejected (violated {od.get('p98', '?'):.1f}° p98 "
                  f"vs 5° limit)")
        elif "constrained" in name:
            print(f"  {name}: All planned trajectories satisfy constraint")

    # Invalid start state warnings
    has_start_issues = False
    for name, s in summaries.items():
        n_skipped = s.get("skipped_invalid_start", 0)
        if n_skipped > 0:
            if not has_start_issues:
                print(f"\n  INVALID START STATE WARNINGS:")
                has_start_issues = True
            print(f"  {name}: {n_skipped} problems had start configs "
                  f"colliding with obstacles (excluded from success rate)")
    if has_start_issues:
        print(f"  → Re-generate problem sets with obstacle-aware workspace scan")

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
