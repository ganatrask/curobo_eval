"""
Metrics computation for cuRobo evaluation.
Computes trajectory quality, orientation constraint satisfaction, timing stats.
"""

import torch
import numpy as np
from typing import Dict, List, Optional
from dataclasses import dataclass, field, asdict


@dataclass
class PlanningMetrics:
    """Metrics for a single planning problem."""
    problem_id: int = -1
    success: bool = False
    status: str = ""
    plan_time_ms: float = 0.0
    
    # Trajectory quality (only if success)
    path_length_rad: float = 0.0
    motion_time_s: float = 0.0
    num_waypoints: int = 0
    max_jerk: float = 0.0
    max_acceleration: float = 0.0
    
    # Pose accuracy at goal
    position_error_mm: float = 0.0
    orientation_error_deg: float = 0.0
    
    # Orientation constraint (Phase 2+)
    max_orientation_deviation_deg: float = 0.0
    mean_orientation_deviation_deg: float = 0.0
    constraint_satisfied: bool = True
    
    # Collision validation
    any_collision_detected: bool = False
    min_obstacle_clearance_mm: float = 0.0

    def to_dict(self) -> Dict:
        return asdict(self)


def compute_path_length(trajectory: torch.Tensor) -> float:
    """Compute total path length in joint space (radians)."""
    if trajectory.shape[0] < 2:
        return 0.0
    diffs = torch.diff(trajectory, dim=0)
    step_lengths = torch.norm(diffs, dim=1)
    return step_lengths.sum().item()


def compute_max_jerk(trajectory: torch.Tensor, dt: float) -> float:
    """Compute maximum jerk across all joints and timesteps."""
    if trajectory.shape[0] < 4:
        return 0.0
    vel = torch.diff(trajectory, dim=0) / dt
    acc = torch.diff(vel, dim=0) / dt
    jerk = torch.diff(acc, dim=0) / dt
    return torch.max(torch.abs(jerk)).item()


def compute_max_acceleration(trajectory: torch.Tensor, dt: float) -> float:
    """Compute maximum acceleration across all joints and timesteps."""
    if trajectory.shape[0] < 3:
        return 0.0
    vel = torch.diff(trajectory, dim=0) / dt
    acc = torch.diff(vel, dim=0) / dt
    return torch.max(torch.abs(acc)).item()


def quaternion_angular_distance(q1: torch.Tensor, q2: torch.Tensor) -> float:
    """
    Compute angular distance between two quaternions in degrees.
    Both in wxyz format.
    """
    # Normalize
    q1 = q1 / torch.norm(q1)
    q2 = q2 / torch.norm(q2)
    
    # Dot product (clamp for numerical stability)
    dot = torch.clamp(torch.abs(torch.sum(q1 * q2)), 0.0, 1.0)
    
    # Angular distance in degrees
    angle_rad = 2.0 * torch.acos(dot)
    return torch.rad2deg(angle_rad).item()


def compute_orientation_deviation(
    ee_quaternions: torch.Tensor,
    target_quaternion_wxyz: List[float],
) -> Dict[str, float]:
    """
    Compute orientation deviation at every waypoint from the target orientation.
    
    Args:
        ee_quaternions: (N, 4) tensor of EE quaternions at each waypoint [wxyz]
        target_quaternion_wxyz: Desired orientation [qw, qx, qy, qz]
    
    Returns:
        Dict with max_deviation_deg, mean_deviation_deg
    """
    target = torch.tensor(
        target_quaternion_wxyz, dtype=torch.float32, device=ee_quaternions.device
    )
    
    deviations = []
    for i in range(ee_quaternions.shape[0]):
        dev = quaternion_angular_distance(ee_quaternions[i], target)
        deviations.append(dev)
    
    deviations = np.array(deviations)
    return {
        "max_deviation_deg": float(np.max(deviations)),
        "mean_deviation_deg": float(np.mean(deviations)),
        "all_deviations_deg": deviations.tolist(),
    }


def aggregate_metrics(all_metrics: List[PlanningMetrics]) -> Dict:
    """
    Compute aggregate statistics from a list of per-problem metrics.
    Reports mean, median, p75, p98 for all numeric fields.
    """
    n_total = len(all_metrics)
    n_success = sum(1 for m in all_metrics if m.success)
    n_fail = n_total - n_success
    
    result = {
        "total_problems": n_total,
        "successes": n_success,
        "failures": n_fail,
        "success_rate_pct": 100 * n_success / n_total if n_total > 0 else 0,
    }
    
    # Failure breakdown
    fail_statuses = [m.status for m in all_metrics if not m.success]
    status_counts = {}
    for s in fail_statuses:
        status_counts[s] = status_counts.get(s, 0) + 1
    result["failure_breakdown"] = status_counts
    
    # Aggregate numeric metrics for successful plans only
    if n_success == 0:
        return result
    
    success_metrics = [m for m in all_metrics if m.success]
    
    numeric_fields = [
        "plan_time_ms", "path_length_rad", "motion_time_s",
        "max_jerk", "max_acceleration",
        "position_error_mm", "orientation_error_deg",
        "max_orientation_deviation_deg", "mean_orientation_deviation_deg",
    ]
    
    for field_name in numeric_fields:
        values = [getattr(m, field_name) for m in success_metrics]
        arr = np.array(values)
        result[field_name] = {
            "mean": float(np.mean(arr)),
            "median": float(np.median(arr)),
            "std": float(np.std(arr)),
            "p75": float(np.percentile(arr, 75)),
            "p98": float(np.percentile(arr, 98)),
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
        }
    
    # Collision stats
    n_collision = sum(1 for m in success_metrics if m.any_collision_detected)
    result["collision_rate_in_successes_pct"] = 100 * n_collision / n_success
    
    # Constraint satisfaction (Phase 2+)
    n_constraint_fail = sum(
        1 for m in success_metrics if not m.constraint_satisfied
    )
    result["constraint_violation_rate_pct"] = 100 * n_constraint_fail / n_success
    
    return result


def print_summary(agg: Dict, phase_name: str = ""):
    """Pretty-print aggregate metrics."""
    print(f"\n{'='*60}")
    print(f"  {phase_name} RESULTS SUMMARY")
    print(f"{'='*60}")
    print(f"  Problems:     {agg['total_problems']}")
    print(f"  Success rate: {agg['success_rate_pct']:.1f}% "
          f"({agg['successes']}/{agg['total_problems']})")
    
    if agg.get("failure_breakdown"):
        print(f"  Failure types:")
        for status, count in agg["failure_breakdown"].items():
            print(f"    {status}: {count}")
    
    if agg["successes"] == 0:
        print("  No successful plans — cannot compute quality metrics.")
        return
    
    print(f"\n  Planning time (ms):")
    pt = agg["plan_time_ms"]
    print(f"    mean={pt['mean']:.1f}  median={pt['median']:.1f}  "
          f"p98={pt['p98']:.1f}")
    
    print(f"\n  Trajectory quality:")
    print(f"    Path length:   {agg['path_length_rad']['mean']:.2f} rad (mean)")
    print(f"    Motion time:   {agg['motion_time_s']['mean']:.2f} s (mean)")
    print(f"    Max jerk:      {agg['max_jerk']['p98']:.1f} rad/s³ (p98)")
    print(f"    Position err:  {agg['position_error_mm']['mean']:.3f} mm (mean)")
    
    if "max_orientation_deviation_deg" in agg:
        od = agg["max_orientation_deviation_deg"]
        print(f"\n  Orientation constraint:")
        print(f"    Max deviation: {od['mean']:.2f}° (mean), "
              f"{od['p98']:.2f}° (p98)")
        print(f"    Violations:    {agg['constraint_violation_rate_pct']:.1f}%")
    
    print(f"\n  Collision check:")
    print(f"    Collision rate: {agg['collision_rate_in_successes_pct']:.1f}% "
          f"of successful plans")
    print(f"{'='*60}\n")
