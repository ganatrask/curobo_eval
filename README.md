# cuRobo Evaluation Pipeline — Phases 0-3

## What this project does

This pipeline systematically tests cuRobo as an IK solver and motion planner for
your spatula-equipped food-transfer robot in Isaac Sim. It answers one question
at each phase:

- **Phase 0**: Is my robot configured correctly? (sphere coverage, FK/IK round-trip)
- **Phase 1**: Can cuRobo plan collision-free paths A→B? (no obstacles, no constraints)
- **Phase 2**: Can it keep the spatula level throughout the path? (orientation constraints)
- **Phase 3**: Can it avoid fixed obstacles while maintaining constraints?

## The "1000 problems" concept

A single motion-planning test proves nothing — the solver might get lucky or unlucky.
The standard approach (used in cuRobo's own benchmarks and MotionBenchMaker) is:

1. **Define your task workspace** — the 3D bounding box where your robot will actually
   operate (e.g., a kitchen countertop area).
2. **Sample N random (start_config, goal_pose) pairs** within that workspace.
3. **Run the planner on every pair**, log success/fail, timing, trajectory quality.
4. **Report statistics** — mean, median, 75th, 98th percentiles.

We use 1000 problems because it gives tight confidence intervals while remaining
computationally feasible (~30ms per plan × 1000 = ~30 seconds of pure planning).

## How workspace discovery works

You don't hardcode your workspace — you discover it from the robot itself:

1. **IK reachability scan**: Sample a dense 3D grid of poses around the robot base.
   For each pose, run batched IK. Mark reachable vs unreachable.
2. **Filter to task-relevant region**: From all reachable poses, keep only those
   within your actual task area (e.g., counter height, within arm's reach of food stations).
3. **This becomes your problem space**: Random sampling within this region guarantees
   every test problem is physically meaningful.

## Project structure

```
curobo_eval/
├── README.md
├── config/
│   └── eval_config.yaml          # Your robot, workspace, obstacle definitions
├── scripts/
│   ├── phase0_verify_robot.py    # Validate spheres, FK/IK, visualize
│   ├── phase1_baseline.py        # A→B planning benchmark
│   ├── phase2_constrained.py     # + orientation constraints
│   ├── phase3_obstacles.py       # + static obstacles
│   ├── analyze_results.py        # Generate reports from logged data
│   └── utils/
│       ├── problem_generator.py  # Workspace discovery + problem sampling
│       └── metrics.py            # Metric computation helpers
└── results/                      # JSON logs from each phase
```

## Setup

### Prerequisites
- Isaac Sim 4.5+ installed
- cuRobo v0.7.6+ installed in Isaac Sim's Python
- Your robot USD + sphere YAML already generated

### Install cuRobo in Isaac Sim
```bash
# Set alias for Isaac Sim's Python
alias omni_python='~/.local/share/ov/pkg/isaac-sim-4.5.0/python.sh'

# Install cuRobo
cd /path/to/curobo
omni_python -m pip install -e ".[isaacsim]" --no-build-isolation
```

### Configure for your robot
Edit `config/eval_config.yaml` — fill in your robot file path, joint names,
workspace bounds, and obstacle layouts. The template has detailed comments.

## Running

```bash
# Phase 0: Verify robot setup (launches Isaac Sim with visualization)
omni_python scripts/phase0_verify_robot.py

# Phase 1: Baseline benchmark (headless, fast)
omni_python scripts/phase1_baseline.py --num_problems 1000 --headless

# Phase 2: Add orientation constraints
omni_python scripts/phase2_constrained.py --num_problems 1000 --headless

# Phase 3: Add obstacles (runs all obstacle layouts)
omni_python scripts/phase3_obstacles.py --num_problems 1000 --headless

# Analyze all results
omni_python scripts/analyze_results.py
```

## Failure diagnosis (THE most important step)

After any phase completes, diagnose every failure to understand root causes:

```bash
# Diagnose all failures from a phase (headless — just the report)
omni_python scripts/failure_diagnosis.py --phase phase1_baseline --headless

# Diagnose + VISUALIZE each failure interactively in Isaac Sim
omni_python scripts/failure_diagnosis.py --phase phase3_hard_constrained --interactive

# Inspect ONE specific failed problem in detail
omni_python scripts/failure_diagnosis.py --phase phase3_hard_constrained --problem_id 42
```

The diagnosis script classifies each failure into one of two categories:

**INFEASIBLE** (nothing cuRobo can do — no valid path exists):
- `INFEASIBLE_START_COLLISION` — start config spheres overlap an obstacle
- `INFEASIBLE_GOAL_COLLISION` — goal is reachable but all IK solutions collide
- `INFEASIBLE_SELF_COLLISION` — start or goal in self-collision
- `INFEASIBLE_UNREACHABLE` — no IK solution exists for the goal pose

**SOLVER LIMITATION** (valid path exists but cuRobo couldn't find it):
- `SOLVER_NEEDS_MORE_SEEDS` — solved with 48 seeds instead of 12 (tuning fix)
- `SOLVER_NEEDS_GRAPH` — solved with PRM* graph planner (narrow passage)
- `SOLVER_TRAJOPT_FAIL` — genuine optimization failure even with escalation
- `SOLVER_IK_FAIL` — internal IK phase failed despite standalone IK succeeding
- `SOLVER_DT_EXCEPTION` — trajectory too long for time budget

The report tells you exactly what to fix:
- If most failures are `SOLVER_NEEDS_MORE_SEEDS` → increase `num_trajopt_seeds`
- If most are `SOLVER_NEEDS_GRAPH` → always set `enable_graph: true`
- If most are `INFEASIBLE_GOAL_COLLISION` → your obstacle layout is too restrictive
- If most are `SOLVER_TRAJOPT_FAIL` → genuine limitation, consider waypoints

## URDF to USD conversion (reference)

If you ever need to regenerate your USD from URDF, cuRobo provides:

```bash
omni_python curobo/examples/isaac_sim/utils/convert_urdf_to_usd.py \
    --robot /path/to/your_robot.yml --save_usd
```

This converts your URDF to USD format that Isaac Sim can load. After conversion,
open the USD in Isaac Sim and use **Isaac Utils → Lula Robot Description Editor**
to generate collision spheres. You already have your USD and spheres, so this
step is only needed if you modify the URDF (e.g., adding the spatula link).

## Key cuRobo conventions to remember

- **Quaternion format**: wxyz (NOT xyzw) — `[qw, qx, qy, qz]`
- **Pose format**: `[x, y, z, qw, qx, qy, qz]` (7 elements)
- **All poses**: Relative to robot base frame
- **Joint limits**: Must have non-zero velocity limits in URDF (silent NaN otherwise)
- **CUDA timing**: Always use `torch.cuda.synchronize()` before/after timing
- **Warmup**: First planning call includes JIT compilation — always discard it
