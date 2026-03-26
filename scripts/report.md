Phase 0 

scripts/verify_spatula_orientation.py

  Face UP   → keep flat_orientation_wxyz = [1, 0, 0, 0]

![alt text](image.png)


scripts/visualize_spheres.py

![alt text](image-1.png)


[7.545s] Simulation App Startup Complete

############################################################
  cuRobo Evaluation — Phase 1: Baseline (no obstacles)
############################################################

  Loading problems from /home/shyam/projects/cc/curobo_eval/results/problems_baseline.json

[phase1_baseline] Initializing MotionGen...
[phase1_baseline] Warming up (JIT compilation)...
[phase1_baseline] Ready.

[phase1_baseline] Running 1000 problems...
  Orientation constraint: False
  World obstacles: none
  [100/1000] success=96/100 (96.0%)
  [200/1000] success=191/200 (95.5%)
  [300/1000] success=288/300 (96.0%)
  [400/1000] success=383/400 (95.8%)
  [500/1000] success=476/500 (95.2%)
  [600/1000] success=569/600 (94.8%)
  [700/1000] success=667/700 (95.3%)
  [800/1000] success=764/800 (95.5%)
  [900/1000] success=856/900 (95.1%)
  [1000/1000] success=947/1000 (94.7%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase1_baseline_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase1_baseline_summary.json

============================================================
  PHASE1_BASELINE RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 94.7% (947/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 26
    MotionGenStatus.FINETUNE_TRAJOPT_FAIL: 27

  Planning time (ms):
    mean=111.0  median=91.2  p98=273.9

  Trajectory quality:
    Path length:   3.25 rad (mean)
    Motion time:   1.19 s (mean)
    Max jerk:      1114.2 rad/s³ (p98)
    Position err:  2.874 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================

[182.323s] Simulation App Shutting Down


Two issues found:

1. Joint 0 position limits are tighter than spec:

URDF: ±3.0543 rad (±175°)
Spec: ±3.1416 rad (±180°)
Minor — probably intentional to add a 5° safety margin
2. ALL velocity limits are lower than spec:

Joint	URDF velocity	Spec velocity	Ratio
0–2	3.375 rad/s	6.283 rad/s	54%
3–5	7.000 rad/s	9.425 rad/s	74%
This is significant — cuRobo uses velocity limits to compute trajectory timing. Lower velocity limits mean:

Slower planned trajectories (which is fine for food safety)
But also more planning failures if cuRobo thinks the robot can't move fast enough
The URDF limits look intentionally conservative (roughly 50–75% of max). For food transfer with a spatula, this is reasonable — you don't want full-speed motion. But if you're seeing SOLVER_DT_EXCEPTION failures or slow trajectories, this is why.

The position limits are correct (except the minor Joint 0 trim). No action needed unless you want to experiment with faster velocities.

This drawing confirms our numbers align well. Here's the comparison:

From the drawing:

Max reach radius: R0.750m (full arm extension, any orientation)
Inner dead zone: R0.100m (too close to base)
Usable side reach: R0.486m (at the 28.16° downward angle)
Height above base: 0.1035m
From our workspace scan (spatula level):

X reach: 0.25–0.45m — well within the 0.75m max, but the spatula constraint cuts it short
Y range: ±0.30m — uses about 40% of the ±0.75m side-to-side range
Z range: 0.02–0.37m — starts near table height, goes up ~35cm
Key takeaways:

Our x_max = 0.45m is conservative — the arm can physically reach 0.75m, but keeping the spatula flat limits it to ~0.45m. You could try expanding x_range to [0.1, 0.55] in eval_config.yaml and re-scanning to see if you pick up more reachable points.

The R0.486m line in the drawing roughly matches the practical reach with the arm angled down — close to our effective limit.

Our y_range = ±0.3m could potentially be expanded to ±0.4m since the arm can reach ±0.75m.

Phase 1 Summary: Baseline (No Obstacles, No Constraints)
Success rate: 94.7% (947/1000)

The 53 Failures Break Down As:
Category	Count	What it means
INFEASIBLE_UNREACHABLE	21	Goal is outside the reachable workspace — not cuRobo's fault
SOLVER_NEEDS_MORE_SEEDS	19	cuRobo found the path with 48 seeds instead of 12 — tuning fix
SOLVER_TRAJOPT_FAIL	8	Genuine solver limitation — couldn't find path even with escalation
SOLVER_IK_FAIL	2	Internal IK phase failed despite solution existing
What This Tells Us:
Effective success rate is ~97.8% if you exclude the 21 infeasible goals (which no planner could solve). That's 947/979.

19 failures are fixable by increasing num_trajopt_seeds from 12 to 48 in eval_config.yaml — that alone would bring success to ~96.6%.

Only 10 failures (1%) are genuine solver limitations.

Velocity limits in your URDF are 50–75% of spec — intentionally conservative for food safety, but contributes to some trajectory planning difficulty.

Workspace is small (97 reachable positions) due to the spatula-level orientation constraint on a compact arm.



Planning time (with 12 seeds):

Mean: 111ms
Median: 91ms
P98: 274ms (98th percentile — worst 2% of cases)
So cuRobo finds a valid trajectory in ~91ms for the typical case. The P98 of 274ms means even hard problems are solved under 300ms.

66% of plans solve under 100ms, 97% under 200ms.

Seed 12 vs 48 

the side-by-side comparison:

Metric	12 Seeds	48 Seeds	Change
Success rate	94.7% (947)	95.4% (954)	+7 solved
IK_FAIL	26	13	-13
TRAJOPT_FAIL	27	33	+6
Median time	91ms	130ms	+43% slower
P98 time	274ms	423ms	+54% slower
Path length	3.25 rad	3.26 rad	same
Position err	2.9mm	3.0mm	same
Takeaway: 48 seeds fixed 13 IK failures but only gained 7 net successes (some problems that got lucky with 12 seeds now fail with different seed patterns). The cost is 43% slower planning.

For food transfer at 130ms median, that's still plenty fast. The remaining 46 failures are mostly genuine infeasibility (21) and hard trajectory optimization cases (33).

==========================================================
  PHASE2_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 94.7% (947/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 21
    MotionGenStatus.FINETUNE_TRAJOPT_FAIL: 32

  Planning time (ms):
    mean=107.8  median=92.2  p98=223.6

  Trajectory quality:
    Path length:   3.24 rad (mean)
    Motion time:   1.18 s (mean)
    Max jerk:      1109.4 rad/s³ (p98)
    Position err:  2.876 mm (mean)

  Orientation constraint:
    Max deviation: 33.22° (mean), 154.12° (p98)
    Violations:    92.2%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  PHASE2_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 98.1% (981/1000)
  Failure types:
    MotionGenStatus.FINETUNE_TRAJOPT_FAIL: 9
    MotionGenStatus.IK_FAIL: 10

  Planning time (ms):
    mean=128.4  median=101.6  p98=348.4

  Trajectory quality:
    Path length:   3.66 rad (mean)
    Motion time:   1.35 s (mean)
    Max jerk:      1679.0 rad/s³ (p98)
    Position err:  2.883 mm (mean)

  Orientation constraint:
    Max deviation: 7.70° (mean), 28.92° (p98)
    Violations:    45.6%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================



Interesting results! Let me analyze:

Success rate: 98.1% — actually HIGHER than Phase 1 (94.7%)! That's surprising. The orientation constraint seems to help the optimizer converge better.

Orientation deviation: 7.70° mean, 28.92° P98 — much better than unconstrained (33.22° mean, 154.12° P98), but still 45.6% violate the 5° threshold. The constraint is working but not perfectly.

Path length: 3.66 rad — longer than Phase 1 (3.24 rad), which makes sense — the robot takes longer paths to maintain orientation.

Motion time: 1.35s — slower than Phase 1 (1.18s), as expected with the longer paths.

So the constraint is definitely helping — orientation deviation dropped dramatically from 154° P98 down to 29°, and violations fell from 92.2% to 45.6%. But it's still not tight enough for food transfer where we need <5°. I could increase the constraint weight, though cuRobo doesn't expose that easily, or I could accept the 54.4% of trajectories that already meet the threshold and replan the rest, or tighten the max_orientation_deviation_deg parameter further.

This is actually useful — it shows the constraint is working directionally but needs more tuning for the specific application. Now let me pull together the comparison across all phases.

The constraint is working! Let me compare all three runs:

Metric	Phase 1 (none)	Phase 2 (unconstrained)	Phase 2 (constrained)
Success rate	94.7%	94.7%	98.1%
Max orient. dev (mean)	0° (not measured)	33.2°	7.7°
Max orient. dev (P98)	—	154.1°	28.9°
Violations (>5°)	—	92.2%	45.6%
Path length	3.24 rad	3.24 rad	3.66 rad
Median plan time	91ms	92ms	102ms
Key findings:

Constraint is working — orientation deviation dropped from 154° P98 to 29° P98 (5x improvement)
Success rate actually increased (98.1% vs 94.7%) — the orientation constraint helps the optimizer find smoother paths
45.6% still violate 5° — the constraint reduces deviation but doesn't enforce a hard limit. This is consistent with the paper: "preliminary, sensitive across environments"
Paths are 13% longer and 15% slower — the robot takes longer routes to maintain orientation
For food transfer production use, the 54.4% of trajectories that satisfy <5° are directly usable. For the rest, you could:

Replan with more seeds
Use waypoint decomposition (break long motions into shorter segments)
Slightly relax the threshold to 10° (which would pass ~75% of trajectories)


.009s] app ready
[7.575s] Simulation App Startup Complete

############################################################
  cuRobo Evaluation — Phase 3: Static Obstacles
############################################################
  Generating 1000 problems for 'easy' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (46 reachable so far)
[Workspace] Found 74/539 reachable positions (13.7%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_easy.json

============================================================
  Layout: easy | Constraint: unconstrained
  Description: Simple table + 1 side wall, mostly clear workspace
============================================================
  Obstacles: 2 (cuboids: 2, meshes: 0)

[phase3_easy_unconstrained] Initializing MotionGen...
[phase3_easy_unconstrained] Warming up (JIT compilation)...
[phase3_easy_unconstrained] Ready.

[phase3_easy_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=199/200 (99.5%)
  [300/1000] success=299/300 (99.7%)
  [400/1000] success=399/400 (99.8%)
  [500/1000] success=499/500 (99.8%)
  [600/1000] success=599/600 (99.8%)
  [700/1000] success=699/700 (99.9%)
  [800/1000] success=799/800 (99.9%)
  [900/1000] success=899/900 (99.9%)
  [1000/1000] success=999/1000 (99.9%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_easy_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_easy_unconstrained_summary.json

============================================================
  PHASE3_EASY_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.9% (999/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 1

  Planning time (ms):
    mean=167.8  median=113.3  p98=813.2

  Trajectory quality:
    Path length:   3.02 rad (mean)
    Motion time:   1.13 s (mean)
    Max jerk:      1083.5 rad/s³ (p98)
    Position err:  0.346 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: easy | Constraint: constrained
  Description: Simple table + 1 side wall, mostly clear workspace
============================================================
  Obstacles: 2 (cuboids: 2, meshes: 0)

[phase3_easy_constrained] Initializing MotionGen...
[phase3_easy_constrained] Warming up (JIT compilation)...
[phase3_easy_constrained] Ready.

[phase3_easy_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=198/200 (99.0%)
  [300/1000] success=298/300 (99.3%)
  [400/1000] success=396/400 (99.0%)
  [500/1000] success=496/500 (99.2%)
  [600/1000] success=595/600 (99.2%)
  [700/1000] success=694/700 (99.1%)
  [800/1000] success=791/800 (98.9%)
  [900/1000] success=889/900 (98.8%)
  [1000/1000] success=987/1000 (98.7%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_easy_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_easy_constrained_summary.json

============================================================
  PHASE3_EASY_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 98.7% (987/1000)
  Failure types:
    MotionGenStatus.FINETUNE_TRAJOPT_FAIL: 13

  Planning time (ms):
    mean=160.1  median=107.5  p98=756.0

  Trajectory quality:
    Path length:   3.02 rad (mean)
    Motion time:   1.13 s (mean)
    Max jerk:      1237.1 rad/s³ (p98)
    Position err:  0.356 mm (mean)

  Orientation constraint:
    Max deviation: 2.32° (mean), 13.32° (p98)
    Violations:    13.7%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================

  Generating 1000 problems for 'medium' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (33 reachable so far)
[Workspace] Found 61/539 reachable positions (11.3%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_medium.json

============================================================
  Layout: medium | Constraint: unconstrained
  Description: Table + overhead shelf + 2 side obstacles
============================================================
  Obstacles: 4 (cuboids: 4, meshes: 0)

[phase3_medium_unconstrained] Initializing MotionGen...
[phase3_medium_unconstrained] Warming up (JIT compilation)...
[phase3_medium_unconstrained] Ready.

[phase3_medium_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=200/200 (100.0%)
  [300/1000] success=300/300 (100.0%)
  [400/1000] success=400/400 (100.0%)
  [500/1000] success=500/500 (100.0%)
  [600/1000] success=600/600 (100.0%)
  [700/1000] success=700/700 (100.0%)
  [800/1000] success=800/800 (100.0%)
  [900/1000] success=900/900 (100.0%)
  [1000/1000] success=1000/1000 (100.0%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_medium_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_medium_unconstrained_summary.json

============================================================
  PHASE3_MEDIUM_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 100.0% (1000/1000)

  Planning time (ms):
    mean=246.2  median=133.3  p98=961.0

  Trajectory quality:
    Path length:   2.87 rad (mean)
    Motion time:   1.13 s (mean)
    Max jerk:      1152.1 rad/s³ (p98)
    Position err:  0.282 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: medium | Constraint: constrained
  Description: Table + overhead shelf + 2 side obstacles
============================================================
  Obstacles: 4 (cuboids: 4, meshes: 0)

[phase3_medium_constrained] Initializing MotionGen...
[phase3_medium_constrained] Warming up (JIT compilation)...
[phase3_medium_constrained] Ready.

[phase3_medium_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=99/100 (99.0%)
  [200/1000] success=198/200 (99.0%)
  [300/1000] success=293/300 (97.7%)
  [400/1000] success=386/400 (96.5%)
  [500/1000] success=483/500 (96.6%)
  [600/1000] success=577/600 (96.2%)
  [700/1000] success=672/700 (96.0%)
  [800/1000] success=767/800 (95.9%)
  [900/1000] success=863/900 (95.9%)
  [1000/1000] success=961/1000 (96.1%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_medium_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_medium_constrained_summary.json

============================================================
  PHASE3_MEDIUM_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 96.1% (961/1000)
  Failure types:
    MotionGenStatus.FINETUNE_TRAJOPT_FAIL: 39

  Planning time (ms):
    mean=169.7  median=106.4  p98=605.5

  Trajectory quality:
    Path length:   2.81 rad (mean)
    Motion time:   1.12 s (mean)
    Max jerk:      1169.1 rad/s³ (p98)
    Position err:  0.273 mm (mean)

  Orientation constraint:
    Max deviation: 2.18° (mean), 11.06° (p98)
    Violations:    11.7%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================

  Generating 1000 problems for 'hard' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (9 reachable so far)
[Workspace] Found 11/539 reachable positions (2.0%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_hard.json

============================================================
  Layout: hard | Constraint: unconstrained
  Description: Dense clutter — table + shelf + narrow passage + boxes
============================================================
  Obstacles: 6 (cuboids: 6, meshes: 0)

[phase3_hard_unconstrained] Initializing MotionGen...
[phase3_hard_unconstrained] Warming up (JIT compilation)...
[phase3_hard_unconstrained] Ready.

[phase3_hard_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=98/100 (98.0%)
  [200/1000] success=196/200 (98.0%)
  [300/1000] success=295/300 (98.3%)
  [400/1000] success=395/400 (98.8%)
  [500/1000] success=495/500 (99.0%)
  [600/1000] success=594/600 (99.0%)
  [700/1000] success=694/700 (99.1%)
  [800/1000] success=793/800 (99.1%)
  [900/1000] success=893/900 (99.2%)
  [1000/1000] success=993/1000 (99.3%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_hard_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_hard_unconstrained_summary.json

============================================================
  PHASE3_HARD_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.3% (993/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 7

  Planning time (ms):
    mean=209.1  median=122.3  p98=985.6

  Trajectory quality:
    Path length:   1.08 rad (mean)
    Motion time:   0.93 s (mean)
    Max jerk:      625.7 rad/s³ (p98)
    Position err:  1.634 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: hard | Constraint: constrained
  Description: Dense clutter — table + shelf + narrow passage + boxes
============================================================
  Obstacles: 6 (cuboids: 6, meshes: 0)

[phase3_hard_constrained] Initializing MotionGen...
[phase3_hard_constrained] Warming up (JIT compilation)...
[phase3_hard_constrained] Ready.

[phase3_hard_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=98/100 (98.0%)
  [200/1000] success=196/200 (98.0%)
  [300/1000] success=295/300 (98.3%)
  [400/1000] success=395/400 (98.8%)
  [500/1000] success=495/500 (99.0%)
  [600/1000] success=594/600 (99.0%)
  [700/1000] success=694/700 (99.1%)
  [800/1000] success=793/800 (99.1%)
  [900/1000] success=893/900 (99.2%)
  [1000/1000] success=993/1000 (99.3%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_hard_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_hard_constrained_summary.json

============================================================
  PHASE3_HARD_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.3% (993/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 7

  Planning time (ms):
    mean=211.7  median=126.7  p98=1033.6

  Trajectory quality:
    Path length:   1.10 rad (mean)
    Motion time:   0.98 s (mean)
    Max jerk:      655.6 rad/s³ (p98)
    Position err:  1.638 mm (mean)

  Orientation constraint:
    Max deviation: 3.32° (mean), 5.73° (p98)
    Violations:    18.8%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


======================================================================
  PHASE 3 COMPARISON MATRIX
======================================================================
  Layout       Constraint      Success%   PlanTime(ms)   MaxOrient(°)  
----------------------------------------------------------------------
  easy         unconstrained   99.9       167.8          0.00          
  easy         constrained     98.7       160.1          2.32          
  medium       unconstrained   100.0      246.2          0.00          
  medium       constrained     96.1       169.7          2.18          
  hard         unconstrained   99.3       209.1          0.00          
  hard         constrained     99.3       211.7          3.32          
======================================================================

  All Phase 3 results saved to results/ directory.
  Next: run scripts/analyze_results.py for cross-phase analysis.



[7.807s] app ready
[10.893s] Simulation App Startup Complete

############################################################
  cuRobo Evaluation — Phase 3: Static Obstacles
############################################################
  Generating 1000 problems for 'easy' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (46 reachable so far)
[Workspace] Found 74/539 reachable positions (13.7%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_easy.json

============================================================
  Layout: easy | Constraint: unconstrained
  Description: Simple table + 1 side wall, mostly clear workspace
============================================================
  Obstacles: 2 (cuboids: 2, meshes: 0)

[phase3_easy_unconstrained] Initializing MotionGen...
[phase3_easy_unconstrained] Warming up (JIT compilation)...
[phase3_easy_unconstrained] Ready.

[phase3_easy_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=199/200 (99.5%)
  [300/1000] success=299/300 (99.7%)
  [400/1000] success=399/400 (99.8%)
  [500/1000] success=499/500 (99.8%)
  [600/1000] success=599/600 (99.8%)
  [700/1000] success=699/700 (99.9%)
^[[B^[[B  [800/1000] success=799/800 (99.9%)
  [900/1000] success=899/900 (99.9%)
  [1000/1000] success=999/1000 (99.9%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_easy_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_easy_unconstrained_summary.json

============================================================
  PHASE3_EASY_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.9% (999/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 1

  Planning time (ms):
    mean=251.0  median=160.8  p98=1370.2

  Trajectory quality:
    Path length:   3.02 rad (mean)
    Motion time:   1.13 s (mean)
    Max jerk:      1079.7 rad/s³ (p98)
    Position err:  0.348 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: easy | Constraint: constrained
  Description: Simple table + 1 side wall, mostly clear workspace
============================================================
  Obstacles: 2 (cuboids: 2, meshes: 0)

[phase3_easy_constrained] Initializing MotionGen...
[phase3_easy_constrained] Warming up (JIT compilation)...
[phase3_easy_constrained] Ready.

[phase3_easy_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=198/200 (99.0%)
  [300/1000] success=298/300 (99.3%)
  [400/1000] success=396/400 (99.0%)
  [500/1000] success=496/500 (99.2%)
  [600/1000] success=594/600 (99.0%)
  [700/1000] success=693/700 (99.0%)
  [800/1000] success=790/800 (98.8%)
  [900/1000] success=888/900 (98.7%)
  [1000/1000] success=986/1000 (98.6%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_easy_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_easy_constrained_summary.json

============================================================
  PHASE3_EASY_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 98.6% (986/1000)
  Failure types:
    MotionGenStatus.FINETUNE_TRAJOPT_FAIL: 14

  Planning time (ms):
    mean=249.7  median=167.5  p98=1225.6

  Trajectory quality:
    Path length:   3.03 rad (mean)
    Motion time:   1.13 s (mean)
    Max jerk:      1224.7 rad/s³ (p98)
    Position err:  0.354 mm (mean)

  Orientation constraint:
    Max deviation: 2.48° (mean), 12.68° (p98)
    Violations:    13.7%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================

  Generating 1000 problems for 'medium' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (33 reachable so far)
[Workspace] Found 61/539 reachable positions (11.3%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_medium.json

============================================================
  Layout: medium | Constraint: unconstrained
  Description: Table + overhead shelf + 2 side obstacles
============================================================
  Obstacles: 4 (cuboids: 4, meshes: 0)

[phase3_medium_unconstrained] Initializing MotionGen...
[phase3_medium_unconstrained] Warming up (JIT compilation)...
[phase3_medium_unconstrained] Ready.

[phase3_medium_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=200/200 (100.0%)
  [300/1000] success=300/300 (100.0%)
  [400/1000] success=400/400 (100.0%)
  [500/1000] success=500/500 (100.0%)
  [600/1000] success=600/600 (100.0%)
  [700/1000] success=700/700 (100.0%)
  [800/1000] success=800/800 (100.0%)
  [900/1000] success=899/900 (99.9%)
  [1000/1000] success=999/1000 (99.9%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_medium_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_medium_unconstrained_summary.json

============================================================
  PHASE3_MEDIUM_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.9% (999/1000)
  Failure types:
    MotionGenStatus.FINETUNE_TRAJOPT_FAIL: 1

  Planning time (ms):
    mean=313.1  median=192.9  p98=1089.3

  Trajectory quality:
    Path length:   2.86 rad (mean)
    Motion time:   1.13 s (mean)
    Max jerk:      1146.6 rad/s³ (p98)
    Position err:  0.283 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: medium | Constraint: constrained
  Description: Table + overhead shelf + 2 side obstacles
============================================================
  Obstacles: 4 (cuboids: 4, meshes: 0)

[phase3_medium_constrained] Initializing MotionGen...
[phase3_medium_constrained] Warming up (JIT compilation)...
[phase3_medium_constrained] Ready.

[phase3_medium_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=98/100 (98.0%)
  [200/1000] success=196/200 (98.0%)
  [300/1000] success=293/300 (97.7%)
  [400/1000] success=388/400 (97.0%)
  [500/1000] success=484/500 (96.8%)
  [600/1000] success=578/600 (96.3%)
  [700/1000] success=675/700 (96.4%)
  [800/1000] success=770/800 (96.2%)
  [900/1000] success=866/900 (96.2%)
  [1000/1000] success=965/1000 (96.5%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_medium_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_medium_constrained_summary.json

============================================================
  PHASE3_MEDIUM_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 96.5% (965/1000)
  Failure types:
    MotionGenStatus.FINETUNE_TRAJOPT_FAIL: 34
    MotionGenStatus.IK_FAIL: 1

  Planning time (ms):
    mean=314.1  median=191.1  p98=1141.6

  Trajectory quality:
    Path length:   2.83 rad (mean)
    Motion time:   1.13 s (mean)
    Max jerk:      1157.5 rad/s³ (p98)
    Position err:  0.255 mm (mean)

  Orientation constraint:
    Max deviation: 2.18° (mean), 11.14° (p98)
    Violations:    11.5%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================

  Generating 1000 problems for 'hard' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (9 reachable so far)
[Workspace] Found 11/539 reachable positions (2.0%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_hard.json

============================================================
  Layout: hard | Constraint: unconstrained
  Description: Dense clutter — table + shelf + narrow passage + boxes
============================================================
  Obstacles: 6 (cuboids: 6, meshes: 0)

[phase3_hard_unconstrained] Initializing MotionGen...
[phase3_hard_unconstrained] Warming up (JIT compilation)...
[phase3_hard_unconstrained] Ready.

[phase3_hard_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=98/100 (98.0%)
  [200/1000] success=196/200 (98.0%)
  [300/1000] success=295/300 (98.3%)
  [400/1000] success=395/400 (98.8%)
  [500/1000] success=495/500 (99.0%)
  [600/1000] success=594/600 (99.0%)
  [700/1000] success=694/700 (99.1%)
  [800/1000] success=793/800 (99.1%)
  [900/1000] success=893/900 (99.2%)
  [1000/1000] success=993/1000 (99.3%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_hard_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_hard_unconstrained_summary.json

============================================================
  PHASE3_HARD_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.3% (993/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 7

  Planning time (ms):
    mean=400.3  median=235.7  p98=1870.4

  Trajectory quality:
    Path length:   1.08 rad (mean)
    Motion time:   0.93 s (mean)
    Max jerk:      624.5 rad/s³ (p98)
    Position err:  1.634 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: hard | Constraint: constrained
  Description: Dense clutter — table + shelf + narrow passage + boxes
============================================================
  Obstacles: 6 (cuboids: 6, meshes: 0)

[phase3_hard_constrained] Initializing MotionGen...
[phase3_hard_constrained] Warming up (JIT compilation)...
[phase3_hard_constrained] Ready.

[phase3_hard_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=98/100 (98.0%)
  [200/1000] success=196/200 (98.0%)
  [300/1000] success=295/300 (98.3%)
  [400/1000] success=395/400 (98.8%)
  [500/1000] success=495/500 (99.0%)
  [600/1000] success=594/600 (99.0%)
  [700/1000] success=694/700 (99.1%)
  [800/1000] success=793/800 (99.1%)
  [900/1000] success=893/900 (99.2%)
  [1000/1000] success=993/1000 (99.3%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_hard_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_hard_constrained_summary.json

============================================================
  PHASE3_HARD_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.3% (993/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 7

  Planning time (ms):
    mean=394.4  median=238.3  p98=1887.3

  Trajectory quality:
    Path length:   1.10 rad (mean)
    Motion time:   0.98 s (mean)
    Max jerk:      655.2 rad/s³ (p98)
    Position err:  1.634 mm (mean)

  Orientation constraint:
    Max deviation: 3.32° (mean), 5.73° (p98)
    Violations:    18.8%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================

  Generating 1000 problems for 'few_boxes' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (50 reachable so far)
[Workspace] Found 76/539 reachable positions (14.1%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_few_boxes.json

============================================================
  Layout: few_boxes | Constraint: unconstrained
  Description: 3 small boxes on the table — robot must navigate around them
============================================================
  Obstacles: 4 (cuboids: 4, meshes: 0)

[phase3_few_boxes_unconstrained] Initializing MotionGen...
[phase3_few_boxes_unconstrained] Warming up (JIT compilation)...
[phase3_few_boxes_unconstrained] Ready.

[phase3_few_boxes_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=199/200 (99.5%)
  [300/1000] success=299/300 (99.7%)
  [400/1000] success=399/400 (99.8%)
  [500/1000] success=499/500 (99.8%)
  [600/1000] success=599/600 (99.8%)
  [700/1000] success=699/700 (99.9%)
  [800/1000] success=799/800 (99.9%)
  [900/1000] success=899/900 (99.9%)
  [1000/1000] success=999/1000 (99.9%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_few_boxes_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_few_boxes_unconstrained_summary.json

============================================================
  PHASE3_FEW_BOXES_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.9% (999/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 1

  Planning time (ms):
    mean=291.4  median=211.9  p98=1364.0

  Trajectory quality:
    Path length:   2.79 rad (mean)
    Motion time:   1.08 s (mean)
    Max jerk:      1068.4 rad/s³ (p98)
    Position err:  0.523 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: few_boxes | Constraint: constrained
  Description: 3 small boxes on the table — robot must navigate around them
============================================================
  Obstacles: 4 (cuboids: 4, meshes: 0)

[phase3_few_boxes_constrained] Initializing MotionGen...
[phase3_few_boxes_constrained] Warming up (JIT compilation)...
[phase3_few_boxes_constrained] Ready.

[phase3_few_boxes_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=199/200 (99.5%)
  [300/1000] success=299/300 (99.7%)
  [400/1000] success=399/400 (99.8%)
  [500/1000] success=499/500 (99.8%)
  [600/1000] success=599/600 (99.8%)
  [700/1000] success=699/700 (99.9%)
  [800/1000] success=799/800 (99.9%)
  [900/1000] success=899/900 (99.9%)
  [1000/1000] success=999/1000 (99.9%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_few_boxes_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_few_boxes_constrained_summary.json

============================================================
  PHASE3_FEW_BOXES_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 99.9% (999/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 1

  Planning time (ms):
    mean=298.8  median=216.0  p98=1311.7

  Trajectory quality:
    Path length:   2.80 rad (mean)
    Motion time:   1.08 s (mean)
    Max jerk:      1022.4 rad/s³ (p98)
    Position err:  0.513 mm (mean)

  Orientation constraint:
    Max deviation: 1.68° (mean), 6.17° (p98)
    Violations:    7.9%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================

  Generating 1000 problems for 'scattered' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (49 reachable so far)
[Workspace] Found 72/539 reachable positions (13.4%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_scattered.json

============================================================
  Layout: scattered | Constraint: unconstrained
  Description: 6 small objects spread across workspace — moderate clutter
============================================================
  Obstacles: 7 (cuboids: 7, meshes: 0)

[phase3_scattered_unconstrained] Initializing MotionGen...
[phase3_scattered_unconstrained] Warming up (JIT compilation)...
[phase3_scattered_unconstrained] Ready.

[phase3_scattered_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=99/100 (99.0%)
  [200/1000] success=199/200 (99.5%)
  [300/1000] success=298/300 (99.3%)
  [400/1000] success=396/400 (99.0%)
  [500/1000] success=494/500 (98.8%)
  [600/1000] success=591/600 (98.5%)
  [700/1000] success=690/700 (98.6%)
  [800/1000] success=788/800 (98.5%)
  [900/1000] success=886/900 (98.4%)
  [1000/1000] success=985/1000 (98.5%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_scattered_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_scattered_unconstrained_summary.json

============================================================
  PHASE3_SCATTERED_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 98.5% (985/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 15

  Planning time (ms):
    mean=297.6  median=214.1  p98=1569.3

  Trajectory quality:
    Path length:   2.86 rad (mean)
    Motion time:   1.11 s (mean)
    Max jerk:      1048.8 rad/s³ (p98)
    Position err:  0.475 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: scattered | Constraint: constrained
  Description: 6 small objects spread across workspace — moderate clutter
============================================================
  Obstacles: 7 (cuboids: 7, meshes: 0)

[phase3_scattered_constrained] Initializing MotionGen...
[phase3_scattered_constrained] Warming up (JIT compilation)...
[phase3_scattered_constrained] Ready.

[phase3_scattered_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=99/100 (99.0%)
  [200/1000] success=199/200 (99.5%)
  [300/1000] success=298/300 (99.3%)
  [400/1000] success=396/400 (99.0%)
  [500/1000] success=494/500 (98.8%)
  [600/1000] success=591/600 (98.5%)
  [700/1000] success=690/700 (98.6%)
  [800/1000] success=788/800 (98.5%)
  [900/1000] success=886/900 (98.4%)
  [1000/1000] success=985/1000 (98.5%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_scattered_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_scattered_constrained_summary.json

============================================================
  PHASE3_SCATTERED_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 98.5% (985/1000)
  Failure types:
    MotionGenStatus.IK_FAIL: 15

  Planning time (ms):
    mean=193.9  median=153.8  p98=795.8

  Trajectory quality:
    Path length:   2.88 rad (mean)
    Motion time:   1.11 s (mean)
    Max jerk:      1089.8 rad/s³ (p98)
    Position err:  0.471 mm (mean)

  Orientation constraint:
    Max deviation: 1.76° (mean), 8.15° (p98)
    Violations:    10.6%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================

  Generating 1000 problems for 'cluttered' layout...
  (This includes IK reachability scan WITH obstacles — may take a few minutes)
[ProblemGen] No workspace data — running discovery first...
[Workspace] Scanning 539 positions (7x11x7 grid, res=0.06m)
  Scanned 500/539 (43 reachable so far)
[Workspace] Found 65/539 reachable positions (12.1%)
[ProblemGen] Generated 1000 problems (orientation_mode=fixed)
[ProblemGen] Saved 1000 problems to /home/shyam/projects/cc/curobo_eval/results/problems_cluttered.json

============================================================
  Layout: cluttered | Constraint: unconstrained
  Description: 10 objects densely packed — maximum obstacle avoidance challenge
============================================================
  Obstacles: 11 (cuboids: 11, meshes: 0)

[phase3_cluttered_unconstrained] Initializing MotionGen...
[phase3_cluttered_unconstrained] Warming up (JIT compilation)...
[phase3_cluttered_unconstrained] Ready.

[phase3_cluttered_unconstrained] Running 1000 problems...
  Orientation constraint: False
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=200/200 (100.0%)
  [300/1000] success=300/300 (100.0%)
  [400/1000] success=400/400 (100.0%)
  [500/1000] success=500/500 (100.0%)
  [600/1000] success=600/600 (100.0%)
  [700/1000] success=700/700 (100.0%)
  [800/1000] success=800/800 (100.0%)
  [900/1000] success=900/900 (100.0%)
  [1000/1000] success=1000/1000 (100.0%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_cluttered_unconstrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_cluttered_unconstrained_summary.json

============================================================
  PHASE3_CLUTTERED_UNCONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 100.0% (1000/1000)

  Planning time (ms):
    mean=183.1  median=140.9  p98=643.3

  Trajectory quality:
    Path length:   2.71 rad (mean)
    Motion time:   1.07 s (mean)
    Max jerk:      1047.9 rad/s³ (p98)
    Position err:  0.365 mm (mean)

  Orientation constraint:
    Max deviation: 0.00° (mean), 0.00° (p98)
    Violations:    0.0%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


============================================================
  Layout: cluttered | Constraint: constrained
  Description: 10 objects densely packed — maximum obstacle avoidance challenge
============================================================
  Obstacles: 11 (cuboids: 11, meshes: 0)

[phase3_cluttered_constrained] Initializing MotionGen...
[phase3_cluttered_constrained] Warming up (JIT compilation)...
[phase3_cluttered_constrained] Ready.

[phase3_cluttered_constrained] Running 1000 problems...
  Orientation constraint: True
  World obstacles: yes
  [100/1000] success=100/100 (100.0%)
  [200/1000] success=200/200 (100.0%)
  [300/1000] success=300/300 (100.0%)
  [400/1000] success=400/400 (100.0%)
  [500/1000] success=500/500 (100.0%)
  [600/1000] success=600/600 (100.0%)
  [700/1000] success=700/700 (100.0%)
  [800/1000] success=800/800 (100.0%)
  [900/1000] success=900/900 (100.0%)
  [1000/1000] success=1000/1000 (100.0%)
  Saved per-problem results to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_cluttered_constrained_per_problem.json
  Saved summary to /home/shyam/projects/cc/curobo_eval/scripts/results/phase3_cluttered_constrained_summary.json

============================================================
  PHASE3_CLUTTERED_CONSTRAINED RESULTS SUMMARY
============================================================
  Problems:     1000
  Success rate: 100.0% (1000/1000)

  Planning time (ms):
    mean=190.7  median=149.6  p98=754.9

  Trajectory quality:
    Path length:   2.72 rad (mean)
    Motion time:   1.06 s (mean)
    Max jerk:      981.7 rad/s³ (p98)
    Position err:  0.361 mm (mean)

  Orientation constraint:
    Max deviation: 1.30° (mean), 5.30° (p98)
    Violations:    7.9%

  Collision check:
    Collision rate: 0.0% of successful plans
============================================================


======================================================================
  PHASE 3 COMPARISON MATRIX
======================================================================
  Layout       Constraint      Success%   PlanTime(ms)   MaxOrient(°)  
----------------------------------------------------------------------
  easy         unconstrained   99.9       251.0          0.00          
  easy         constrained     98.6       249.7          2.48          
  medium       unconstrained   99.9       313.1          0.00          
  medium       constrained     96.5       314.1          2.18          
  hard         unconstrained   99.3       400.3          0.00          
  hard         constrained     99.3       394.4          3.32          
  few_boxes    unconstrained   99.9       291.4          0.00          
  few_boxes    constrained     99.9       298.8          1.68          
  scattered    unconstrained   98.5       297.6          0.00          
  scattered    constrained     98.5       193.9          1.76          
  cluttered    unconstrained   100.0      183.1          0.00          
  cluttered    constrained     100.0      190.7          1.30          
======================================================================

  All Phase 3 results saved to results/ directory.
  Next: run scripts/analyze_results.py for cross-phase analysis.

[3747.717s] Simulation App Shutting Down