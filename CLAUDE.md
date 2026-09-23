# Launch Vehicle Recovery — Project Context

This file exists so a coding session (Claude Code or otherwise) picking up this
project has the full design context without needing to re-derive decisions
that were already discussed and settled. It supplements `README.md` (which
covers file structure and how to run things) with the *reasoning* behind the
current design and the *next concrete task*.

## Purpose of the project

A portfolio project in Simulink/MATLAB simulating a reusable first-stage
booster's return to a landing site: boostback burn → ballistic coast →
powered landing burn, guided by convex-optimization-based powered descent
guidance (lossless convexification, in the style of Açıkmeşe & Ploen and the
"G-FOLD" family of algorithms actually flown by SpaceX).

Built to demonstrate control/simulation engineering skills for GNC-focused
roles (Munich area; motivating example: Isar Aerospace's working-student GNC
postings, which explicitly mention "GNC algorithms for reusability scenarios,
trajectory optimization, Monte Carlo analysis, MATLAB/Simulink").

## Current state (Phase 2 complete — see "Phase 2 as implemented" below)

Two parallel implementations of the same Phase 1 physics exist, deliberately:

- `dynamics/threeDOF_EOM.m` + `sim/run_open_loop_demo.m` — a plain-MATLAB
  prototype, used to validate the equations of motion quickly (unit-tested
  via `tests/test_threeDOF_EOM.m`).
- `simulink/booster_return_3dof.slx` — the actual deliverable. Built from
  **native Simulink blocks** (Integrator/Product/Trigonometric
  Function/Sum/Switch), not a MATLAB Function block wrapping the equations —
  this was a deliberate choice so the block diagram itself demonstrates
  Simulink modeling skill, not just MATLAB coding skill hidden in a box.
  Generated programmatically by `simulink/build_model.m` (the source of
  truth for the diagram — re-run it to rebuild, don't hand-edit the `.slx`
  and let the two drift apart).

Both fly the **real convex boostback guidance** (Phase 2, done) and then
coast to landing-burn ignition altitude, arriving within ~0.5 m of the
aimed-for point. The landing burn (`guidance/pdg_landing_guidance.m`) is
still a **placeholder** fixed profile, so touchdown is hard. **Phase 3
(convex landing guidance) is the next task** — but read the "vacuum
consequence" note in "Phase 2 as implemented" first.

The project is a git repository (initialized at the Phase 1 baseline).

Params, including vehicle constants, are in `params/vehicleParams.m` — all
values there are illustrative/order-of-magnitude, not sourced from any real
vehicle's performance data.

## Architecture decisions already made (with reasoning)

### Scope

Boostback burn is in scope (not just the landing burn) — the project covers
the full return sequence: boostback → coast → landing.

### Guidance dynamics fidelity is deliberately split from the simulation plant's fidelity

This is important and was discussed at length — **do not conflate these two
things**:

- The **convex guidance/optimizer** always operates on simple **point-mass
  translational dynamics** — thrust magnitude and direction are free control
  variables, no attitude states involved. This keeps the problem a clean
  SOCP solvable via `coneprog`.
- The **simulation plant** can later be upgraded to a higher-fidelity model
  with real rotational dynamics (see "Vendor Aerospace Blockset block" below)
  — but this is a **cascaded architecture**: the convex guidance still
  outputs a desired thrust magnitude/direction; a *separate*, simple attitude
  controller (e.g. PD on pitch) converts "desired direction" into a moment
  command that the higher-fidelity plant needs to physically rotate the
  vehicle. This mirrors real GNC architecture (fast inner attitude-control
  loop, slower outer trajectory-guidance loop) and means **upgrading plant
  fidelity later does NOT require reformulating the optimizer**.
- This split is *not yet implemented* — Phase 1's Simulink model still uses
  the simplified point-mass plant throughout. The attitude-aware plant +
  attitude controller is a separately-scoped fidelity upgrade for later.

### Native Simulink blocks over MATLAB Function block

Chosen deliberately so the block diagram visibly demonstrates the physics,
rather than hiding it in a single black-box function call — this was an
explicit trade-off (more build effort, more legible/idiomatic Simulink
model) made because the point of the project is to demonstrate Simulink
fluency specifically, not just algorithmic correctness.

## Phase 2 as implemented (done)

The design below was implemented as agreed in `guidance/boostback_guidance.m`
+ `guidance/estimate_coast_duration.m`, tested by
`tests/test_boostback_guidance.m`. Details and findings worth remembering:

- **Variables**: Açıkmeşe & Ploen change of variables (u = T/m, σ = Γ/m,
  z = ln(m/m0)) so dynamics are linear; exact ZOH discretization on u;
  thrust bounds via Taylor expansion of e^-z about the max-thrust mass
  profile. Solved solver-based (explicit matrices + `secondordercone`),
  nondimensionalized to km / g units. Fixed-structure on purpose, with the
  later shrinking-horizon replanning / code generation in mind.
- **Burn duration** is not in the convex problem: outer 1D search (25-point geometric
  grid + `fminbnd`), one SOCP per evaluation (~2 s total).
- **Pointing constraint**: boostback uses its own `p.boostbackMaxDeviation`
  (default 90°, a half-space) about n = direction of the impulsive Δv into
  the coast arc. `p.maxTiltAngle` (20°) is the landing burn's, about
  vertical. History: v1 reused the 20° cone about n — fine for RTLS, but for
  downrange landing sites it cost up to ~70% extra fuel and made feasible
  targets infeasible (n is only an estimate; the real vehicle flips freely,
  its limit is attitude *rate*, not a cone). Don't tighten it back without
  a reason.
- **Landing site** is free: `p.target.x` (x = 0 is the launch site / RTLS;
  large x = droneship). Verified −20…400 km, all via the T_coast estimate.
  Near the natural impact point (~300 km) the burn shrinks to a ~1 s trim
  (min-throttle impulse is the floor).
- **Thrust bounds per burn**: `p.nEnginesBoostback` → `p.T_min/T_max_boostback`
  (guidance uses these); `p.T_min/T_max` remain the landing burn's.
- **T_coast fallback**: if the estimated T_coast is infeasible, a scan over
  0.3–2× the estimate (re-deriving n per value) picks the cheapest feasible
  one (`bb.T_coastSource` says which). With the 90° cone it hasn't been
  needed in any tested case, but infeasible cases are slow (~13 s) because
  of it.
- **Output / how it's flown**: ZOH profile of thrust-acceleration magnitude
  `bb.aCmd` and angle `bb.theta` at node times `bb.t`, flown as
  T = aCmd·m with the plant's actual mass (exact replay of the plan). In
  Simulink: 1-D Lookup Tables (flat interpolation) on Clock, times m_in.
- **aCmd = ‖u‖, not σ** (fixed after the first Monte Carlo): flying the slack
  σ over-accelerates by the relaxation gap; a 5e-5 gap became >10 m of miss
  after a long coast.
- **Result**: losslessness gap ≤ ~5e-5; nonlinear replay reaches the ignition
  point within ~1 m nominally (both MATLAB and Simulink), up to ~10 m across
  the speed/FPA Monte Carlo. The residual is coneprog's tolerance on the 40
  dynamics equalities (~0.03 m/s at cutoff) amplified by the coast. If it
  ever matters: a least-norm polish of u against the exact linear
  cutoff-state map (2 equations) would remove it. `coneprog`'s default
  tolerance leaves ~0.2 m on the affine constraint; tightening
  `ConstraintTolerance` to 1e-9 makes it stall (all solves "infeasible").
- **Params changed**: the original 3000 kg propellant was infeasible (needs
  ~1.85 km/s impulsive; had 1.55 km/s total). Now 8000 kg, plus
  `p.landingPropReserve = 800` kg and `p.boostbackNodes = 40`.
- **T_coast estimate quality**: at 8000 kg it's within ~0.2% of the best
  fixed T_coast. The low-T/W infeasibility seen in v1 (10000 kg) was caused
  by the tight pointing cone and is gone with the 90° half-space.
- **Falcon Heavy check** (`sim/validate_falcon_heavy.m`,
  `params/falconHeavySideBoosterParams.m`): BECO state and propellant aren't
  public, so it sweeps an envelope (1.5–1.9 km/s, 25–35° FPA). Feasibility is
  monotonic in propellant everywhere; min boostback propellant 14–21 t
  (~4–5% of the 411 t load). But predicted min-fuel burns are 15–28 s vs the
  observed "just over one minute" — unresolved; likely the real BECO state
  (speed/mass) differs from the envelope and/or the real burn isn't
  min-fuel at full 3-engine thrust. Needs real BECO telemetry to settle.
- **Plots**: `sim/plot_return_trajectory.m` is shared by the MATLAB demo and
  `simulink/run_and_plot.m` (trajectory by phase, altitude, speed, thrust,
  flight-path vs thrust angle, mass, thrust acceleration).
- **Vacuum consequence for Phase 3**: without drag the coast reaches the
  3000 m ignition altitude at ~1.6 km/s — no landing burn from there can be
  feasible. Phase 3 needs either Phase 4 drag first, or a much higher
  ignition point for a vacuum-only demo. Decide before starting Phase 3.

## Phase 2 design (as agreed before implementation): Boostback guidance formulation

### Goal

Take the vehicle from its separation state (position, velocity, mass — all
known/given) through a powered boostback burn, ending in a state such that
the subsequent **unpowered ballistic coast** naturally arcs down onto (or
very near) the landing target. The boostback burn is NOT trying to reach the
landing site itself — it's setting up the coast arc. The landing burn
(Phase 3) handles final precision and soft touchdown.

### Cost function

Minimize fuel consumption. Since ṁ = −T/(Isp·g₀), minimizing ∫|ṁ|dt is
equivalent to minimizing ∫T dt, equivalent to maximizing final mass. In the
lossless-convexification formulation this is typically the integral of the
slack variable Γ (the relaxed thrust-magnitude bound) — a linear cost term,
cheap for the solver.

### Handling the target: fixed coast duration, NOT the raw nonlinear impact-point formula

**Do not** use the closed-form ballistic impact-point formula directly as a
constraint/cost — it involves `t_impact = [vy_f + sqrt(vy_f² + 2·g·y_f)] / g`,
which is non-convex in the state and would break the SOCP.

**Instead**: fix the coast duration `T_coast` as a known parameter (not a
decision variable). Since coast dynamics under constant gravity are linear,
this makes the "burn-cutoff state leads to landing target after coasting"
relationship an **affine** (linear) constraint on the burn-cutoff state
(x_f, y_f, vx_f, vy_f):

```
x_f + vx_f·T_coast = x_target
y_f + vy_f·T_coast − ½·g·T_coast²  = y_target   (or landing-burn ignition altitude)
```

**Start with this as a hard equality constraint** (agreed — simpler to debug
first). A later refinement (not needed for v1): instead of a hard equality,
penalize squared distance from the burn-cutoff state to the affine manifold
of all states that would hit the target after coasting T_coast — this stays
convex (distance-to-affine-set is a convex quadratic) and turns the hard
constraint into a soft, tunable penalty.

### Estimating T_coast

T_coast is sensitive to the (not-yet-known) burn-cutoff velocity, so it can't
just be guessed arbitrarily. Method agreed:

For a candidate T, compute the velocity that *would be needed* (from the known
separation position, ignoring the actual separation velocity for a moment) to
hit the target after coasting exactly T seconds:

```
vx_needed(T) = (x_target − x0) / T
vy_needed(T) = (y_target − y0 + ½·g·T²) / T
```

Then choose T to **minimize** the squared distance between this needed
velocity and the actual known separation velocity:

```
J(T) = (vx_needed(T) − vx_actual)² + (vy_needed(T) − vy_actual)²
```

This is a cheap 1D scalar minimization (e.g. `fminbnd`), not a nonlinear ODE
solve — minimizing required Δv approximately minimizes fuel too, since the
burn is short relative to the coast. This calculation is cheap enough to
redo at every guidance cycle if/when receding-horizon replanning is added
later (unlike the drag-correction approach below, which is NOT cheap enough
for that — see Phase 4).

Known limitation, consistent with current scope (not a new gap): this T_coast
estimate is a vacuum/no-drag calculation, and uses the separation position
rather than the (not-yet-known) true burn-cutoff position as the reference
point — both acceptable first-order approximations for now.

### Implementation notes

- Use MATLAB's `coneprog` (Optimization Toolbox) as the SOCP solver — it's
  natively available (confirmed installed) and avoids needing a third-party
  dependency like CVX, which also happens to be a nice showcase of MathWorks'
  own tooling given MathWorks is one of the target employers for this
  project.
- Constraints to include: thrust magnitude bounds (`p.T_min`, `p.T_max`, via
  the lossless-convexification slack variable Γ), thrust pointing-angle
  bound (`p.maxTiltAngle`), the boundary conditions (initial state = known
  separation state; terminal state = whatever the fixed-T_coast affine
  target constraint requires), and the mass/thrust dynamics (ṁ = −Γ/(Isp·g₀)).
- Discretize the burn duration into a reasonable number of nodes (zero-order
  hold on thrust between nodes is standard).

## Later phases — key points to remember when we get there

### Phase 3: Landing burn guidance

Same convex-SOCP approach as boostback, but the full treatment: minimize
fuel (or landing error), thrust bounds, pointing constraint, AND glide-slope
constraint (stay inside a cone above the landing site so the trajectory
doesn't dive into terrain early). Terminal condition: zero (or near-zero)
position and velocity error at the pad.

### Monte Carlo framework (built before Phase 3, reuse it)

`montecarlo/` (see README table): catalog → select → sample (seeded LHS) →
apply → run(caseFcn) → analyze. **Only the case function is phase-specific**
(`mc_case_boostback` now; write `mc_case_landing` / a full-sequence case
later). Decisions and why:

- Derived params live in `params/deriveParams.m` and are recomputed after
  every dispersion — otherwise dispersing dryMass/thrust would leave m0 and
  the thrust bounds stale.
- Separation velocity is dispersed in polar form ("speed", "fpa"), the way
  trajectory dispersions are usually specified. Position is held nominal
  unless "sep.x"/"sep.y" are selected — real ascent dispersions correlate
  these (a lofted ascent separates higher); correlated sampling isn't
  implemented yet.
- Default distributions are uniform = sensitivity/feasibility *mapping*.
  Phase 5 statistics should switch rows to "normal" (truncated at ±3σ).
- LHS so a few hundred samples cover each variable's range evenly.
- Infeasible is an *outcome* (feasible=false), not an error; genuine
  exceptions are recorded per case (status/message) and don't stop a run.
- No Parallel Computing Toolbox installed → serial loop. Long runs: launch
  a separate `matlab -batch "run('montecarlo/run_mc_boostback.m')"` process
  and load the saved .mat, so the interactive session isn't blocked.
  Infeasible cases cost ~12 s each (the T_coast fallback scan).
- Simulink-based cases (later): the model's InitFcn currently rebuilds `p`
  from vehicleParams(), which would discard dispersions. A Simulink case
  function needs InitFcn to accept an externally supplied `p` (e.g. via
  Simulink.SimulationInput variables) first.

### Phase 4: Atmosphere & aerodynamics (drag)

Drag is a first-order effect at low altitude/high speed, not a rounding
error — don't skip this consideration when the time comes. Two legitimate
approaches were discussed, not yet decided between:

1. **Offline bias-correction table**: precompute (via full nonlinear
   drag-inclusive simulation, offline) a correction/overshoot across a grid
   of representative conditions, fit a cheap function (lookup table /
   regression), evaluate that cheaply online. Simple, works well if the
   family of conditions is bounded (dovetails naturally with the Monte Carlo
   dispersion runs in Phase 5 — same offline runs could build both).
2. **Successive Convexification (SCvx)**: the real, published, more rigorous
   answer (Mao, Szmuk, Açıkmeşe — "Successive Convexification for Fuel-
   Optimal Powered Landing with Aerodynamic Drag"). Linearize dynamics
   (including a drag term) around a reference trajectory, solve the
   resulting convex SOCP, re-linearize around the new solution, repeat for a
   handful of iterations until convergence. Each iteration is still a fast
   convex solve, so this is genuinely real-time-capable — this is the
   literature-standard extension of G-FOLD-style guidance to handle drag,
   and would be the stronger portfolio choice if time allows.

### Phase 5: Monte Carlo dispersion analysis

Randomize separation state, wind (once aero exists), and mass properties
across many runs; report landing-accuracy statistics. This is the most
direct match to real job-posting language ("Monte Carlo analysis").

### Stretch: Receding-horizon replanning

How real systems (SpaceX/JPL) actually do this, confirmed via research
(sources: Blackmore's NAE Bridge article; DLR CALLISTO paper; Szmuk &
Açıkmeşe SCvx paper) — worth mirroring rather than reinventing:

- It is **not** a fixed-length sliding window (classic receding-horizon
  MPC). It's a **shrinking/decreasing horizon**: every re-solve targets the
  fixed landing condition as its terminal constraint, so the target is
  always exactly at the end of whatever horizon is being solved, never
  outside it. No separate terminal-cost approximation is needed.
- Real implementations (SpaceX, via CVXGEN-generated embedded code) re-solve
  the convex problem repeatedly onboard, in a fraction of a second, during
  the terminal landing-burn "divert" phase specifically — not just once
  offline. Convexity's real value here is *reliability* (bounded solve time,
  no local minima) as much as speed — critical for a safety-critical,
  real-time system.
- Computational cost of a "long" remaining horizon is handled by
  **normalizing time**: define τ ∈ [0,1], t = τ·s, where s ("temporal
  dilation factor") is the remaining flight duration, itself a decision
  variable/re-estimated each solve. A **fixed number of discretization
  nodes N** is used over τ regardless of how large s is — so the SOCP has
  identical size/shape at every re-solve (required for CVXGEN-style fixed-
  structure code generation), with only the physical node spacing (s/N)
  changing. Adopt this same normalized-time approach here rather than
  inventing a different scheme.
- Warm-starting from an offline/simplified solve (per the DLR CALLISTO
  paper's "educated guess" approach) is legitimate practice — offline
  computation's role in real systems is initialization/validation, not
  "the flight plan" itself.

### Vendor Aerospace Blockset block (for the later attitude-aware plant upgrade)

Found via `aerolibv1` (Aerospace Blockset's actual library name;
`slblocks.m` maps `Browser.Library = 'aerolibv1'`), under
`Equations of Motion → 3DOF`. Relevant block:
**"Simple Variable Mass 3DOF (Body Axes)"**
(`aerolib3dof2/Simple Variable Mass 3dof (Body Axes)` once the 3DOF
sub-library is opened — it's a locked/protected library block, contents not
introspectable via `find_system`, but its documented interface is):

- Inputs (in order): Fx, Fz (forces in **body axes**, not inertial), M
  (pitching moment), dm/dt (mass flow rate, optional), Vre (relative mass-
  flow velocity, optional), g (optional external gravity).
- Outputs (in order): θ (pitch attitude), q (pitch rate), XeZe (Earth-frame
  position), U,w (body-axis velocity), Fuel (tank status), plus optional g,
  m, I, dI/dt.

Note this block works in **body axes** — using it means the guidance's
"desired thrust angle" output needs to go through the attitude controller
described above (commanded pitch → moment M), not straight into the plant.

## Environment / tooling notes

- Local machine: Windows 11, MATLAB R2026a with Simulink, Aerospace
  Blockset/Toolbox, Optimization Toolbox (has `coneprog` — no need for
  third-party CVX), Simscape Multibody, Simulink Coder, Simulink Test, and
  the MATLAB MCP Server / Simulink Agentic Toolkit installed and connected.
- Project lives at `Documents\MATLAB\Launch_vehicle_recovery` on that
  machine.
- `simulink/build_model.m`'s `InitFcn` makes the model self-contained (it
  derives `p`, `bb`, `ld` from the params/guidance functions at simulation
  start) — if you add new guidance outputs/variables, update the `InitFcn`
  string in `build_model.m` accordingly, and re-run `build_model.m` to
  regenerate the `.slx` rather than hand-editing it.
- One gotcha hit already: MATLAB can cache a stale copy of an edited `.m`
  file even after it's rewritten on disk — if a newly added struct field
  seems to not exist, try `clear functions; rehash` before assuming the file
  edit didn't take.
