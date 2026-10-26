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

## Current state (full mission works end to end in Simulink: boostback, coast, entry burn, landing with soft touchdown)

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

Both fly the same full mission and agree closely (ignition ~0.3 m,
touchdown 0.83 vs 0.85 m/s): **convex boostback** (Phase 2) aimed by
**`boostback_targeting`** → coast with **drag** (COESA plant) → **entry
burn** sized online at 60 km → drag braking → **landing burn: SCvx
guidance re-planned in flight** (Phase 3) → **soft touchdown on the pad**
(nominal RTLS: 0.83 m/s, x ≈ 0 m, 435 kg left). Results are saved as flight
records and plotted from files (see "Results files and plotting").

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
  grid + `fminbnd`), one SOCP per evaluation (~2–3 s total).
- **Pointing constraint**: boostback uses its own `p.boostbackMaxDeviation`
  (default 90°, a half-space) about n = direction of the impulsive Δv into
  the coast arc. `p.maxTiltAngle` (20°) is the landing burn's, about
  vertical. History: v1 reused the 20° cone about n — fine for RTLS, but for
  downrange landing sites it cost up to ~70% extra fuel and made feasible
  targets infeasible (n is only an estimate; the real vehicle flips freely,
  its limit is attitude *rate*, not a cone). Don't tighten it back without
  a reason.
- **Landing site** is free: `p.target.x` (x = 0 is the launch site / RTLS;
  large x = droneship). Verified −20…400 km, all via the T_coast estimate:
  at the 30° nominal, 6.3 t (−20 km) → 6.1 t (RTLS) → 4.8 t (100 km) →
  2.9 t (200 km) → 0.5 t (400 km). The natural (no-burn) impact point is
  ~365 km; close to it the burn shrinks to a short trim (min-throttle
  impulse is the floor).
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
- **Nominal numbers** (committed params: 30° FPA separation at 1649 m/s,
  RTLS): impulsive Δv estimate 1.62 km/s; burn 85.5 s using 6.1 t, leaving
  1.1 t above the landing reserve; T_coast 256 s; apogee 159 km; arrives
  at the ignition point at 1.79 km/s (vacuum) with a ~1.2 m replay miss.
  History: the original nominal was 1600/400 m/s (14°) — 97 s, 6.9 t, only
  259 kg margin, sitting next to the infeasible corner the speed × FPA
  Monte Carlo found (FPA < ~18° and high speed), hence the move to 30°.
- **Params changed**: the original 3000 kg propellant was infeasible (needs
  ~1.85 km/s impulsive at the original 14° separation, ~1.6 km/s at 30°;
  3000 kg gives 1.55 km/s total). Now 8000 kg, plus
  `p.landingPropReserve = 800` kg and `p.boostbackNodes = 40`.
- **T_coast estimate quality**: within ~1% of the best fixed T_coast at the
  30° nominal (~0.2% at the old 14° one). The low-T/W infeasibility seen in v1 (10000 kg) was caused
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
  3000 m ignition altitude at ~1.8 km/s — no landing burn from there can be
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

### Phase 3: Landing burn guidance (DONE — in Simulink, re-planned in flight, soft touchdown)

`guidance/landing_guidance_scvx.m` (tested: `test_landing_guidance_scvx`).
Decided with the user: **full SCvx with trust region** (not frozen drag
coefficients — at ignition drag ≈ 150 kN vs 200 kN thrust, too strong for
the shortcut); touchdown v = 0; fixed 3 km ignition for v1; no separate
min-landing-error fallback (non-zero virtual control flags infeasibility).

Formulation: same LCvx variables as boostback (u = T/m, σ, z = ln m/m0),
plus drag (guidance model: exponential atmosphere) in nonlinear dynamics;
**normalized time τ ∈ [0,1], burn duration s a decision variable** (time
dilation → no outer burn-time search); landing-engine thrust bounds
(Taylor about the reference z̄); pointing ≤ 20° from vertical; glide slope
|x − x_pad| ≤ y·cot(10°) (2 linear rows in 2-D); r_N = pad, v_N = 0,
m ≥ dry; cost −z_N. SCvx: exact discretization of the dynamics linearized
along the reference (RK4 on x, Φ, B, S per ZOH interval; Jacobians by
central differences); c_k = affine term making the model exact at the
reference; virtual control ν (λ = 1e3, ‖ν‖₁ via auxiliary η); hard box
trust region in scaled units; ratio test ρ = actual/predicted on
J = −z_N + λΣ|defects|.

Gotchas hit:
- Initial guess must satisfy the convex constraints (tilt cone, thrust
  bounds) or the trust-region subproblems are infeasible: Hermite cubic
  path (matches r, v at both ends) + controls projected into the set.
- `coneprog` returns exitflag **−7** on subproblems that are actually
  solved (residuals ~1e-11): accept when output.primal/dualfeasibility
  and dualitygap < 1e-6.
- Accept a step only if predicted > 0 (negative/negative gave a positive
  ratio and accepted worse steps); converge when predicted < 1e-4·|J| and
  max defect < 1e-6 (below that, cost changes are numerical noise).
- Fly the plan as thrust **acceleration** (T = aCmd·m(t)), like boostback.

Results (nominal RTLS ignition: 3 km, 355 m/s, 5225 kg): converges in 5
iterations / 1.8 s; 16.9 s burn, 703 kg; defect 4e-9, relaxation gap 3e-5;
tilt limit, thrust bounds (bang-bang 80–200 kN) and glide slope active.
Replay in the guidance model: lands at 0.3 m/s. **Open-loop in the COESA
plant: hits the ground at ~98 m/s** (exponential density 2–16 % too high
below 3 km → plan expects more drag braking than exists). **Closed-loop
replanning** (re-solve every 1 s from the plant state, plan frozen for the
last 2.5 s): touchdown **5.4 m/s on the pad**. Open items: warm-start
re-solves from the shifted previous plan (cold starts ~6 s each), shrink
the frozen end window / terminal controller, then Simulink integration
(periodic replanning, e.g. MATLAB Function at a fixed rate) and
`mission_setup`/MC hookup.

**Completed (2026-10-26):**
- Closed loop: `sim/fly_landing_closed_loop.m` (MATLAB reference) and the
  Simulink `LandingGuidance` MATLAB Function block (discrete,
  `p.landing.loopPeriod` = 0.1 s) both call `guidance/landing_solve.m`
  (margin + fallback) — via `landing_replan.m` (plain arrays) in Simulink.
  Plan at ignition (descending through 3 km), re-plan every 1 s,
  warm-started (previous plan resampled + mass re-referenced), until the
  last 2.5 s / 30 m. Native blocks fly the plan: lookup tables with data
  from input ports (**port order: input, TABLE, breakpoints** — verified),
  indexed by time since the plan, × mass. Placeholder removed.
- **Root cause of the poor closed-loop landings was the onboard density
  model**: the single exponential was 10–16 % too dense below 3 km → stage
  always faster than planned → re-plans needing more than T_max →
  infeasible (9.5 m/s touchdowns). Replaced everywhere in guidance by an
  **onboard atmosphere table** (`guidance/guidance_density.m`: COESA every
  5 km, log-linear; 0.8 % below 5 km, 2.5 % worst). Expect this to also
  shrink the boostback-targeting miss and the entry burn's q
  under-prediction — `p.entry.limitMargin = 0.10` may now be
  over-conservative (not re-checked yet).
- **Thrust margin** `p.landing.thrustPlanFraction = 0.9` (plan with 90 % of
  T_max, keep authority for re-plans; fallback to 100 % if a re-plan fails).
  Sweep with the old density model: 9.5 → 3.9 m/s (0.85); with the table:
  1.7–1.9 m/s in the MATLAB loop, 0.83 m/s in the full Simulink mission.
- Bug fixed in the closed loop: the re-plan time kept bounding the step
  inside the final window (where it isn't advanced) → 1e-6 s steps for
  seconds (looked like a hang). Now errors on non-advancing steps, has a
  wall-clock limit and Verbose progress.
- Simulink gotchas: MATLAB Function output sizes must be declared
  (`Props.Array.Size`), else lookup tables see size 1; extrinsic calls +
  persistent state need `ChartUpdate = 'DISCRETE'` with a SampleTime.
- Re-plans take ~1–3 s of wall time (4–12 SCvx iterations warm-started), so
  a Simulink mission now takes ~1 min; the MC will be correspondingly
  slower. Speed-ups not done: cap iterations / looser tolerance for
  re-plans, fewer nodes late in the burn.
- MC metrics: touchdownSpeed/Vx/Vy, touchdownMissX, propAtTouchdown,
  nLandingPlans; flight loads now cover the whole flight incl. landing.

### Results files and plotting (decoupled)

User requirement: simulations store data in files; plots read only files.
- `make_flight_record(raw, p)` → record: `signals` table (t + one column per
  catalog signal, SI, VariableUnits/Descriptions, `phase` categorical),
  `limits` (per-signal lower/upper columns; thrust limits follow the active
  engine configuration per phase, NaN while coasting), `events` table,
  `display` (units/scales/styles copied in, so old files plot after catalog
  changes), `meta` (name, source, created, git commit "+modified" if the
  tree was dirty), `params`, `schemaVersion` = 1.
- `raw` = struct t, X, T, theta, ev — simulate_mission's output as is; the
  MATLAB prototype builds the same.
- **Scaling**: new variable = one entry in `sim/flight_signal_defs.m`
  (compute fcn from raw/aux/p, units, display, limits). For new DoFs, add
  the states to `raw` and their signals to the catalog.
- `save_flight_record` → `results/flights/` (gitignored);
  `load_flight_record` (latest / by prefix); `plot_flight_record(src,
  Panels=...)`. `run_and_plot` and `run_open_loop_demo` both save + plot
  from the file. `plot_return_trajectory` was removed.
- Heating limit (display only, user choice): `p.loads.heatFluxMax` = 150 kW/m² on the heat-flux panel and, converted (×√Rn/1.7415e-4), on the heating-index panel. Previously: line only when `p.entry.heatMaxRel` is finite
  AND the record has the no-burn reference peak (not available yet) —
  currently the panel says "no limit active". Open question for the user.
- Monte Carlo cases don't save per-flight records yet (would be one line in
  `mc_case_mission`, but 200 records ≈ hundreds of MB — decide first).

### Monte Carlo framework (built before Phase 3, reuse it)

`montecarlo/` (see README table): catalog → select → sample (seeded LHS) →
apply → run(caseFcn) → analyze. **The Monte Carlo flies the main
simulation, not its own copy of it** (user's explicit requirement):
`mc_case_mission(p)` calls `sim/simulate_mission.m`, which runs the
**Simulink model** with the sampled `p` and only measures the result.
New mission features go into the model / `mission_setup`, and the MC picks
them up automatically. (History: the first case function,
`mc_case_boostback`, chained guidance + MATLAB propagation itself and
silently drifted from the demo when targeting was added — removed.)

**How the Simulink run gets the sampled p**: `simulate_mission` computes
`[bb, ld] = mission_setup(p)` and passes `p`, `bb`, `ld` through a
`Simulink.SimulationInput` (`setVariable`) with
`setModelParameter('InitFcn', '')` — the InitFcn (interactive runs)
would otherwise rebuild them from `vehicleParams()` and discard the
dispersion. Tested (`test_simulate_mission`: the flown initial state and
target are the ones passed in). ~14 s for the first run (compile), ~6 s
per case after; infeasible boostbacks ~12 s. Decisions and why:

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
  a separate `matlab -batch "run('montecarlo/run_mc_mission.m')"` process
  and load the saved .mat, so the interactive session isn't blocked.
  (With PCT, `parsim` over the SimulationInput array would be the upgrade.)
- The saved `results/mc_boostback_*.mat` runs predate drag/entry
  burn/targeting (vacuum, boostback only): valid for what they were, not
  for the current mission.
- Plots are labelled via `mc_labels` (inputs from the stored catalog,
  metrics from a table with units/scaling) — add new metrics there.

**First full-mission MC** (2026-09-30; speed ±15% × FPA 5–45°, 200 LHS
samples each, seed 1, entry qMax 150 kPa; `results/mc_mission_site*`):

| | Launch site (RTLS) | 100 km site |
|---|---|---|
| Feasible | 173/200 (86.5%) | 200/200 |
| Landing margin (P5–P95) | 330–1077 kg, min 248 | 998–2382 kg, min 719 |
| Peak q (P5–P95) | 136–164 kPa, max 171 | 89–157 kPa |
| Peak q ≤ 150 kPa in plant | **22/173** | 101/200 (= the no-burn cases) |
| Miss at ignition | −21…−109 m (P95 88) | −48…−165 m (P95 124) |

- Infeasible corner unchanged from the vacuum study (horizontal speed
  > ~1612 m/s): drag/entry/targeting don't affect boostback feasibility.
- **Every case that needed an entry burn ends above the 150 kPa limit**
  (typically 155–164 kPa): the exponential guidance atmosphere
  under-predicts peak q by ~3–8%. RTLS outliers to 171 kPa are
  propellant-limited (entry burn clamped at the landing reserve, e.g.
  42 kg left for it). **Fixed with `p.entry.limitMargin = 0.10`**
  (burn sized to qMax/1.1; heating limit is relative, so no margin there).
  Check (3 cases, Simulink): peak q 163/154/156 → 148/140/142 kPa, all
  within 150, for +130–170 kg entry propellant (−90–100 kg landing
  margin). 0.10 is slightly conservative at 100 km (~6% under); 0.06–0.08
  would also cover the observed error. Full MC not re-run.
- Misses are **all negative** (systematic, same model error) → calibratable.
  The margin does **not** fix this (misses −48/−63/−60 → −41/−27/−60 m):
  the miss comes from the targeting predictor's exponential atmosphere,
  not the entry-burn limit. Fix = better onboard density model (e.g.
  piecewise exponential), which would also let the margin shrink, or a
  calibrated aim offset.
- 100 km: steeper FPA ⇒ cheaper boostback (ρ −0.74) but higher apogee
  (+0.98) ⇒ bigger entry burn (+0.93); landing margin driven by speed
  (−0.95).
- Uniform sampling ⇒ histograms show spread over the swept range, not
  flight probabilities (Phase 5 needs realistic distributions).

**Flight-loads MC** (2026-10-25; `sim/flight_loads.m`, metrics in
`mc_case_mission`; 2 × 100 missions, same dispersions, limitMargin 0.10;
`results/mc_mission_loads_site*`): window = separation → landing-burn
ignition. RTLS (89/100 feasible): max-Q 119–160 kPa (median 148) at ~11 km,
Mach 2.8–3.2; axial 2.8–4.0 g boostback, 5.9–8.1 g descent; Sutton-Graves
heat flux ~100–158 kW/m² (assumed Rn = 0.75 m, order of magnitude); 84/89
within 150 kPa (5 propellant-limited). 100 km (100/100): max-Q 78–143 kPa,
all within limit. q is ~0.2–0.3 kPa at separation/entry start (thin air) —
independently recomputed from ½ρ‖v‖², identical; the user initially
expected max-Q at entry, explanation: density rises ~2500× from 60 to 11 km.

### Phase 4 as implemented so far: drag in the plant + effect study

Done: drag in both plants; guidance unchanged (still vacuum). Decisions:

- **Atmosphere split**: plant = COESA 1976 (`atmoscoesa` / Aerospace
  Blockset "COESA Atmosphere Model" block); guidance/bias estimator (not
  built yet) = exponential fit `p.atmo.rho0 = 1.4168`, `p.atmo.H = 7008 m`
  (0–80 km least squares, density within ±23% — deliberate model mismatch).
- COESA's extrapolation above 86 km drives temperature negative near
  175 km (NaN speed of sound; MC apogees reach 220 km), so altitude into it
  is clamped at `p.atmo.maxAltitude = 150 km` (density ~3e-14, drag ~0),
  identically in MATLAB and Simulink. The Simulink block's once-per-run
  extrapolation warning is silenced around `sim` in `run_and_plot.m`.
- **CD(Mach)** is illustrative (generic blunt-base-first trend: ~0.8
  subsonic, peak 1.5 at Mach 1.5, ~1.3 hypersonic); diameter 1.5 m.
  Real tail-first data lives in wind-tunnel/CFD databases (DLR CALLISTO,
  EU RETALT). Sensitivity on CD/diameter later via the MC catalog.
- Drag written as a_drag = −k·v, k = ½ρV·CD·A/m: no division by V.
- Existing EOM/guidance tests pin `p.aero.dragEnabled = false` (they test
  vacuum physics / the vacuum plan's consistency); `tests/test_drag.m`
  covers drag. MATLAB and Simulink agree exactly at ignition (checked at
  the 100 km site: 99.93 km, 491.5 m/s, t = 255.9 s).

**Findings** (`sim/study_drag_effect.m`, vacuum plan flown with drag,
30° nominal separation):

| Landing site | Downrange error at ignition (drag − vac) | Speed at 3 km: vac → drag | Ideal landing prop: vac → drag (available) | Peak q | Peak drag decel |
|---|---|---|---|---|---|
| 0 km (RTLS) | +0.08 km | 1789 → 458 m/s (M 1.4) | 2.8 → 0.9 t (1.9 t) | 229 kPa | 9.3 g |
| 100 km | −0.07 km | 1443 → 492 m/s | 2.9 → 1.2 t (3.2 t) | 178 kPa | 6.1 g |
| 200 km | −0.47 km | 1486 → 572 m/s | 3.8 → 1.7 t (5.1 t) | 209 kPa | 5.6 g |
| 300 km | −0.75 km | 1787 → 691 m/s | 5.2 → 2.4 t (6.9 t) | 304 kPa | 6.7 g |

- **Range effect is small (< 1 km)**: the descent is steep (FPA −50° to
  −100° at ignition) and drag acts almost entirely below ~40 km, where
  little horizontal distance remains. So the planned drag *bias
  estimator* (aim-point iteration) matters less than expected for
  targeting — the landing burn's divert can likely absorb it.
- **Speed effect is large**: drag removes 1.0–1.4 km/s, which turns the
  landing burn from impossible (vacuum) to affordable in every case.
- **Loads are the real problem**: peak q 180–300 kPa and 6–9 g, around
  8–11 km at Mach 3–4 — far beyond typical launcher ascent max-q (tens of
  kPa). This is what an **entry burn** is for (next step): cut speed high
  up so the dense-air part of the descent is flown slower.
- Ignition at 3 km is still supersonic (Mach 1.4–2.1) — terminal velocity
  (~250 m/s at sea level for this ballistic coefficient) is not reached.

### Entry burn (Phase 4: sizing, trade study, Simulink — done)

Agreed design: **fixed start altitude** (`p.entry.startAltitude`, 60 km),
retrograde at full thrust (`p.nEnginesEntry`), Δv sized **online at burn
start** from the measured state by `guidance/entry_burn_sizing.m`:
numerical predictor (guidance model: exponential atmosphere, constant
speed of sound `p.atmo.soundSpeed`, the CD table) + `fzero` for the
**minimum Δv meeting each active limit** — peak q (`p.entry.qMax`), peak
heating index √ρ·V³ relative to no burn (`p.entry.heatMaxRel`), speed at
ignition (`p.entry.vIgnitionMax`) — taking the largest. Plant flies it via
`simulate_boostback_coast` (cut off at the planned mass). **On by default**
with `p.entry.qMax = 150e3` (reachable for RTLS; without the burn the
nominal descent peaks at ~229 kPa).

In Simulink (`GuidancePhaseLogic`): a **Triggered Subsystem** fires on the
falling edge of y − startAltitude (zero-crossing located) and runs a
**MATLAB Function block** (`size_entry_burn`) that calls
`entry_cutoff_mass` (→ `entry_burn_sizing`) via `coder.extrinsic`, with
`p` as a non-tunable Parameter-scope input. It holds the cutoff mass; the
burn itself is native blocks (m > mCutoff AND sized → T_max_entry at
atan2(−vy, −vx)). Extrinsic = normal-mode simulation only, not code
generation — the sizing uses ode45/fzero. The wrapper returns one double
so the block's typing stays trivial.

Why size to the limits and not lower (discussed at length): only ~¼–⅓ of
speed removed at 60 km survives to 3 km (drag would remove the rest for
free), so the propellant-optimal burn is the *minimum* that respects the
limits; the landing burn removes the rest one-for-one. Riding q = q_max
continuously is the textbook optimum but infeasible at entry speeds:
holding q constant needs T/m = V²·sinγ/(2H) + g·sinγ − q/β (~14 g at
1.5 km/s), hence short, high-thrust burns high up.

Gotchas hit:
- **`fzero`'s TolX is relative to |x|**: TolX = 0.5 accepted ±50% and
  returned the first secant step. Use ~1e-4.
- Δv is capped at the current speed: "opposite to the velocity" flips
  when V → 0 and the integrator chatters (a 10-min run).

**Trade-study findings** (`sim/study_entry_burn.m`, 1 engine, start 60 km):
- **RTLS**: no burn = 229 kPa. Minimum reachable ≈ 104 kPa (plant) using
  all 1.1 t above the landing reserve → **60 kPa is impossible for RTLS**
  with this vehicle. 150 kPa costs 530 kg (landing margin 690 kg).
- **100 km site** (more propellant left): 60 kPa costs 1.76 t, margin 850 kg.
- Guidance model under-predicts plant peak q by ~3–8% → limits are
  exceeded by that much. Needs a margin (or feedback) before calling a
  limit "met".
- **Start altitude**: lower is cheaper, but only by ~10–16% from 80 → 30 km
  (RTLS: 580 → 474 kg at 150 kPa). The main reason is gravity re-accelerating
  the stage between an early burn and the dense air, not drag "waste" —
  there's little air above 30 km. Fixed 60 km costs ~50 kg vs 30 km.
- Heating-limit and q-limit costs track each other closely (heat 0.63 ≈
  q 150 kPa ≈ 530 kg RTLS).
- The entry burn shifts the ignition point (RTLS: up to +1.3 km; 100 km
  site: ~−0.3 km) because it also slows horizontal speed → handled by
  `boostback_targeting` (below).

### Drag- and entry-burn-aware boostback targeting (done)

`guidance/boostback_targeting.m`: predictor-corrector aim-point bias
around the unchanged convex `boostback_guidance` (only the constant
x_target moves, so it stays an SOCP). Predictor = `predict_descent.m`:
guidance-model coast (`guidance_model_eom.m`, shared with the entry
sizing) → the entry burn the flight computer would size → coast to
ignition. Secant update, first step bias −= miss; tolerance 10 m. With
neither drag nor entry burn it returns `boostback_guidance` unchanged.
Used via `mission_setup` (Simulink InitFcn, `simulate_mission`, hence the
MC) and by the MATLAB demo; the studies (`study_drag_effect`,
`study_entry_burn`) deliberately still use the raw `boostback_guidance`
to measure the unbiased effect.

Results (COESA plant, qMax 150 kPa): converges in 1–2 iterations (~9 s,
≤ 7 kg extra propellant). RTLS + entry burn: plant miss +641 m → −48 m;
100 km site: −93 m → −63 m. The residual ~50–70 m is the exponential-vs-
COESA density model error; a better onboard density model (e.g. piecewise
exponential) would shrink it, but it's already small for the landing burn.

### Phase 4: Atmosphere & aerodynamics (drag) — original design notes

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
- If the MCP tools report "failed to attach to MATLAB session", the MCP
  server is up but the MATLAB desktop session it drives is gone (closed or
  restarted). Fallback that always works: run scripts in a separate
  process, `matlab -batch "script"` (~30 s startup each; note a script
  that calls `clear` wipes the caller's variables).
- `p.target.x` has been kept at 0 (RTLS) in commits while the user
  experiments locally with 100 km — stage only the intended lines of
  `vehicleParams.m` (build the staged blob from HEAD + edits).
