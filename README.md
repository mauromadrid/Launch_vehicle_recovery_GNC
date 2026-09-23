# Launch Vehicle Recovery — Booster Boostback & Powered-Descent Guidance

A Simulink/MATLAB simulation of a reusable first-stage booster returning to a
designated landing site after stage separation: a boostback burn, a
ballistic coast, and a powered landing burn guided by convex-optimization-
based powered descent guidance (lossless-convexified SOCP, in the style of
Açıkmeşe & Ploen's work and the "G-FOLD" family of algorithms).

## Motivation

Built as a portfolio project targeting control/simulation engineering roles
in the Munich area (GNC-focused roles such as Isar Aerospace's working
student positions), to demonstrate: 6DOF/3DOF vehicle dynamics modeling,
convex-optimization-based guidance, Monte Carlo dispersion analysis, and
eventually a Model-in-the-Loop -> Simulink implementation.

## Status

**Phase 1 (done):** planar 3DOF point-mass dynamics for the full
boostback -> coast -> landing sequence.

**Phase 2 (done):** minimum-fuel boostback guidance as a lossless-
convexified SOCP solved with `coneprog` (`guidance/boostback_guidance.m`).
The burn targets a fixed-duration ballistic coast that ends at the
landing-burn ignition point; flown through the nonlinear EOM (MATLAB and
Simulink), the vehicle arrives there within ~0.5 m after a ~97 s burn
and ~205 s coast.

**Phase 3 (next):** convex landing-burn guidance. The landing burn is
still a placeholder, so touchdown is still hard.

## Planned phases

1. **3DOF dynamics** — point-mass planar dynamics, no
   aerodynamics, validated with placeholder guidance.
2. **Convex boostback guidance** — minimum-fuel burn solved as a
   second-order cone program (SOCP) via `coneprog`.
3. **Convex powered-descent guidance (landing)** — full lossless-
   convexification formulation (thrust bounds, pointing constraint,
   glide-slope constraint) via `coneprog`.
4. **Atmosphere & aerodynamics** — add drag (and optionally lift /
   grid-fin control) for the descent phase.
5. **Monte Carlo dispersion analysis** — randomize separation state,
   wind, and mass properties; report landing-accuracy statistics.
6. **Stretch goals** — receding-horizon re-solving (closed-loop
   replanning), a navigation filter (EKF) on noisy sensor data, and a
   Simulink implementation wrapping the validated MATLAB algorithms
   (Model-in-the-Loop), with code generation via Simulink Coder.

## Folder structure

```
params/     vehicle & environment parameters (vehicleParams.m)
dynamics/   equations of motion and ODE event functions (MATLAB prototype)
guidance/   guidance algorithms (convex boostback; landing placeholder)
sim/        top-level MATLAB script that runs the .m-only simulation
simulink/   the Simulink model, built from native blocks, plus the
            script that (re)generates it and a results-plotting script
tests/      MATLAB unit tests (matlab.unittest framework)
results/    generated plots/data (not version-controlled)
```

## Two parallel implementations (by design)

This project deliberately keeps both a plain-MATLAB prototype
(`dynamics/`, `sim/run_open_loop_demo.m`) and a Simulink block-diagram
implementation (`simulink/`) of the same Phase 1 physics:

- The MATLAB version was used first to validate the equations of
  motion quickly (unit-testable, fast to iterate).
- The Simulink version (`simulink/booster_return_3dof.slx`) is the
  actual deliverable — a genuine block diagram built from native
  Simulink blocks (Integrator, Product, Trigonometric Function, Sum,
  Switch, etc.), not a single MATLAB Function block hiding the
  equations. It has two subsystems:
  - **VehicleDynamics** — the planar 3DOF point-mass equations of
    motion as a wired block diagram, mirroring `threeDOF_EOM.m`.
  - **GuidancePhaseLogic** — boostback / coast / landing phase
    sequencing using Switch blocks driven by elapsed time and
    altitude (a Stateflow chart is a natural, higher-value upgrade
    here — planned as a follow-up).
  - The top level logs all signals to the workspace and includes a
    Stop-Simulation block that halts the run at ground contact.

`simulink/build_model.m` is the source of truth for the diagram (it
programmatically constructs the `.slx` from scratch) — re-run it
whenever the model's structure needs to change, rather than hand-editing
the `.slx` and letting the two drift apart.

## Running the Simulink model

```matlab
cd simulink
run_and_plot
```

Or rebuild the model from scratch first with `build_model`, then
`run_and_plot`.

## Running the Phase 1 MATLAB-only demo

```matlab
cd sim
run_open_loop_demo
```

Both the MATLAB and Simulink versions fly the convex boostback profile
open-loop (as a zero-order-hold thrust-acceleration and angle schedule,
`T = aCmd(t) * m`), coast to landing-burn ignition altitude, then run the
placeholder landing burn — so neither lands softly yet (Phase 3).

## Plots

Both runners produce the same figure (`sim/plot_return_trajectory.m`):
trajectory coloured by phase with launch site / landing site / aimed
ignition point, altitude, speed, thrust magnitude (with boostback thrust
bounds), flight-path angle vs thrust angle, mass, and thrust acceleration.

## Choosing the landing site

Set `p.target.x` in `params/vehicleParams.m` (x = 0 is the launch site, so
0 is a return-to-launch-site landing; a large positive x is a downrange
droneship). The boostback guidance aims for whatever is set.

## Falcon Heavy sanity check

```matlab
run sim/validate_falcon_heavy.m   % ~7 min
```

Runs the boostback guidance with Falcon Heavy side-booster parameters
(sourced where public, see `params/falconHeavySideBoosterParams.m`) over an
envelope of BECO states, checking feasibility is monotonic in propellant and
reporting the minimum boostback propellant and burn duration.

## Monte Carlo / sensitivity studies

```matlab
run montecarlo/run_mc_boostback.m
```

A phase-agnostic framework in `montecarlo/`:

| Step | Function | Role |
|---|---|---|
| Catalog | `mc_dispersion_catalog(p)` | Every variable that *can* be dispersed, with default ranges around `p`'s nominal values |
| Select | `mc_select(catalog, names)` | Which variables *are* dispersed in this study |
| Sample | `mc_generate_samples(spec, N, Seed=, Method=)` | N input sets: seeded Latin hypercube (or plain random); uniform or truncated-normal |
| Apply | `mc_apply_sample(p, row)` | One case's parameter set (handles speed/flight-path angle, re-derives `m0` and thrust bounds) |
| Run | `mc_run(p, samples, caseFcn)` | One case per sample; errors captured per case; inputs + metrics in one table |
| Case | `mc_case_boostback(p)` | The only phase-specific part: guidance + nonlinear replay → metrics |
| Analyse | `mc_analyze(results, inputs, outputs)` | Summary stats, input-space map, rank-correlation sensitivity heatmap, histograms |

To study other variables, change `vars` in the run script (any name from the
catalog, or append a catalog row for any other numeric field of `p`). Later
phases reuse everything except the case function. Results are saved to
`results/` (not version-controlled).

## Running the tests

```matlab
runtests('tests')
```

(`test_boostback_guidance` takes ~20 s; the rest are fast.)

## Modeling assumptions (Phase 1)

- Flat-Earth, constant-gravity approximation (`g = g0`). Reasonable for
  the altitude range involved here; revisit with an inverse-square or
  WGS84 gravity model if the trajectory range grows significantly.
- No aerodynamic forces yet (added in Phase 4). In vacuum the coast
  reaches landing-burn ignition altitude at ~1.6 km/s, which the landing
  propellant reserve (`p.landingPropReserve`, sized assuming drag brings
  the stage near terminal velocity) cannot cancel — expected until
  Phase 4.
- All parameters in `vehicleParams.m` are illustrative, order-of-
  magnitude values for a small reusable first stage — not sourced from
  any specific real vehicle's performance data. Refine and justify them
  as the project matures.
